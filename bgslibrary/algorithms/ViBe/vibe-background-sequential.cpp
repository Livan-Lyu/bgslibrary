#include <assert.h>
#include <iostream>
#include <stdio.h>
#include <time.h>

#include "vibe-fpga-sim.h"
#include "vibe-fpga-shared.h"
#include "vibe-background-sequential.h"

#define PROFILE_PRINT_INTERVAL 100

//using namespace bgslibrary::algorithms::vibe;
namespace bgslibrary
{
  namespace algorithms
  {
    namespace vibe
    {
      // self adaptive distance threshold.
      // uint32_t distance_Han2014Improved(uint8_t pixel, uint8_t bg)
      // {
      //   uint8_t min, max;

      //   // Computes R = 0.13 min{ max[bg,26], 230}
      //   max = 26;
      //   if (bg > max) { max = bg; }

      //   min = 230;
      //   if (min > max) { min = max; }

      //   return (uint32_t)(0.13*min);
      // }

      // static int abs_uint(const int i)
      // {
      //   return (i >= 0) ? i : -i;
      // }

      static inline uint32_t seeded_sample_count(const uint32_t numberOfSamples)
      {
        return (numberOfSamples < NUMBER_OF_HISTORY_IMAGES)
          ? numberOfSamples
          : NUMBER_OF_HISTORY_IMAGES;
      }

      static inline size_t fpga_shared_foreground_bytes(const uint32_t width, const uint32_t height)
      {
        return static_cast<size_t>(width) * static_cast<size_t>(height);
      }

      struct vibeModel_Sequential
      {
        /* Parameters. */
        uint32_t width;
        uint32_t height;
        uint32_t numberOfSamples;
        uint32_t matchingThreshold;
        uint32_t matchingNumber;
        uint32_t updateFactor;

        /* Storage for the history. */
        uint8_t *historyBuffer;              // Legacy buffer kept for the inactive C1 path.
        uint32_t lastHistorySampleSwapped;   // Legacy promotion state kept for compatibility.
        uint8_t *fpgaSharedMemory;
        size_t fpgaSharedMemoryBytes;
        size_t fpgaSharedForegroundBytes;

        /* Buffers with random values. */
        uint32_t *jump;
        int *neighbor;
        uint32_t *position;
      };

      // -----------------------------------------------------------------------------
      // Profiling: segmented timing
      // -----------------------------------------------------------------------------
      // static inline double elapsed_sec(clock_t t0, clock_t t1) {
      //   return static_cast<double>(t1 - t0) / CLOCKS_PER_SEC;
      // }

      // static double g_c1_seg_clear_mask = 0.0, g_c1_seg_first_hist = 0.0, g_c1_seg_other_hists = 0.0;
      // static double g_c1_seg_buffer_search = 0.0, g_c1_seg_make_output = 0.0;
      // static unsigned long g_c1_seg_frames = 0;

      // static double g_c1_upd_interior = 0.0, g_c1_upd_first_row = 0.0, g_c1_upd_last_row = 0.0;
      // static double g_c1_upd_first_col = 0.0, g_c1_upd_last_col = 0.0, g_c1_upd_first_pixel = 0.0;
      // static unsigned long g_c1_upd_frames = 0;


      // static double g_c3_seg_clear_mask = 0.0, g_c3_seg_first_hist = 0.0, g_c3_seg_other_hists = 0.0;
      // static double g_c3_seg_buffer_search = 0.0, g_c3_seg_make_output = 0.0;
      // static unsigned long g_c3_seg_frames = 0;

      // static double g_c3_upd_interior = 0.0, g_c3_upd_first_row = 0.0, g_c3_upd_last_row = 0.0;
      // static double g_c3_upd_first_col = 0.0, g_c3_upd_last_col = 0.0, g_c3_upd_first_pixel = 0.0;
      // static unsigned long g_c3_upd_frames = 0;

      // /* Inner function timer */
      // static double g_c3_seg_inner_distance_closest = 0.0;

      // -----------------------------------------------------------------------------
      // Print parameters
      // -----------------------------------------------------------------------------
      uint32_t libvibeModel_Sequential_PrintParameters(const vibeModel_Sequential_t *model)
      {
        printf(
          "Using ViBe background subtraction algorithm\n"
          "  - Number of samples per pixel:       %03d\n"
          "  - Number of matches needed:          %03d\n"
          "  - Matching threshold:                %03d\n"
          "  - Model update subsampling factor:   %03d\n",
          libvibeModel_Sequential_GetNumberOfSamples(model),
          libvibeModel_Sequential_GetMatchingNumber(model),
          libvibeModel_Sequential_GetMatchingThreshold(model),
          libvibeModel_Sequential_GetUpdateFactor(model)
        );

        return(0);
      }

      // -----------------------------------------------------------------------------
      // Creates the data structure
      // -----------------------------------------------------------------------------
      vibeModel_Sequential_t *libvibeModel_Sequential_New()
      {
        /* Model structure alloc. */
        vibeModel_Sequential_t *model = NULL;
        model = (vibeModel_Sequential_t*)calloc(1, sizeof(*model));
        assert(model != NULL);

        /* Default parameters values. */
        model->numberOfSamples = fpga::kHistoryFrames;
        model->matchingThreshold = 20;
        model->matchingNumber = 2;
        model->updateFactor = 16;

        /* Storage for the history. */
        model->historyBuffer = NULL;
        model->lastHistorySampleSwapped = 0;
        model->fpgaSharedMemory = NULL;
        model->fpgaSharedMemoryBytes = 0u;
        model->fpgaSharedForegroundBytes = 0u;

        /* Buffers with random values. */
        model->jump = NULL;
        model->neighbor = NULL;
        model->position = NULL;

        return(model);
      }

      // -----------------------------------------------------------------------------
      // Some "Get-ers"
      // -----------------------------------------------------------------------------
      uint32_t libvibeModel_Sequential_GetNumberOfSamples(const vibeModel_Sequential_t *model)
      {
        assert(model != NULL); return(model->numberOfSamples);
      }

      uint32_t libvibeModel_Sequential_GetMatchingNumber(const vibeModel_Sequential_t *model)
      {
        assert(model != NULL); return(model->matchingNumber);
      }

      uint32_t libvibeModel_Sequential_GetMatchingThreshold(const vibeModel_Sequential_t *model)
      {
        assert(model != NULL); return(model->matchingThreshold);
      }

      uint32_t libvibeModel_Sequential_GetUpdateFactor(const vibeModel_Sequential_t *model)
      {
        assert(model != NULL); return(model->updateFactor);
      }

      // int32_t libvibeModel_Sequential_GetHistoryView(
      //   const vibeModel_Sequential_t *model,
      //   vibeHistoryView_Sequential_t *view
      // ) {
      //   if (model == NULL || view == NULL || model->historyBuffer == NULL)
      //     return(-1);

      //   view->historyBuffer = model->historyBuffer;
      //   view->width = model->width;
      //   view->height = model->height;
      //   view->numberOfSamples = model->numberOfSamples;
      //   view->matchingThreshold = model->matchingThreshold;
      //   view->matchingNumber = model->matchingNumber;

      //   return(0);
      // }

      int32_t libvibeModel_Sequential_GetFpgaSharedMemoryView(
        vibeModel_Sequential_t *model,
        vibeFpgaSharedMemoryView_Sequential_t *view
      ) {
        if (model == NULL || view == NULL || model->fpgaSharedMemory == NULL)
          return(-1);

        view->buffer = model->fpgaSharedMemory;
        view->totalBytes = model->fpgaSharedMemoryBytes;
        view->foregroundBytes = model->fpgaSharedForegroundBytes;
        view->width = model->width;
        view->height = model->height;
        view->numberOfSamples = model->numberOfSamples;
        view->matchingThreshold = model->matchingThreshold;
        view->matchingNumber = model->matchingNumber;

        return(0);
      }

      // -----------------------------------------------------------------------------
      // Some "Set-ers"
      // -----------------------------------------------------------------------------
      int32_t libvibeModel_Sequential_SetMatchingThreshold(
        vibeModel_Sequential_t *model,
        const uint32_t matchingThreshold
      ) {
        assert(model != NULL);
        assert(matchingThreshold > 0);

        model->matchingThreshold = matchingThreshold;

        return(0);
      }

      // -----------------------------------------------------------------------------
      int32_t libvibeModel_Sequential_SetMatchingNumber(
        vibeModel_Sequential_t *model,
        const uint32_t matchingNumber
      ) {
        assert(model != NULL);
        assert(matchingNumber > 0);

        model->matchingNumber = matchingNumber;

        return(0);
      }

      // -----------------------------------------------------------------------------
      int32_t libvibeModel_Sequential_SetUpdateFactor(
        vibeModel_Sequential_t *model,
        const uint32_t updateFactor
      ) {
        assert(model != NULL);
        assert(updateFactor > 0);

        model->updateFactor = updateFactor;

        /* We also need to change the values of the jump buffer ! */
        assert(model->jump != NULL);

        /* Shifts. */
        int size = (model->width > model->height) ? 2 * model->width + 1 : 2 * model->height + 1;

        for (int i = 0; i < size; ++i)
          model->jump[i] = (updateFactor == 1) ? 1 : (rand() % (2 * model->updateFactor)) + 1; // 1 or values between 1 and 2 * updateFactor.

        return(0);
      }

      // ----------------------------------------------------------------------------
      // Frees the structure
      // ----------------------------------------------------------------------------
      int32_t libvibeModel_Sequential_Free(vibeModel_Sequential_t *model)
      {
        if (model == NULL)
          return(-1);

        free(model->historyBuffer);
        free(model->fpgaSharedMemory);
        free(model->jump);
        free(model->neighbor);
        free(model->position);
        free(model);

        return(0);
      }

      // // -----------------------------------------------------------------------------
      // // Allocates and initializes a C1R model structure
      // // -----------------------------------------------------------------------------
      // int32_t libvibeModel_Sequential_AllocInit_8u_C1R(
      //   vibeModel_Sequential_t *model,
      //   const uint8_t *image_data,
      //   const uint32_t width,
      //   const uint32_t height
      // ) {
      //   // Some basic checks. */
      //   assert((image_data != NULL) && (model != NULL));
      //   assert((width > 0) && (height > 0));

      //   /* Finish model alloc - parameters values cannot be changed anymore. */
      //   model->width = width;
      //   model->height = height;

      //   /* Stores the full history with the historyBuffer layout:
      //    * each pixel owns numberOfSamples contiguous entries.
      //    */
      //   model->historyBuffer = (uint8_t*)malloc(width * height * model->numberOfSamples * sizeof(*(model->historyBuffer)));
      //   assert(model->historyBuffer != NULL);

      //   const uint32_t seededSamples = seeded_sample_count(model->numberOfSamples);
      //   for (int index = width * height - 1; index >= 0; --index) {
      //     uint8_t value = image_data[index];

      //     for (uint32_t sampleIndex = 0; sampleIndex < model->numberOfSamples; ++sampleIndex) {
      //       if (sampleIndex < seededSamples) {
      //         history_sample_8u_C1R(model->historyBuffer, model->numberOfSamples, index, sampleIndex) = value;
      //       }
      //       else {
      //         int value_plus_noise = value + rand() % 20 - 10;

      //         if (value_plus_noise < 0) { value_plus_noise = 0; }
      //         if (value_plus_noise > 255) { value_plus_noise = 255; }

      //         history_sample_8u_C1R(model->historyBuffer, model->numberOfSamples, index, sampleIndex) = value_plus_noise;
      //       }
      //     }
      //   }

      //   /* Fills the buffers with random values. */
      //   int size = (width > height) ? 2 * width + 1 : 2 * height + 1;

      //   model->jump = (uint32_t*)malloc(size * sizeof(*(model->jump)));
      //   assert(model->jump != NULL);

      //   model->neighbor = (int*)malloc(size * sizeof(*(model->neighbor)));
      //   assert(model->neighbor != NULL);

      //   model->position = (uint32_t*)malloc(size * sizeof(*(model->position)));
      //   assert(model->position != NULL);

      //   for (int i = 0; i < size; ++i) {
      //     model->jump[i] = (rand() % (2 * model->updateFactor)) + 1;            // Values between 1 and 2 * updateFactor.
      //     model->neighbor[i] = ((rand() % 3) - 1) + ((rand() % 3) - 1) * width; // Values between { -width - 1, ... , width + 1 }.
      //     model->position[i] = rand() % (model->numberOfSamples);               // Values between 0 and numberOfSamples - 1.
      //   }

      //   return(0);
      // }

      // // -----------------------------------------------------------------------------
      // // Segmentation of a C1R model
      // // -----------------------------------------------------------------------------
      // int32_t libvibeModel_Sequential_Segmentation_8u_C1R(
      //   vibeModel_Sequential_t *model,
      //   const uint8_t *image_data,
      //   uint8_t *segmentation_map
      // ) {
      //   /* Basic checks. */
      //   assert((image_data != NULL) && (model != NULL) && (segmentation_map != NULL));
      //   assert((model->width > 0) && (model->height > 0));
      //   assert(model->historyBuffer != NULL);
      //   assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

      //   /* Some variables. */
      //   uint32_t width = model->width;
      //   uint32_t height = model->height;
      //   uint32_t matchingNumber = model->matchingNumber;
      //   //uint32_t matchingThreshold = model->matchingThreshold;

      //   uint8_t *historyBuffer = model->historyBuffer;
      //   const uint32_t seededSamples = seeded_sample_count(model->numberOfSamples);

      //   clock_t t0, t1;

      //   /* Segmentation. */
      //   t0 = clock();
      //   memset(segmentation_map, matchingNumber - 1, width * height);
      //   t1 = clock();
      //   g_c1_seg_clear_mask += elapsed_sec(t0, t1);

      //   /* Seeded sample slot 0 is the promoted fast-access sample. */
      //   t0 = clock();
      //   for (int index = width * height - 1; index >= 0; --index) {
      //     const uint8_t sample = history_sample_8u_C1R(historyBuffer, model->numberOfSamples, index, 0);
      //     if (abs_uint(image_data[index] - sample) > distance_Han2014Improved(image_data[index], sample))
      //       segmentation_map[index] = matchingNumber;
      //   }
      //   t1 = clock();
      //   g_c1_seg_first_hist += elapsed_sec(t0, t1);

      //   /* Remaining seeded samples keep the promoted sample semantics. */
      //   t0 = clock();
      //   for (uint32_t sampleIndex = 1; sampleIndex < seededSamples; ++sampleIndex) {
      //     for (int index = width * height - 1; index >= 0; --index) {
      //       const uint8_t sample = history_sample_8u_C1R(historyBuffer, model->numberOfSamples, index, sampleIndex);
      //       if (abs_uint(image_data[index] - sample) <= distance_Han2014Improved(image_data[index], sample))
      //         --segmentation_map[index];
      //     }
      //   }
      //   t1 = clock();
      //   g_c1_seg_other_hists += elapsed_sec(t0, t1);

      //   /* Search the remaining samples stored in the unified history buffer. */
      //   t0 = clock();
      //   const uint32_t swappingSample = (seededSamples == 0)
      //     ? 0u
      //     : ((model->lastHistorySampleSwapped + 1) % seededSamples);
      //   model->lastHistorySampleSwapped = swappingSample;

      //   for (int index = width * height - 1; index >= 0; --index) {
      //     if (segmentation_map[index] > 0) {
      //       uint8_t currentValue = image_data[index];

      //       for (uint32_t sampleIndex = seededSamples; sampleIndex < model->numberOfSamples; ++sampleIndex) {
      //         uint8_t &sample = history_sample_8u_C1R(historyBuffer, model->numberOfSamples, index, sampleIndex);
      //         if (abs_uint(currentValue - sample) <= distance_Han2014Improved(currentValue, sample)) {
      //           --segmentation_map[index];

      //           if (seededSamples > 0) {
      //             uint8_t &promotedSample = history_sample_8u_C1R(historyBuffer, model->numberOfSamples, index, swappingSample);
      //             uint8_t temp = promotedSample;
      //             promotedSample = sample;
      //             sample = temp;
      //           }

      //           if (segmentation_map[index] <= 0) break;
      //         }
      //       }
      //     }
      //   }
      //   t1 = clock();
      //   g_c1_seg_buffer_search += elapsed_sec(t0, t1);

      //   /* Produces the output. Note that this step is application-dependent. */
      //   t0 = clock();
      //   for (uint8_t *mask = segmentation_map; mask < segmentation_map + (width * height); ++mask)
      //     if (*mask > 0) *mask = COLOR_FOREGROUND;
      //   t1 = clock();
      //   g_c1_seg_make_output += elapsed_sec(t0, t1);

      //   ++g_c1_seg_frames;
      //   if (g_c1_seg_frames % PROFILE_PRINT_INTERVAL == 0) {
      //     double total = g_c1_seg_clear_mask + g_c1_seg_first_hist + g_c1_seg_other_hists + g_c1_seg_buffer_search + g_c1_seg_make_output;
      //     if (total > 0) {
      //       printf("[ViBe C1R Seg] over %lu frames:\n", g_c1_seg_frames);
      //       printf("  clear_mask:     %6.2f%%\n", 100.0 * g_c1_seg_clear_mask / total);
      //       printf("  first_hist:     %6.2f%%\n", 100.0 * g_c1_seg_first_hist / total);
      //       printf("  other_hists:    %6.2f%%\n", 100.0 * g_c1_seg_other_hists / total);
      //       printf("  buffer_search:  %6.2f%%\n", 100.0 * g_c1_seg_buffer_search / total);
      //       printf("  make_output:    %6.2f%%\n", 100.0 * g_c1_seg_make_output / total);
      //     }
      //   }

      //   return(0);
      // }

      // // ----------------------------------------------------------------------------
      // // Update a C1R model
      // // ----------------------------------------------------------------------------
      // int32_t libvibeModel_Sequential_Update_8u_C1R(
      //   vibeModel_Sequential_t *model,
      //   const uint8_t *image_data,
      //   uint8_t *updating_mask
      // ) {
      //   /* Basic checks . */
      //   assert((image_data != NULL) && (model != NULL) && (updating_mask != NULL));
      //   assert((model->width > 0) && (model->height > 0));
      //   assert(model->historyBuffer != NULL);
      //   assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

      //   /* Some variables. */
      //   uint32_t width = model->width;
      //   uint32_t height = model->height;

      //   uint8_t *historyBuffer = model->historyBuffer;

      //   /* Updating. */
      //   uint32_t *jump = model->jump;
      //   int *neighbor = model->neighbor;
      //   uint32_t *position = model->position;

      //   clock_t t0, t1;

      //   /* All the frame, except the border. */
      //   uint32_t shift, indX, indY;
      //   unsigned int x, y;

      //   t0 = clock();
      //   for (y = 1; y < height - 1; ++y) {
      //     shift = rand() % width;
      //     indX = jump[shift]; // index_jump should never be zero (> 1).

      //     while (indX < width - 1) {
      //       int index = indX + y * width;

      //       if (updating_mask[index] == COLOR_BACKGROUND) {
      //         /* In-place substitution. */
      //         uint8_t value = image_data[index];
      //         int index_neighbor = index + neighbor[shift];
      //         const uint32_t sampleIndex = position[shift];

      //         history_sample_8u_C1R(historyBuffer, model->numberOfSamples, index, sampleIndex) = value;
      //         history_sample_8u_C1R(historyBuffer, model->numberOfSamples, index_neighbor, sampleIndex) = value;
      //       }

      //       ++shift;
      //       indX += jump[shift];
      //     }
      //   }
      //   t1 = clock();
      //   g_c1_upd_interior += elapsed_sec(t0, t1);

      //   /* First row. */
      //   t0 = clock();
      //   y = 0;
      //   shift = rand() % width;
      //   indX = jump[shift]; // index_jump should never be zero (> 1).

      //   while (indX <= width - 1) {
      //     int index = indX + y * width;

      //     if (updating_mask[index] == COLOR_BACKGROUND) {
      //       history_sample_8u_C1R(historyBuffer, model->numberOfSamples, index, position[shift]) = image_data[index];
      //     }

      //     ++shift;
      //     indX += jump[shift];
      //   }
      //   t1 = clock();
      //   g_c1_upd_first_row += elapsed_sec(t0, t1);

      //   /* Last row. */
      //   t0 = clock();
      //   y = height - 1;
      //   shift = rand() % width;
      //   indX = jump[shift]; // index_jump should never be zero (> 1).

      //   while (indX <= width - 1) {
      //     int index = indX + y * width;

      //     if (updating_mask[index] == COLOR_BACKGROUND) {
      //       history_sample_8u_C1R(historyBuffer, model->numberOfSamples, index, position[shift]) = image_data[index];
      //     }

      //     ++shift;
      //     indX += jump[shift];
      //   }
      //   t1 = clock();
      //   g_c1_upd_last_row += elapsed_sec(t0, t1);

      //   /* First column. */
      //   t0 = clock();
      //   x = 0;
      //   shift = rand() % height;
      //   indY = jump[shift]; // index_jump should never be zero (> 1).

      //   while (indY <= height - 1) {
      //     int index = x + indY * width;

      //     if (updating_mask[index] == COLOR_BACKGROUND) {
      //       history_sample_8u_C1R(historyBuffer, model->numberOfSamples, index, position[shift]) = image_data[index];
      //     }

      //     ++shift;
      //     indY += jump[shift];
      //   }
      //   t1 = clock();
      //   g_c1_upd_first_col += elapsed_sec(t0, t1);

      //   /* Last column. */
      //   t0 = clock();
      //   x = width - 1;
      //   shift = rand() % height;
      //   indY = jump[shift]; // index_jump should never be zero (> 1).

      //   while (indY <= height - 1) {
      //     int index = x + indY * width;

      //     if (updating_mask[index] == COLOR_BACKGROUND) {
      //       history_sample_8u_C1R(historyBuffer, model->numberOfSamples, index, position[shift]) = image_data[index];
      //     }

      //     ++shift;
      //     indY += jump[shift];
      //   }
      //   t1 = clock();
      //   g_c1_upd_last_col += elapsed_sec(t0, t1);

      //   /* The first pixel! */
      //   t0 = clock();
      //   if (rand() % model->updateFactor == 0) {
      //     if (updating_mask[0] == 0) {
      //       int sampleIndex = rand() % model->numberOfSamples;
      //       history_sample_8u_C1R(historyBuffer, model->numberOfSamples, 0, sampleIndex) = image_data[0];
      //     }
      //   }
      //   t1 = clock();
      //   g_c1_upd_first_pixel += elapsed_sec(t0, t1);

      //   ++g_c1_upd_frames;
      //   if (g_c1_upd_frames % PROFILE_PRINT_INTERVAL == 0) {
      //     double total = g_c1_upd_interior + g_c1_upd_first_row + g_c1_upd_last_row + g_c1_upd_first_col + g_c1_upd_last_col + g_c1_upd_first_pixel;
      //     if (total > 0) {
      //       printf("[ViBe C1R Update] over %lu frames:\n", g_c1_upd_frames);
      //       printf("  interior:    %6.2f%%\n", 100.0 * g_c1_upd_interior / total);
      //       printf("  first_row:   %6.2f%%\n", 100.0 * g_c1_upd_first_row / total);
      //       printf("  last_row:    %6.2f%%\n", 100.0 * g_c1_upd_last_row / total);
      //       printf("  first_col:   %6.2f%%\n", 100.0 * g_c1_upd_first_col / total);
      //       printf("  last_col:    %6.2f%%\n", 100.0 * g_c1_upd_last_col / total);
      //       printf("  first_pixel: %6.2f%%\n", 100.0 * g_c1_upd_first_pixel / total);
      //     }
      //   }

      //   return(0);
      // }

      // ----------------------------------------------------------------------------
      // -------------------------- The same for C3R models -------------------------
      // ----------------------------------------------------------------------------

      // -----------------------------------------------------------------------------
      // Allocates and initializes a C3R model structure
      // -----------------------------------------------------------------------------
      int32_t libvibeModel_Sequential_AllocInit_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        const uint32_t width,
        const uint32_t height
      ) {
        /* Some basic checks. */
        assert((image_data != NULL) && (model != NULL));
        assert((width > 0) && (height > 0));

        /* Finish model alloc - parameters values cannot be changed anymore. */
        model->width = width;
        model->height = height;
        model->historyBuffer = NULL;
        model->lastHistorySampleSwapped = 0u;

        model->fpgaSharedForegroundBytes = fpga_shared_foreground_bytes(width, height);
        model->fpgaSharedMemoryBytes = libvibeModel_Sequential_FpgaSharedMemoryBytes_8u_C3R(
          width,
          height,
          model->numberOfSamples
        );
        model->fpgaSharedMemory = (uint8_t*)malloc(model->fpgaSharedMemoryBytes);
        assert(model->fpgaSharedMemory != NULL);

        // Shared memory layout for the C3R/FPGA path:
        // 1) width*height bytes of foreground flags.
        // 2) For each pixel, one BGRX32 current slot followed by N BGRX32 history slots.
        auto sharedSlot = [&](const uint32_t pixelIndex, const uint32_t slotIndex) -> uint8_t* {
          return model->fpgaSharedMemory
            + model->fpgaSharedForegroundBytes
            + (pixelIndex * (model->numberOfSamples + 1u) + slotIndex) * fpga::kInputPixelBytes;
        };
        auto writeBgrx = [](uint8_t *dst, const uint8_t *src) {
          dst[0] = src[0];
          dst[1] = src[1];
          dst[2] = src[2];
          dst[3] = 0u;
        };
        auto writeBgrxValues = [](uint8_t *dst, const uint8_t c0, const uint8_t c1, const uint8_t c2) {
          dst[0] = c0;
          dst[1] = c1;
          dst[2] = c2;
          dst[3] = 0u;
        };
        auto plusNoise = [](uint8_t value) -> uint8_t {
          int value_plus_noise = value + rand() % 20 - 10;
          if (value_plus_noise < 0) { value_plus_noise = 0; }
          if (value_plus_noise > 255) { value_plus_noise = 255; }
          return static_cast<uint8_t>(value_plus_noise);
        };

        memset(model->fpgaSharedMemory, 0, model->fpgaSharedMemoryBytes);

        const uint32_t pixelCount = model->width * model->height;
        const uint32_t seededSamples = seeded_sample_count(model->numberOfSamples);
        for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
          const uint8_t *pixel = image_data + 3u * pixelIndex;

          // Slot 0 mirrors the current frame so the FPGA-side compare function can
          // read a single, uniform per-pixel record block.
          writeBgrx(sharedSlot(pixelIndex, 0u), pixel);

          // History slots are initialized in place and remain the single source of
          // truth for the C3R path after this point.
          for (uint32_t sampleIndex = 0; sampleIndex < model->numberOfSamples; ++sampleIndex) {
            uint8_t *historySlot = sharedSlot(pixelIndex, sampleIndex + 1u);
            if (sampleIndex < seededSamples) {
              writeBgrx(historySlot, pixel);
            }
            else {
              writeBgrxValues(
                historySlot,
                plusNoise(pixel[0]),
                plusNoise(pixel[1]),
                plusNoise(pixel[2])
              );
            }
          }
        }

        /* Fills the buffers with random values. */
        int size = (width > height) ? 2 * width + 1 : 2 * height + 1;

        model->jump = (uint32_t*)malloc(size * sizeof(*(model->jump)));
        assert(model->jump != NULL);

        model->neighbor = (int*)malloc(size * sizeof(*(model->neighbor)));
        assert(model->neighbor != NULL);

        model->position = (uint32_t*)malloc(size * sizeof(*(model->position)));
        assert(model->position != NULL);

        for (int i = 0; i < size; ++i) {
          model->jump[i] = (rand() % (2 * model->updateFactor)) + 1;            // Values between 1 and 2 * updateFactor.
          model->neighbor[i] = ((rand() % 3) - 1) + ((rand() % 3) - 1) * width; // Values between { width - 1, ... , width + 1 }.
          model->position[i] = rand() % (model->numberOfSamples);               // Values between 0 and numberOfSamples - 1.
        }

        return(0);
      }

      // -----------------------------------------------------------------------------
      // Segmentation of a C3R model
      // -----------------------------------------------------------------------------
      int32_t libvibeModel_Sequential_Segmentation_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *segmentation_map
      ) {
        /* Basic checks. */
        assert((image_data != NULL) && (model != NULL) && (segmentation_map != NULL));
        assert((model->width > 0) && (model->height > 0));
        assert(model->fpgaSharedMemory != NULL);
        assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

        /* Some variables. */
        static uint64_t g_c3_seg_debug_frame = 0u;
        uint32_t width = model->width;
        uint32_t height = model->height;
        vibeFpgaSharedMemoryView_Sequential_t fpgaView{};
        auto sharedSlot = [&](const uint32_t pixelIndex, const uint32_t slotIndex) -> uint8_t* {
          return model->fpgaSharedMemory
            + model->fpgaSharedForegroundBytes
            + (pixelIndex * (model->numberOfSamples + 1u) + slotIndex) * fpga::kInputPixelBytes;
        };
        auto writeBgrx = [](uint8_t *dst, const uint8_t *src) {
          dst[0] = src[0];
          dst[1] = src[1];
          dst[2] = src[2];
          dst[3] = 0u;
        };

        // Stage 1: clear the software-visible foreground mask and the shared
        // foreground flag area before preparing the new frame for the FPGA path.
        memset(segmentation_map, COLOR_BACKGROUND, width * height);
        memset(model->fpgaSharedMemory, 0, model->fpgaSharedForegroundBytes);

        // Stage 2: refresh only slot 0 for each pixel. History slots stay in place
        // in shared memory and are updated later by the software update stage.
        const uint32_t pixelCount = width * height;
        for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex)
          writeBgrx(sharedSlot(pixelIndex, 0u), image_data + 3u * pixelIndex);

        // Stage 3: expose the shared buffer to the FPGA simulator, run the
        // compare logic, then read the per-pixel foreground bytes back out.
        libvibeModel_Sequential_GetFpgaSharedMemoryView(model, &fpgaView);
        libvibeModel_Sequential_SimulateFpgaSharedMemory_8u_C3R(&fpgaView);
        libvibeModel_Sequential_ReadFpgaForegroundMask_8u_C1R(&fpgaView, segmentation_map);

        {
          uint32_t foregroundCount = 0u;
          for (uint32_t index = 0; index < pixelCount; ++index) {
            if (segmentation_map[index] == COLOR_FOREGROUND)
              ++foregroundCount;
          }

          std::cerr
            << "[ViBe C3R Seg] frame=" << g_c3_seg_debug_frame
            << " segmentation_foreground=" << foregroundCount
            << " segmentation_background=" << (pixelCount - foregroundCount)
            << std::endl;
        }

        ++g_c3_seg_debug_frame;

        return(0);
      }

      // ----------------------------------------------------------------------------
      // Update a C3R model
      // ----------------------------------------------------------------------------
      int32_t libvibeModel_Sequential_Update_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *updating_mask
      ) {
        /* Basic checks. */
        assert((image_data != NULL) && (model != NULL) && (updating_mask != NULL));
        assert((model->width > 0) && (model->height > 0));
        assert(model->fpgaSharedMemory != NULL);
        assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

        /* Some variables. */
        uint32_t width = model->width;
        uint32_t height = model->height;
        auto historySlot = [&](const uint32_t pixelIndex, const uint32_t sampleIndex) -> uint8_t* {
          return model->fpgaSharedMemory
            + model->fpgaSharedForegroundBytes
            + (pixelIndex * (model->numberOfSamples + 1u) + (sampleIndex + 1u)) * fpga::kInputPixelBytes;
        };
        auto writeHistorySample = [&](const uint32_t pixelIndex, const uint32_t sampleIndex, const uint8_t r, const uint8_t g, const uint8_t b) {
          uint8_t *slot = historySlot(pixelIndex, sampleIndex);
          slot[0] = r;
          slot[1] = g;
          slot[2] = b;
          slot[3] = 0u;
        };

        /* Updating. */
        uint32_t *jump = model->jump;
        int *neighbor = model->neighbor;
        uint32_t *position = model->position;

        /* All the frame, except the border. */
        uint32_t shift, indX, indY;
        int x, y;

        // Update only the history slots in shared memory. Slot 0 is always reserved
        // for the current frame of the next segmentation pass.
        for (y = 1; y < height - 1; ++y) {
          shift = rand() % width;
          indX = jump[shift]; // index_jump should never be zero (> 1).

          while (indX < width - 1) {
            int index = indX + y * width;

            if (updating_mask[index] == COLOR_BACKGROUND) {
              /* In-place substitution. */
              uint8_t r = image_data[3 * index];
              uint8_t g = image_data[3 * index + 1];
              uint8_t b = image_data[3 * index + 2];

              const int neighborIndex = index + neighbor[shift];
              const uint32_t sampleIndex = position[shift];
              writeHistorySample(index, sampleIndex, r, g, b);
              writeHistorySample(neighborIndex, sampleIndex, r, g, b);
            }

            ++shift;
            indX += jump[shift];
          }
        }

        /* First row. */
        y = 0;
        shift = rand() % width;
        indX = jump[shift]; // index_jump should never be zero (> 1).

        while (indX <= width - 1) {
          int index = indX + y * width;

          uint8_t r = image_data[3 * index];
          uint8_t g = image_data[3 * index + 1];
          uint8_t b = image_data[3 * index + 2];

          if (updating_mask[index] == COLOR_BACKGROUND) {
            writeHistorySample(index, position[shift], r, g, b);
          }

          ++shift;
          indX += jump[shift];
        }

        /* Last row. */
        y = height - 1;
        shift = rand() % width;
        indX = jump[shift]; // index_jump should never be zero (> 1).

        while (indX <= width - 1) {
          int index = indX + y * width;

          uint8_t r = image_data[3 * index];
          uint8_t g = image_data[3 * index + 1];
          uint8_t b = image_data[3 * index + 2];

          if (updating_mask[index] == COLOR_BACKGROUND) {
            writeHistorySample(index, position[shift], r, g, b);
          }

          ++shift;
          indX += jump[shift];
        }

        /* First column. */
        x = 0;
        shift = rand() % height;
        indY = jump[shift]; // index_jump should never be zero (> 1).

        while (indY <= height - 1) {
          int index = x + indY * width;

          uint8_t r = image_data[3 * index];
          uint8_t g = image_data[3 * index + 1];
          uint8_t b = image_data[3 * index + 2];

          if (updating_mask[index] == COLOR_BACKGROUND) {
            writeHistorySample(index, position[shift], r, g, b);
          }

          ++shift;
          indY += jump[shift];
        }

        /* Last column. */
        x = width - 1;
        shift = rand() % height;
        indY = jump[shift]; // index_jump should never be zero (> 1).

        while (indY <= height - 1) {
          int index = x + indY * width;

          uint8_t r = image_data[3 * index];
          uint8_t g = image_data[3 * index + 1];
          uint8_t b = image_data[3 * index + 2];

          if (updating_mask[index] == COLOR_BACKGROUND) {
            writeHistorySample(index, position[shift], r, g, b);
          }

          ++shift;
          indY += jump[shift];
        }

        /* The first pixel! */
        if (rand() % model->updateFactor == 0) {
          if (updating_mask[0] == 0) {
            int sampleIndex = rand() % model->numberOfSamples;

            uint8_t r = image_data[0];
            uint8_t g = image_data[1];
            uint8_t b = image_data[2];

            writeHistorySample(0u, sampleIndex, r, g, b);
          }
        }

        return(0);
      }
    }
  }
}

/*
[ViBe C3R Seg] over 300 frames:
  clear_mask:       0.87%
  first_hist:      20.12%
  other_hists:     23.62%
  buffer_search:   50.76%
  make_output:      4.63%

[ViBe C3R Update] over 300 frames:
  interior:      98.44%
  first_row:      0.42%
  last_row:       0.21%
  first_col:      0.43%
  last_col:       0.33%
  first_pixel:    0.16%

[ViBe C3R] Seg vs Update (over 300 frames):
Seg:     82.76%  (0.462284 s total, 1.5409 ms/frame)
Update:  17.24%  (0.096281 s total, 0.3220 ms/frame)
*/
