#include <assert.h>
#include <stdio.h>
#include <time.h>

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
      uint32_t distance_Han2014Improved(uint8_t pixel, uint8_t bg)
      {
        uint8_t min, max;

        // Computes R = 0.13 min{ max[bg,26], 230}
        max = 26;
        if (bg > max) { max = bg; }

        min = 230;
        if (min > max) { min = max; }

        return (uint32_t)(0.13*min);
      }

      static int abs_uint(const int i)
      {
        return (i >= 0) ? i : -i;
      }

      static int32_t distance_is_close_8u_C3R(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2, uint32_t threshold)
      {
        uint32_t sum = abs_uint(r1 - r2) + abs_uint(g1 - g2) + abs_uint(b1 - b2);
        return (sum <= threshold);
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
        uint8_t *historyImage;
        uint8_t *historyBuffer;
        uint32_t lastHistoryImageSwapped;

        /* Buffers with random values. */
        uint32_t *jump;
        int *neighbor;
        uint32_t *position;
      };

      // -----------------------------------------------------------------------------
      // Profiling: segmented timing
      // -----------------------------------------------------------------------------
      static inline double elapsed_sec(clock_t t0, clock_t t1) {
        return static_cast<double>(t1 - t0) / CLOCKS_PER_SEC;
      }

      static double g_c1_seg_clear_mask = 0.0, g_c1_seg_first_hist = 0.0, g_c1_seg_other_hists = 0.0;
      static double g_c1_seg_buffer_search = 0.0, g_c1_seg_make_output = 0.0;
      static unsigned long g_c1_seg_frames = 0;

      static double g_c1_upd_interior = 0.0, g_c1_upd_first_row = 0.0, g_c1_upd_last_row = 0.0;
      static double g_c1_upd_first_col = 0.0, g_c1_upd_last_col = 0.0, g_c1_upd_first_pixel = 0.0;
      static unsigned long g_c1_upd_frames = 0;

      static double g_c3_seg_clear_mask = 0.0, g_c3_seg_first_hist = 0.0, g_c3_seg_other_hists = 0.0;
      static double g_c3_seg_buffer_search = 0.0, g_c3_seg_make_output = 0.0;
      static unsigned long g_c3_seg_frames = 0;

      static double g_c3_upd_interior = 0.0, g_c3_upd_first_row = 0.0, g_c3_upd_last_row = 0.0;
      static double g_c3_upd_first_col = 0.0, g_c3_upd_last_col = 0.0, g_c3_upd_first_pixel = 0.0;
      static unsigned long g_c3_upd_frames = 0;

      /* Inner function timer */
      static double g_c3_seg_inner_distance_closest = 0.0;

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
        model->historyImage = NULL;
        model->historyBuffer = NULL;
        model->lastHistoryImageSwapped = 0;

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

        if (model->historyBuffer == NULL) {
          free(model);
          return(0);
        }

        free(model->historyImage);
        free(model->historyBuffer);
        free(model->jump);
        free(model->neighbor);
        free(model->position);
        free(model);

        return(0);
      }

      // -----------------------------------------------------------------------------
      // Allocates and initializes a C1R model structure
      // -----------------------------------------------------------------------------
      int32_t libvibeModel_Sequential_AllocInit_8u_C1R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        const uint32_t width,
        const uint32_t height
      ) {
        // Some basic checks. */
        assert((image_data != NULL) && (model != NULL));
        assert((width > 0) && (height > 0));

        /* Finish model alloc - parameters values cannot be changed anymore. */
        model->width = width;
        model->height = height;

        /* Creates the historyImage structure. */
        model->historyImage = NULL;
        model->historyImage = (uint8_t*)malloc(NUMBER_OF_HISTORY_IMAGES * width * height * sizeof(*(model->historyImage)));

        assert(model->historyImage != NULL);

        for (int i = 0; i < NUMBER_OF_HISTORY_IMAGES; ++i) {
          for (int index = width * height - 1; index >= 0; --index)
            model->historyImage[i * width * height + index] = image_data[index];
        }

        /* Now creates and fills the history buffer. */
        model->historyBuffer = (uint8_t*)malloc(width * height * (model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES) * sizeof(uint8_t));
        assert(model->historyBuffer != NULL);

        for (int index = width * height - 1; index >= 0; --index) {
          uint8_t value = image_data[index];

          for (int x = 0; x < model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES; ++x) {
            int value_plus_noise = value + rand() % 20 - 10;

            if (value_plus_noise < 0) { value_plus_noise = 0; }
            if (value_plus_noise > 255) { value_plus_noise = 255; }

            model->historyBuffer[index * (model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES) + x] = value_plus_noise;
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
          model->neighbor[i] = ((rand() % 3) - 1) + ((rand() % 3) - 1) * width; // Values between { -width - 1, ... , width + 1 }.
          model->position[i] = rand() % (model->numberOfSamples);               // Values between 0 and numberOfSamples - 1.
        }

        return(0);
      }

      // -----------------------------------------------------------------------------
      // Segmentation of a C1R model
      // -----------------------------------------------------------------------------
      int32_t libvibeModel_Sequential_Segmentation_8u_C1R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *segmentation_map
      ) {
        /* Basic checks. */
        assert((image_data != NULL) && (model != NULL) && (segmentation_map != NULL));
        assert((model->width > 0) && (model->height > 0));
        assert(model->historyBuffer != NULL);
        assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

        /* Some variables. */
        uint32_t width = model->width;
        uint32_t height = model->height;
        uint32_t matchingNumber = model->matchingNumber;
        //uint32_t matchingThreshold = model->matchingThreshold;

        uint8_t *historyImage = model->historyImage;
        uint8_t *historyBuffer = model->historyBuffer;

        clock_t t0, t1;

        /* Segmentation. */
        t0 = clock();
        memset(segmentation_map, matchingNumber - 1, width * height);
        t1 = clock();
        g_c1_seg_clear_mask += elapsed_sec(t0, t1);

        /* First history Image structure. */
        t0 = clock();
        for (int index = width * height - 1; index >= 0; --index) {
          //if (abs_uint(image_data[index] - historyImage[index]) > matchingThreshold)
          if (abs_uint(image_data[index] - historyImage[index]) > distance_Han2014Improved(image_data[index], historyImage[index]))
            segmentation_map[index] = matchingNumber;
        }
        t1 = clock();
        g_c1_seg_first_hist += elapsed_sec(t0, t1);

        /* Next historyImages. */
        t0 = clock();
        for (int i = 1; i < NUMBER_OF_HISTORY_IMAGES; ++i) {
          uint8_t *pels = historyImage + i * width * height;

          for (int index = width * height - 1; index >= 0; --index) {
            // if (abs_uint(image_data[index] - pels[index]) <= matchingThreshold)
            if (abs_uint(image_data[index] - pels[index]) <= distance_Han2014Improved(image_data[index], pels[index]))
              --segmentation_map[index];
          }
        }
        t1 = clock();
        g_c1_seg_other_hists += elapsed_sec(t0, t1);

        /* For swapping. */
        t0 = clock();
        model->lastHistoryImageSwapped = (model->lastHistoryImageSwapped + 1) % NUMBER_OF_HISTORY_IMAGES;
        uint8_t *swappingImageBuffer = historyImage + (model->lastHistoryImageSwapped) * width * height;

        /* Now, we move in the buffer and leave the historyImages. */
        int numberOfTests = (model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES);

        for (int index = width * height - 1; index >= 0; --index) {
          if (segmentation_map[index] > 0) {
            /* We need to check the full border and swap values with the first or second historyImage.
             * We still need to find a match before we can stop our search.
             */
            uint32_t indexHistoryBuffer = index * numberOfTests;
            uint8_t currentValue = image_data[index];

            for (int i = numberOfTests; i > 0; --i, ++indexHistoryBuffer) {
              // if (abs_uint(currentValue - historyBuffer[indexHistoryBuffer]) <= matchingThreshold) {
              if (abs_uint(currentValue - historyBuffer[indexHistoryBuffer]) <= distance_Han2014Improved(currentValue, historyBuffer[indexHistoryBuffer])) {
                --segmentation_map[index];

                /* Swaping: Putting found value in history image buffer. */
                uint8_t temp = swappingImageBuffer[index];
                swappingImageBuffer[index] = historyBuffer[indexHistoryBuffer];
                historyBuffer[indexHistoryBuffer] = temp;

                /* Exit inner loop. */
                if (segmentation_map[index] <= 0) break;
              }
            } // for
          } // if
        } // for
        t1 = clock();
        g_c1_seg_buffer_search += elapsed_sec(t0, t1);

        /* Produces the output. Note that this step is application-dependent. */
        t0 = clock();
        for (uint8_t *mask = segmentation_map; mask < segmentation_map + (width * height); ++mask)
          if (*mask > 0) *mask = COLOR_FOREGROUND;
        t1 = clock();
        g_c1_seg_make_output += elapsed_sec(t0, t1);

        ++g_c1_seg_frames;
        if (g_c1_seg_frames % PROFILE_PRINT_INTERVAL == 0) {
          double total = g_c1_seg_clear_mask + g_c1_seg_first_hist + g_c1_seg_other_hists + g_c1_seg_buffer_search + g_c1_seg_make_output;
          if (total > 0) {
            printf("[ViBe C1R Seg] over %lu frames:\n", g_c1_seg_frames);
            printf("  clear_mask:     %6.2f%%\n", 100.0 * g_c1_seg_clear_mask / total);
            printf("  first_hist:     %6.2f%%\n", 100.0 * g_c1_seg_first_hist / total);
            printf("  other_hists:    %6.2f%%\n", 100.0 * g_c1_seg_other_hists / total);
            printf("  buffer_search:  %6.2f%%\n", 100.0 * g_c1_seg_buffer_search / total);
            printf("  make_output:    %6.2f%%\n", 100.0 * g_c1_seg_make_output / total);
          }
        }

        return(0);
      }

      // ----------------------------------------------------------------------------
      // Update a C1R model
      // ----------------------------------------------------------------------------
      int32_t libvibeModel_Sequential_Update_8u_C1R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *updating_mask
      ) {
        /* Basic checks . */
        assert((image_data != NULL) && (model != NULL) && (updating_mask != NULL));
        assert((model->width > 0) && (model->height > 0));
        assert(model->historyBuffer != NULL);
        assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

        /* Some variables. */
        uint32_t width = model->width;
        uint32_t height = model->height;

        uint8_t *historyImage = model->historyImage;
        uint8_t *historyBuffer = model->historyBuffer;

        /* Some utility variable. */
        int numberOfTests = (model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES);

        /* Updating. */
        uint32_t *jump = model->jump;
        int *neighbor = model->neighbor;
        uint32_t *position = model->position;

        clock_t t0, t1;

        /* All the frame, except the border. */
        uint32_t shift, indX, indY;
        unsigned int x, y;

        t0 = clock();
        for (y = 1; y < height - 1; ++y) {
          shift = rand() % width;
          indX = jump[shift]; // index_jump should never be zero (> 1).

          while (indX < width - 1) {
            int index = indX + y * width;

            if (updating_mask[index] == COLOR_BACKGROUND) {
              /* In-place substitution. */
              uint8_t value = image_data[index];
              int index_neighbor = index + neighbor[shift];

              if (position[shift] < NUMBER_OF_HISTORY_IMAGES) {
                historyImage[index + position[shift] * width * height] = value;
                historyImage[index_neighbor + position[shift] * width * height] = value;
              }
              else {
                int pos = position[shift] - NUMBER_OF_HISTORY_IMAGES;
                historyBuffer[index * numberOfTests + pos] = value;
                historyBuffer[index_neighbor * numberOfTests + pos] = value;
              }
            }

            ++shift;
            indX += jump[shift];
          }
        }
        t1 = clock();
        g_c1_upd_interior += elapsed_sec(t0, t1);

        /* First row. */
        t0 = clock();
        y = 0;
        shift = rand() % width;
        indX = jump[shift]; // index_jump should never be zero (> 1).

        while (indX <= width - 1) {
          int index = indX + y * width;

          if (updating_mask[index] == COLOR_BACKGROUND) {
            if (position[shift] < NUMBER_OF_HISTORY_IMAGES)
              historyImage[index + position[shift] * width * height] = image_data[index];
            else {
              int pos = position[shift] - NUMBER_OF_HISTORY_IMAGES;
              historyBuffer[index * numberOfTests + pos] = image_data[index];
            }
          }

          ++shift;
          indX += jump[shift];
        }
        t1 = clock();
        g_c1_upd_first_row += elapsed_sec(t0, t1);

        /* Last row. */
        t0 = clock();
        y = height - 1;
        shift = rand() % width;
        indX = jump[shift]; // index_jump should never be zero (> 1).

        while (indX <= width - 1) {
          int index = indX + y * width;

          if (updating_mask[index] == COLOR_BACKGROUND) {
            if (position[shift] < NUMBER_OF_HISTORY_IMAGES)
              historyImage[index + position[shift] * width * height] = image_data[index];
            else {
              int pos = position[shift] - NUMBER_OF_HISTORY_IMAGES;
              historyBuffer[index * numberOfTests + pos] = image_data[index];
            }
          }

          ++shift;
          indX += jump[shift];
        }
        t1 = clock();
        g_c1_upd_last_row += elapsed_sec(t0, t1);

        /* First column. */
        t0 = clock();
        x = 0;
        shift = rand() % height;
        indY = jump[shift]; // index_jump should never be zero (> 1).

        while (indY <= height - 1) {
          int index = x + indY * width;

          if (updating_mask[index] == COLOR_BACKGROUND) {
            if (position[shift] < NUMBER_OF_HISTORY_IMAGES)
              historyImage[index + position[shift] * width * height] = image_data[index];
            else {
              int pos = position[shift] - NUMBER_OF_HISTORY_IMAGES;
              historyBuffer[index * numberOfTests + pos] = image_data[index];
            }
          }

          ++shift;
          indY += jump[shift];
        }
        t1 = clock();
        g_c1_upd_first_col += elapsed_sec(t0, t1);

        /* Last column. */
        t0 = clock();
        x = width - 1;
        shift = rand() % height;
        indY = jump[shift]; // index_jump should never be zero (> 1).

        while (indY <= height - 1) {
          int index = x + indY * width;

          if (updating_mask[index] == COLOR_BACKGROUND) {
            if (position[shift] < NUMBER_OF_HISTORY_IMAGES)
              historyImage[index + position[shift] * width * height] = image_data[index];
            else {
              int pos = position[shift] - NUMBER_OF_HISTORY_IMAGES;
              historyBuffer[index * numberOfTests + pos] = image_data[index];
            }
          }

          ++shift;
          indY += jump[shift];
        }
        t1 = clock();
        g_c1_upd_last_col += elapsed_sec(t0, t1);

        /* The first pixel! */
        t0 = clock();
        if (rand() % model->updateFactor == 0) {
          if (updating_mask[0] == 0) {
            int position = rand() % model->numberOfSamples;

            if (position < NUMBER_OF_HISTORY_IMAGES)
              historyImage[position * width * height] = image_data[0];
            else {
              int pos = position - NUMBER_OF_HISTORY_IMAGES;
              historyBuffer[pos] = image_data[0];
            }
          }
        }
        t1 = clock();
        g_c1_upd_first_pixel += elapsed_sec(t0, t1);

        ++g_c1_upd_frames;
        if (g_c1_upd_frames % PROFILE_PRINT_INTERVAL == 0) {
          double total = g_c1_upd_interior + g_c1_upd_first_row + g_c1_upd_last_row + g_c1_upd_first_col + g_c1_upd_last_col + g_c1_upd_first_pixel;
          if (total > 0) {
            printf("[ViBe C1R Update] over %lu frames:\n", g_c1_upd_frames);
            printf("  interior:    %6.2f%%\n", 100.0 * g_c1_upd_interior / total);
            printf("  first_row:   %6.2f%%\n", 100.0 * g_c1_upd_first_row / total);
            printf("  last_row:    %6.2f%%\n", 100.0 * g_c1_upd_last_row / total);
            printf("  first_col:   %6.2f%%\n", 100.0 * g_c1_upd_first_col / total);
            printf("  last_col:    %6.2f%%\n", 100.0 * g_c1_upd_last_col / total);
            printf("  first_pixel: %6.2f%%\n", 100.0 * g_c1_upd_first_pixel / total);
          }
        }

        return(0);
      }

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

        /* Creates the historyImage structure. */
        model->historyImage = NULL;
        model->historyImage = (uint8_t*)malloc(NUMBER_OF_HISTORY_IMAGES * (3 * width) * height * sizeof(uint8_t));
        assert(model->historyImage != NULL);

        for (int i = 0; i < NUMBER_OF_HISTORY_IMAGES; ++i) {
          for (int index = (3 * width) * height - 1; index >= 0; --index)
            model->historyImage[i * (3 * width) * height + index] = image_data[index];
        }

        assert(model->historyImage != NULL);

        /* Now creates and fills the history buffer. */
        model->historyBuffer = (uint8_t *)malloc((3 * width) * height * (model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES) * sizeof(uint8_t));
        assert(model->historyBuffer != NULL);

        auto plus_noise = [](uint8_t value) -> int {int value_plus_noise = value + rand() % 20 - 10;
          if (value_plus_noise < 0) { value_plus_noise = 0; }
          if (value_plus_noise > 255) { value_plus_noise = 255; }
          return value_plus_noise;};
        for(int index = (3 * width) * height - 1; index >= 0; index -= 3) {
          uint8_t value_1 = image_data[index];
          uint8_t value_2 = image_data[index - 1];
          uint8_t value_3 = image_data[index - 2];
          for(uint32_t x = 0; x < model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES; ++x) {
            int value_plus_noise1 = plus_noise(value_1);
            int value_plus_noise2 = plus_noise(value_2);
            int value_plus_noise3 = plus_noise(value_3);
            model->historyBuffer[(index - 2) * (model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES) + x * 3 + 2] = value_plus_noise1;
            model->historyBuffer[(index - 2) * (model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES) + x * 3 + 1] = value_plus_noise2;
            model->historyBuffer[(index - 2) * (model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES) + x * 3] = value_plus_noise3;
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
        assert(model->historyBuffer != NULL);
        assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

        /* Some variables. */
        uint32_t width = model->width;
        uint32_t height = model->height;
        uint32_t matchingNumber = model->matchingNumber;
        uint32_t matchingThreshold = model->matchingThreshold;

        uint8_t *historyImage = model->historyImage;
        uint8_t *historyBuffer = model->historyBuffer;

        clock_t t0, t1;

        /* Segmentation. */
        t0 = clock();
        memset(segmentation_map, matchingNumber - 1, width * height);
        t1 = clock();
        g_c3_seg_clear_mask += elapsed_sec(t0, t1); // clear_mask: 0.87%

        /* First history Image structure. */
        t0 = clock();
        uint8_t *first = historyImage;

        for (int index = width * height - 1; index >= 0; --index) {
          if (
            !distance_is_close_8u_C3R(
              image_data[3 * index], image_data[3 * index + 1], image_data[3 * index + 2],
              first[3 * index], first[3 * index + 1], first[3 * index + 2], matchingThreshold
            )
            )
            segmentation_map[index] = matchingNumber; // 2
        }
        t1 = clock();
        g_c3_seg_first_hist += elapsed_sec(t0, t1); //first_hist: 20.12%

        /* Next historyImages. */
        t0 = clock();
        for (int i = 1; i < NUMBER_OF_HISTORY_IMAGES; ++i) {
          uint8_t *pels = historyImage + i * (3 * width) * height;

          for (int index = width * height - 1; index >= 0; --index) {
            if (
              distance_is_close_8u_C3R(
                image_data[3 * index], image_data[3 * index + 1], image_data[3 * index + 2],
                pels[3 * index], pels[3 * index + 1], pels[3 * index + 2], matchingThreshold
              )
              )
              --segmentation_map[index];
          }
        }
        t1 = clock();
        g_c3_seg_other_hists += elapsed_sec(t0, t1); //other_hists: 23.62%

        /*========================================= buffer searching ========================================*/
        // For swapping
        t0 = clock();
        model->lastHistoryImageSwapped = (model->lastHistoryImageSwapped + 1) % NUMBER_OF_HISTORY_IMAGES;
        uint8_t *swappingImageBuffer = historyImage + (model->lastHistoryImageSwapped) * (3 * width) * height;

        // Now, we move in the buffer and leave the historyImages
        int numberOfTests = (model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES); // 20 frames are compared in total

        for (int index = width * height - 1; index >= 0; --index) {
          if (segmentation_map[index] > 0) {
            /* We need to check the full border and swap values with the first or second historyImage.
             * We still need to find a match before we can stop our search.
             */
            uint32_t indexHistoryBuffer = (3 * index) * numberOfTests;

            for (int i = numberOfTests; i > 0; --i, indexHistoryBuffer += 3) {
              if (
                distance_is_close_8u_C3R(
                  image_data[(3 * index)], image_data[(3 * index) + 1], image_data[(3 * index) + 2],
                  historyBuffer[indexHistoryBuffer], historyBuffer[indexHistoryBuffer + 1], historyBuffer[indexHistoryBuffer + 2],
                  matchingThreshold
                )
                )
                --segmentation_map[index];
              //g_c3_seg_inner_distance_closest += elapsed_sec(t0, t1); // inner_distance_closest_function

              /* Swaping: Putting found value in history image buffer. */
              uint8_t temp_r = swappingImageBuffer[(3 * index)];
              uint8_t temp_g = swappingImageBuffer[(3 * index) + 1];
              uint8_t temp_b = swappingImageBuffer[(3 * index) + 2];

              swappingImageBuffer[(3 * index)] = historyBuffer[indexHistoryBuffer];
              swappingImageBuffer[(3 * index) + 1] = historyBuffer[indexHistoryBuffer + 1];
              swappingImageBuffer[(3 * index) + 2] = historyBuffer[indexHistoryBuffer + 2];

              historyBuffer[indexHistoryBuffer] = temp_r;
              historyBuffer[indexHistoryBuffer + 1] = temp_g;
              historyBuffer[indexHistoryBuffer + 2] = temp_b;

              /* Exit inner loop. */
              if (segmentation_map[index] <= 0) break;
            } // for
          } // if
        } // for
        /*===============================================================================================*/

        t1 = clock();
        g_c3_seg_buffer_search += elapsed_sec(t0, t1); // buffer_search: 50.76%

        /* Produces the output. Note that this step is application-dependent. */
        t0 = clock();
        for (uint8_t *mask = segmentation_map; mask < segmentation_map + (width * height); ++mask)
          if (*mask > 0) *mask = COLOR_FOREGROUND;
        t1 = clock();
        g_c3_seg_make_output += elapsed_sec(t0, t1); // make_output: 4.63%

      /*============================================ Profiling. =================================================*/
        ++g_c3_seg_frames;
        if (g_c3_seg_frames % PROFILE_PRINT_INTERVAL == 0) {
          double total_seg = g_c3_seg_clear_mask + g_c3_seg_first_hist + g_c3_seg_other_hists + g_c3_seg_buffer_search + g_c3_seg_make_output;
          double total_upd = g_c3_upd_interior + g_c3_upd_first_row + g_c3_upd_last_row + g_c3_upd_first_col + g_c3_upd_last_col + g_c3_upd_first_pixel;
          double total_both = total_seg + total_upd;

          if (total_seg > 0) {
            printf("[ViBe C3R Seg] over %lu frames:\n", g_c3_seg_frames);
            printf("  clear_mask:     %6.2f%%\n", 100.0 * g_c3_seg_clear_mask / total_seg);
            printf("  first_hist:     %6.2f%%\n", 100.0 * g_c3_seg_first_hist / total_seg);
            printf("  other_hists:    %6.2f%%\n", 100.0 * g_c3_seg_other_hists / total_seg);
            printf("  buffer_search:  %6.2f%%\n", 100.0 * g_c3_seg_buffer_search / total_seg);
            printf("  make_output:    %6.2f%%\n", 100.0 * g_c3_seg_make_output / total_seg);
            printf("  buffer_search:  %6.2f%% ms/frame\n", g_c3_seg_buffer_search / g_c3_seg_frames);
            //printf("  inner funtion of buffer search: %6.2f%%\n", 100.0 * g_c3_seg_inner_distance_closest / g_c3_seg_frames * loop);
          }
          if (total_both > 0) { // Since the segmentation and update functions are separate, put this print in one of them. And since the time is total time of hundreds of frames, so the error of one frame is negligible.
            printf("[ViBe C3R] Seg vs Update (over %lu frames):\n", g_c3_seg_frames);
            printf("  Seg:    %6.2f%%  (%.6f s total, %.4f ms/frame)\n", 100.0 * total_seg / total_both, total_seg, 1000.0 * total_seg / g_c3_seg_frames);
            printf("  Update: %6.2f%%  (%.6f s total, %.4f ms/frame)\n", 100.0 * total_upd / total_both, total_upd, 1000.0 * total_upd / g_c3_upd_frames);
          }
        }

        //if (g_c3_seg_frames == 3) { printf("%ld\n", (uintptr_t)historyBuffer % 16); }  print is 0, which means historyBuffer is 16-byte aligned, which is good for SIMD optimization in the future.

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
        assert(model->historyBuffer != NULL);
        assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

        /* Some variables. */
        uint32_t width = model->width;
        uint32_t height = model->height;

        uint8_t *historyImage = model->historyImage;
        uint8_t *historyBuffer = model->historyBuffer;

        /* Some utility variable. */
        int numberOfTests = (model->numberOfSamples - NUMBER_OF_HISTORY_IMAGES);

        /* Updating. */
        uint32_t *jump = model->jump;
        int *neighbor = model->neighbor;
        uint32_t *position = model->position;

        clock_t t0, t1;

        /* All the frame, except the border. */
        uint32_t shift, indX, indY;
        int x, y;

      /* ================================ Updating the interior of the image. ====================================*/
        t0 = clock();
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

              int index_neighbor = 3 * (index + neighbor[shift]);

              if (position[shift] < NUMBER_OF_HISTORY_IMAGES) {
                historyImage[3 * index + position[shift] * (3 * width) * height] = r;
                historyImage[3 * index + position[shift] * (3 * width) * height + 1] = g;
                historyImage[3 * index + position[shift] * (3 * width) * height + 2] = b;

                historyImage[index_neighbor + position[shift] * (3 * width) * height] = r;
                historyImage[index_neighbor + position[shift] * (3 * width) * height + 1] = g;
                historyImage[index_neighbor + position[shift] * (3 * width) * height + 2] = b;
              }
              else {
                int pos = position[shift] - NUMBER_OF_HISTORY_IMAGES;

                historyBuffer[(3 * index) * numberOfTests + 3 * pos] = r;
                historyBuffer[(3 * index) * numberOfTests + 3 * pos + 1] = g;
                historyBuffer[(3 * index) * numberOfTests + 3 * pos + 2] = b;

                historyBuffer[index_neighbor * numberOfTests + 3 * pos] = r;
                historyBuffer[index_neighbor * numberOfTests + 3 * pos + 1] = g;
                historyBuffer[index_neighbor * numberOfTests + 3 * pos + 2] = b;
              }
            }

            ++shift;
            indX += jump[shift];
          }
        }
        t1 = clock();
        g_c3_upd_interior += elapsed_sec(t0, t1); // interior: 98.44%

        /* ================================ Updating the borders of the image. ====================================*/

        /* First row. */
        t0 = clock();
        y = 0;
        shift = rand() % width;
        indX = jump[shift]; // index_jump should never be zero (> 1).

        while (indX <= width - 1) {
          int index = indX + y * width;

          uint8_t r = image_data[3 * index];
          uint8_t g = image_data[3 * index + 1];
          uint8_t b = image_data[3 * index + 2];

          if (updating_mask[index] == COLOR_BACKGROUND) {
            if (position[shift] < NUMBER_OF_HISTORY_IMAGES) {
              historyImage[3 * index + position[shift] * (3 * width) * height] = r;
              historyImage[3 * index + position[shift] * (3 * width) * height + 1] = g;
              historyImage[3 * index + position[shift] * (3 * width) * height + 2] = b;
            }
            else {
              int pos = position[shift] - NUMBER_OF_HISTORY_IMAGES;

              historyBuffer[(3 * index) * numberOfTests + 3 * pos] = r;
              historyBuffer[(3 * index) * numberOfTests + 3 * pos + 1] = g;
              historyBuffer[(3 * index) * numberOfTests + 3 * pos + 2] = b;
            }
          }

          ++shift;
          indX += jump[shift];
        }
        t1 = clock();
        g_c3_upd_first_row += elapsed_sec(t0, t1);

        /* Last row. */
        t0 = clock();
        y = height - 1;
        shift = rand() % width;
        indX = jump[shift]; // index_jump should never be zero (> 1).

        while (indX <= width - 1) {
          int index = indX + y * width;

          uint8_t r = image_data[3 * index];
          uint8_t g = image_data[3 * index + 1];
          uint8_t b = image_data[3 * index + 2];

          if (updating_mask[index] == COLOR_BACKGROUND) {
            if (position[shift] < NUMBER_OF_HISTORY_IMAGES) {
              historyImage[3 * index + position[shift] * (3 * width) * height] = r;
              historyImage[3 * index + position[shift] * (3 * width) * height + 1] = g;
              historyImage[3 * index + position[shift] * (3 * width) * height + 2] = b;
            }
            else {
              int pos = position[shift] - NUMBER_OF_HISTORY_IMAGES;

              historyBuffer[(3 * index) * numberOfTests + 3 * pos] = r;
              historyBuffer[(3 * index) * numberOfTests + 3 * pos + 1] = g;
              historyBuffer[(3 * index) * numberOfTests + 3 * pos + 2] = b;
            }
          }

          ++shift;
          indX += jump[shift];
        }
        t1 = clock();
        g_c3_upd_last_row += elapsed_sec(t0, t1);

        /* First column. */
        t0 = clock();
        x = 0;
        shift = rand() % height;
        indY = jump[shift]; // index_jump should never be zero (> 1).

        while (indY <= height - 1) {
          int index = x + indY * width;

          uint8_t r = image_data[3 * index];
          uint8_t g = image_data[3 * index + 1];
          uint8_t b = image_data[3 * index + 2];

          if (updating_mask[index] == COLOR_BACKGROUND) {
            if (position[shift] < NUMBER_OF_HISTORY_IMAGES) {
              historyImage[3 * index + position[shift] * (3 * width) * height] = r;
              historyImage[3 * index + position[shift] * (3 * width) * height + 1] = g;
              historyImage[3 * index + position[shift] * (3 * width) * height + 2] = b;
            }
            else {
              int pos = position[shift] - NUMBER_OF_HISTORY_IMAGES;
              historyBuffer[(3 * index) * numberOfTests + 3 * pos] = r;
              historyBuffer[(3 * index) * numberOfTests + 3 * pos + 1] = g;
              historyBuffer[(3 * index) * numberOfTests + 3 * pos + 2] = b;
            }
          }

          ++shift;
          indY += jump[shift];
        }
        t1 = clock();
        g_c3_upd_first_col += elapsed_sec(t0, t1);

        /* Last column. */
        t0 = clock();
        x = width - 1;
        shift = rand() % height;
        indY = jump[shift]; // index_jump should never be zero (> 1).

        while (indY <= height - 1) {
          int index = x + indY * width;

          uint8_t r = image_data[3 * index];
          uint8_t g = image_data[3 * index + 1];
          uint8_t b = image_data[3 * index + 2];

          if (updating_mask[index] == COLOR_BACKGROUND) {
            if (position[shift] < NUMBER_OF_HISTORY_IMAGES) {
              historyImage[3 * index + position[shift] * (3 * width) * height] = r;
              historyImage[3 * index + position[shift] * (3 * width) * height + 1] = g;
              historyImage[3 * index + position[shift] * (3 * width) * height + 2] = b;
            }
            else {
              int pos = position[shift] - NUMBER_OF_HISTORY_IMAGES;

              historyBuffer[(3 * index) * numberOfTests + 3 * pos] = r;
              historyBuffer[(3 * index) * numberOfTests + 3 * pos + 1] = g;
              historyBuffer[(3 * index) * numberOfTests + 3 * pos + 2] = b;
            }
          }

          ++shift;
          indY += jump[shift];
        }
        t1 = clock();
        g_c3_upd_last_col += elapsed_sec(t0, t1);

        /* The first pixel! */
        t0 = clock();
        if (rand() % model->updateFactor == 0) {
          if (updating_mask[0] == 0) {
            int position = rand() % model->numberOfSamples;

            uint8_t r = image_data[0];
            uint8_t g = image_data[1];
            uint8_t b = image_data[2];

            if (position < NUMBER_OF_HISTORY_IMAGES) {
              historyImage[position * (3 * width) * height] = r;
              historyImage[position * (3 * width) * height + 1] = g;
              historyImage[position * (3 * width) * height + 2] = b;
            }
            else {
              int pos = position - NUMBER_OF_HISTORY_IMAGES;

              historyBuffer[3 * pos] = r;
              historyBuffer[3 * pos + 1] = g;
              historyBuffer[3 * pos + 2] = b;
            }
          }
        }
        t1 = clock();
        g_c3_upd_first_pixel += elapsed_sec(t0, t1);

        ++g_c3_upd_frames;
        if (g_c3_upd_frames % PROFILE_PRINT_INTERVAL == 0) {
          double total = g_c3_upd_interior + g_c3_upd_first_row + g_c3_upd_last_row + g_c3_upd_first_col + g_c3_upd_last_col + g_c3_upd_first_pixel;
          if (total > 0) {
            printf("[ViBe C3R Update] over %lu frames:\n", g_c3_upd_frames);
            printf("  interior:     %6.2f%%\n", 100.0 * g_c3_upd_interior / total);
            printf("  first_row:    %6.2f%%\n", 100.0 * g_c3_upd_first_row / total);
            printf("  last_row:     %6.2f%%\n", 100.0 * g_c3_upd_last_row / total);
            printf("  first_col:    %6.2f%%\n", 100.0 * g_c3_upd_first_col / total);
            printf("  last_col:     %6.2f%%\n", 100.0 * g_c3_upd_last_col / total);
            printf("  first_pixel:  %6.2f%%\n", 100.0 * g_c3_upd_first_pixel / total);
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
