#include <assert.h>
#include <iostream>
#include <stdio.h>
#include <time.h>

#if defined(__linux__)
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

#include "vibe-fpga-pdma.h"
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

      static inline bool fpga_transport_use_mmio()
      {
        const char *mode = getenv("VIBE_FPGA_TRANSPORT");
        return mode != NULL && strcmp(mode, "mmio") == 0;
      }

      static inline bool fpga_transport_use_pdma()
      {
        const char *mode = getenv("VIBE_FPGA_TRANSPORT");
        return mode != NULL && strcmp(mode, "pdma") == 0;
      }

      static inline uint32_t env_u32_hex_or_dec(const char *name, const uint32_t defaultValue)
      {
        const char *value = getenv(name);
        if (value == NULL || *value == '\0')
          return defaultValue;

        char *end = NULL;
        unsigned long parsed = strtoul(value, &end, 0);
        if (end == value)
          return defaultValue;

        return static_cast<uint32_t>(parsed);
      }

#if defined(__linux__)
      static inline void fpga_reg_write(volatile uint32_t *regs, const uint32_t offset, const uint32_t value)
      {
        regs[offset >> 2] = value;
        __sync_synchronize();
      }

      static inline uint32_t fpga_reg_read(volatile uint32_t *regs, const uint32_t offset)
      {
        __sync_synchronize();
        return regs[offset >> 2];
      }
#endif

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
        size_t fpgaSharedFrameOffset;
        size_t fpgaSharedHistoryOffset;
        size_t fpgaSharedOutOffset;
        size_t fpgaSharedFrameBytes;
        size_t fpgaSharedHistoryBytes;
        size_t fpgaSharedOutBytes;
        size_t fpgaSharedControlOffset;
        bool fpgaUseMmio;
        uint32_t fpgaSharedPhysBase;
        uint32_t fpgaApbPhysBase;
        int fpgaDevMemFd;
        void *fpgaSharedMapping;
        size_t fpgaSharedMappingBytes;
        void *fpgaApbMapping;
        size_t fpgaApbMappingBytes;
        volatile uint32_t *fpgaApbRegs;
        uint64_t fpgaFrameSequence;

        /* PDMA pixel_proc FPGA fields (VIBE_FPGA_TRANSPORT=pdma). */
        bool fpgaUsePdma;
        uint8_t *fpgaDdrBuffer;
        size_t fpgaDdrBufferBytes;
        uint32_t fpgaDdrPhysBase;
        uint32_t fpgaPixelProcPhysBase;
        void *fpgaDdrMapping;
        size_t fpgaDdrMappingBytes;
        void *fpgaPixelProcMapping;
        size_t fpgaPixelProcMappingBytes;
        volatile uint32_t *fpgaPixelProcRegs;

        /* Buffers with random values. */
        uint32_t *jump;
        int *neighbor;
        uint32_t *position;
      };

#if defined(__linux__)
      static bool map_fpga_transport(vibeModel_Sequential_t *model)
      {
        if (model == NULL)
          return false;

        if (!model->fpgaUseMmio)
          return false;

        if (model->fpgaApbRegs != NULL && model->fpgaSharedMemory != NULL)
          return true;

        const long pageSize = sysconf(_SC_PAGESIZE);
        if (pageSize <= 0)
          return false;

        const uint32_t sharedPhysBase = env_u32_hex_or_dec("VIBE_FPGA_SHARED_BASE", fpga::kSharedMemoryPhysBase);
        const uint32_t apbPhysBase = env_u32_hex_or_dec("VIBE_FPGA_APB_BASE", 0u);
        if (apbPhysBase == 0u)
          return false;

        model->fpgaDevMemFd = open("/dev/mem", O_RDWR | O_SYNC);
        if (model->fpgaDevMemFd < 0)
          return false;

        const off_t sharedMapBase = static_cast<off_t>(sharedPhysBase & ~(static_cast<uint32_t>(pageSize) - 1u));
        const size_t sharedMapDelta = static_cast<size_t>(sharedPhysBase - static_cast<uint32_t>(sharedMapBase));
        const size_t sharedMapBytes = fpga::AlignUp(model->fpgaSharedMemoryBytes + sharedMapDelta, static_cast<size_t>(pageSize));
        void *sharedMap = mmap(NULL, sharedMapBytes, PROT_READ | PROT_WRITE, MAP_SHARED, model->fpgaDevMemFd, sharedMapBase);
        if (sharedMap == MAP_FAILED) {
          close(model->fpgaDevMemFd);
          model->fpgaDevMemFd = -1;
          return false;
        }

        const off_t apbMapBase = static_cast<off_t>(apbPhysBase & ~(static_cast<uint32_t>(pageSize) - 1u));
        const size_t apbMapDelta = static_cast<size_t>(apbPhysBase - static_cast<uint32_t>(apbMapBase));
        const size_t apbMapBytes = fpga::AlignUp(fpga::kApbRegisterSpanBytes + apbMapDelta, static_cast<size_t>(pageSize));
        void *apbMap = mmap(NULL, apbMapBytes, PROT_READ | PROT_WRITE, MAP_SHARED, model->fpgaDevMemFd, apbMapBase);
        if (apbMap == MAP_FAILED) {
          munmap(sharedMap, sharedMapBytes);
          close(model->fpgaDevMemFd);
          model->fpgaDevMemFd = -1;
          return false;
        }

        model->fpgaSharedPhysBase = sharedPhysBase;
        model->fpgaApbPhysBase = apbPhysBase;
        model->fpgaSharedMapping = sharedMap;
        model->fpgaSharedMappingBytes = sharedMapBytes;
        model->fpgaApbMapping = apbMap;
        model->fpgaApbMappingBytes = apbMapBytes;
        model->fpgaSharedMemory = reinterpret_cast<uint8_t*>(sharedMap) + sharedMapDelta;
        model->fpgaApbRegs = reinterpret_cast<volatile uint32_t*>(reinterpret_cast<uint8_t*>(apbMap) + apbMapDelta);
        return true;
      }

      static void unmap_fpga_transport(vibeModel_Sequential_t *model)
      {
        if (model == NULL)
          return;

        if (model->fpgaApbMapping != NULL) {
          munmap(model->fpgaApbMapping, model->fpgaApbMappingBytes);
          model->fpgaApbMapping = NULL;
          model->fpgaApbMappingBytes = 0u;
          model->fpgaApbRegs = NULL;
        }

        if (model->fpgaSharedMapping != NULL) {
          munmap(model->fpgaSharedMapping, model->fpgaSharedMappingBytes);
          model->fpgaSharedMapping = NULL;
          model->fpgaSharedMappingBytes = 0u;
          model->fpgaSharedMemory = NULL;
        }

        if (model->fpgaDevMemFd >= 0) {
          close(model->fpgaDevMemFd);
          model->fpgaDevMemFd = -1;
        }
      }

      // -----------------------------------------------------------------------
      // PDMA pixel_proc mapping helpers (use the same /dev/mem fd)
      // -----------------------------------------------------------------------
      static bool map_pdma_ddr_buffer(vibeModel_Sequential_t *model,
                                      const uint32_t ddr_phys_base,
                                      const size_t ddr_bytes)
      {
        if (model == NULL || ddr_bytes == 0u)
          return false;

        const long pageSize = sysconf(_SC_PAGESIZE);
        if (pageSize <= 0)
          return false;

        if (model->fpgaDevMemFd < 0) {
          model->fpgaDevMemFd = open("/dev/mem", O_RDWR | O_SYNC);
          if (model->fpgaDevMemFd < 0)
            return false;
        }

        const off_t mapBase = static_cast<off_t>(ddr_phys_base & ~(static_cast<uint32_t>(pageSize) - 1u));
        const size_t mapDelta = static_cast<size_t>(ddr_phys_base - static_cast<uint32_t>(mapBase));
        const size_t mapBytes = fpga::AlignUp(ddr_bytes + mapDelta, static_cast<size_t>(pageSize));

        void *map = mmap(NULL, mapBytes, PROT_READ | PROT_WRITE, MAP_SHARED,
                         model->fpgaDevMemFd, mapBase);
        if (map == MAP_FAILED)
          return false;

        model->fpgaDdrPhysBase = ddr_phys_base;
        model->fpgaDdrMapping = map;
        model->fpgaDdrMappingBytes = mapBytes;
        model->fpgaDdrBuffer = reinterpret_cast<uint8_t*>(map) + mapDelta;
        model->fpgaDdrBufferBytes = ddr_bytes;
        return true;
      }

      static bool map_pdma_pixel_proc_regs(vibeModel_Sequential_t *model,
                                           const uint32_t pixel_proc_phys)
      {
        if (model == NULL)
          return false;

        const long pageSize = sysconf(_SC_PAGESIZE);
        if (pageSize <= 0)
          return false;

        if (model->fpgaDevMemFd < 0) {
          model->fpgaDevMemFd = open("/dev/mem", O_RDWR | O_SYNC);
          if (model->fpgaDevMemFd < 0)
            return false;
        }

        const off_t mapBase = static_cast<off_t>(pixel_proc_phys & ~(static_cast<uint32_t>(pageSize) - 1u));
        const size_t mapDelta = static_cast<size_t>(pixel_proc_phys - static_cast<uint32_t>(mapBase));
        // Register block is 0x20 bytes per the device-tree snippet.
        const size_t regBytes = 0x20u;
        const size_t mapBytes = fpga::AlignUp(regBytes + mapDelta, static_cast<size_t>(pageSize));

        void *map = mmap(NULL, mapBytes, PROT_READ | PROT_WRITE, MAP_SHARED,
                         model->fpgaDevMemFd, mapBase);
        if (map == MAP_FAILED)
          return false;

        model->fpgaPixelProcPhysBase = pixel_proc_phys;
        model->fpgaPixelProcMapping = map;
        model->fpgaPixelProcMappingBytes = mapBytes;
        model->fpgaPixelProcRegs = reinterpret_cast<volatile uint32_t*>(
            reinterpret_cast<uint8_t*>(map) + mapDelta);
        return true;
      }

      static void unmap_pdma_transport(vibeModel_Sequential_t *model)
      {
        if (model == NULL)
          return;

        if (model->fpgaPixelProcMapping != NULL) {
          munmap(model->fpgaPixelProcMapping, model->fpgaPixelProcMappingBytes);
          model->fpgaPixelProcMapping = NULL;
          model->fpgaPixelProcMappingBytes = 0u;
          model->fpgaPixelProcRegs = NULL;
        }

        if (model->fpgaDdrMapping != NULL) {
          munmap(model->fpgaDdrMapping, model->fpgaDdrMappingBytes);
          model->fpgaDdrMapping = NULL;
          model->fpgaDdrMappingBytes = 0u;
          model->fpgaDdrBuffer = NULL;
          model->fpgaDdrBufferBytes = 0u;
        }

        if (model->fpgaDevMemFd >= 0) {
          close(model->fpgaDevMemFd);
          model->fpgaDevMemFd = -1;
        }
      }
#endif

      static bool launch_fpga_compare(vibeModel_Sequential_t *model)
      {
        if (model == NULL || model->fpgaSharedMemory == NULL)
          return false;

        const fpga::SharedLayout layout = fpga::MakeSharedLayout(model->width, model->height);
        fpga::ControlBlock *control = reinterpret_cast<fpga::ControlBlock*>(model->fpgaSharedMemory + layout.control_offset);

        if (!model->fpgaUseMmio) {
          vibeFpgaSharedMemoryView_Sequential_t fpgaView{};
          libvibeModel_Sequential_GetFpgaSharedMemoryView(model, &fpgaView);
          return libvibeModel_Sequential_SimulateFpgaSharedMemory_8u_C3R(&fpgaView) == 0;
        }

#if defined(__linux__)
        if (!map_fpga_transport(model))
          return false;

        control->start = 1u;
        control->done = 0u;
        control->status = fpga::kStatusIdle;
        control->error_code = 0u;
        __sync_synchronize();

        fpga_reg_write(model->fpgaApbRegs, fpga::kRegControlBlockAddr, model->fpgaSharedPhysBase + static_cast<uint32_t>(layout.control_offset));
        fpga_reg_write(model->fpgaApbRegs, fpga::kRegControl, 0x2u);
        fpga_reg_write(model->fpgaApbRegs, fpga::kRegControl, 0x1u);

        const uint32_t maxPolls = env_u32_hex_or_dec("VIBE_FPGA_POLL_LIMIT", 200000u);
        for (uint32_t poll = 0u; poll < maxPolls; ++poll) {
          const uint32_t status = fpga_reg_read(model->fpgaApbRegs, fpga::kRegControl);
          if ((status & fpga::kStatusError) != 0u)
            return false;
          if ((status & fpga::kStatusDone) != 0u) {
            __sync_synchronize();
            return control->done != 0u && control->status == fpga::kStatusDone && control->error_code == 0u;
          }

          if ((poll % 128u) == 0u)
            usleep(10);
        }
#endif

        return false;
      }

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
        model->fpgaSharedFrameOffset = 0u;
        model->fpgaSharedHistoryOffset = 0u;
        model->fpgaSharedOutOffset = 0u;
        model->fpgaSharedFrameBytes = 0u;
        model->fpgaSharedHistoryBytes = 0u;
        model->fpgaSharedOutBytes = 0u;
        model->fpgaSharedControlOffset = 0u;
        model->fpgaUseMmio = fpga_transport_use_mmio();
        model->fpgaSharedPhysBase = fpga::kSharedMemoryPhysBase;
        model->fpgaApbPhysBase = 0u;
        model->fpgaDevMemFd = -1;
        model->fpgaSharedMapping = NULL;
        model->fpgaSharedMappingBytes = 0u;
        model->fpgaApbMapping = NULL;
        model->fpgaApbMappingBytes = 0u;
        model->fpgaApbRegs = NULL;
        model->fpgaFrameSequence = 0u;

        /* PDMA pixel_proc FPGA fields. */
        model->fpgaUsePdma = fpga_transport_use_pdma();
        model->fpgaDdrBuffer = NULL;
        model->fpgaDdrBufferBytes = 0u;
        model->fpgaDdrPhysBase = 0u;
        model->fpgaPixelProcPhysBase = fpga::kPixelProcPhysBase + fpga::kPixelProcRegOffset;
        model->fpgaDdrMapping = NULL;
        model->fpgaDdrMappingBytes = 0u;
        model->fpgaPixelProcMapping = NULL;
        model->fpgaPixelProcMappingBytes = 0u;
        model->fpgaPixelProcRegs = NULL;

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
        view->frameOffset = model->fpgaSharedFrameOffset;
        view->historyOffset = model->fpgaSharedHistoryOffset;
        view->outOffset = model->fpgaSharedOutOffset;
        view->controlOffset = model->fpgaSharedControlOffset;
        view->frameBytes = model->fpgaSharedFrameBytes;
        view->historyBytes = model->fpgaSharedHistoryBytes;
        view->outBytes = model->fpgaSharedOutBytes;
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

        if (model->fpgaUsePdma) {
          // Hardware threshold is fixed at 45 (SAD ≤ 45); the software
          // parameter is advisory and does not affect FPGA behaviour.
          fprintf(stderr,
            "[ViBe PDMA] WARNING: matchingThreshold is hard-coded to %u by the FPGA. "
            "Ignoring request to set %u.\n",
            static_cast<unsigned>(fpga::kHardwareSadThreshold),
            static_cast<unsigned>(matchingThreshold));
          return(0);
        }

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

        if (model->fpgaUsePdma) {
          // Hardware uses fixed match_count > 2 → matching_number = 3.
          fprintf(stderr,
            "[ViBe PDMA] WARNING: matchingNumber is hard-coded to %u by the FPGA. "
            "Ignoring request to set %u.\n",
            static_cast<unsigned>(fpga::kHardwareMatchingNumber),
            static_cast<unsigned>(matchingNumber));
          return(0);
        }

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
        if (model->fpgaUsePdma) {
#if defined(__linux__)
          if (model->fpgaDdrMapping != NULL || model->fpgaPixelProcMapping != NULL)
            unmap_pdma_transport(model);
          else
            free(model->fpgaDdrBuffer);
#else
          free(model->fpgaDdrBuffer);
#endif
        } else if (model->fpgaUseMmio) {
#if defined(__linux__)
          unmap_fpga_transport(model);
#endif
        } else {
          free(model->fpgaSharedMemory);
        }
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

        const uint32_t pixelCount = model->width * model->height;

        if (model->fpgaUsePdma) {
          // -----------------------------------------------------------------
          // PDMA pixel_proc path: pixel-interleaved DDR buffer (96 B/pixel).
          // -----------------------------------------------------------------

          // Hardware overrides — the FPGA RTL hard-codes these parameters.
          model->numberOfSamples = fpga::kHistoryFramesPdma;       // 23
          model->matchingThreshold = fpga::kHardwareSadThreshold;  // 45
          model->matchingNumber = fpga::kHardwareMatchingNumber;   // 3

          // Allocate DDR buffer (heap first; remap via /dev/mem if a
          // physical base was supplied through the environment).
          model->fpgaDdrBufferBytes = fpga::DdrPixelLayout::totalBytes(pixelCount);
          model->fpgaDdrBuffer = (uint8_t*)malloc(model->fpgaDdrBufferBytes);
          assert(model->fpgaDdrBuffer != NULL);
          memset(model->fpgaDdrBuffer, 0, model->fpgaDdrBufferBytes);

#if defined(__linux__)
          {
            const uint32_t ddrPhys = env_u32_hex_or_dec("VIBE_FPGA_DDR_BASE", 0u);
            if (ddrPhys != 0u) {
              uint8_t *heapBuf = model->fpgaDdrBuffer;
              const size_t heapBytes = model->fpgaDdrBufferBytes;
              if (map_pdma_ddr_buffer(model, ddrPhys, heapBytes)) {
                memcpy(model->fpgaDdrBuffer, heapBuf, heapBytes);
                free(heapBuf);
              }
            }
          }
#endif

          // Map pixel_proc registers (may fail; we retry in Segmentation).
#if defined(__linux__)
          map_pdma_pixel_proc_regs(model, model->fpgaPixelProcPhysBase);
#endif

          // Lambdas for the pixel-interleaved DDR layout.
          auto ddrCurrentSlot = [&](const uint32_t pixelIndex) -> uint8_t* {
            return model->fpgaDdrBuffer
              + fpga::DdrPixelLayout::currentOffset(pixelIndex);
          };
          auto ddrHistorySlot = [&](const uint32_t pixelIndex, const uint32_t sampleIndex) -> uint8_t* {
            return model->fpgaDdrBuffer
              + fpga::DdrPixelLayout::historyOffset(pixelIndex, sampleIndex);
          };
          // Hardware expects RGBX byte order (byte 0 = R, byte 1 = G, byte 2 = B).
          auto writeRgbx = [](uint8_t *dst, const uint8_t *src) {
            dst[0] = src[2];  // R = OpenCV BGR byte 2
            dst[1] = src[1];  // G
            dst[2] = src[0];  // B = OpenCV BGR byte 0
            dst[3] = 0u;
          };
          auto writeRgbxValues = [](uint8_t *dst, const uint8_t r, const uint8_t g, const uint8_t b) {
            dst[0] = r;
            dst[1] = g;
            dst[2] = b;
            dst[3] = 0u;
          };
          auto plusNoise = [](uint8_t value) -> uint8_t {
            int value_plus_noise = value + rand() % 20 - 10;
            if (value_plus_noise < 0) { value_plus_noise = 0; }
            if (value_plus_noise > 255) { value_plus_noise = 255; }
            return static_cast<uint8_t>(value_plus_noise);
          };

          const uint32_t seededSamples = seeded_sample_count(model->numberOfSamples);
          for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
            const uint8_t *pixel = image_data + 3u * pixelIndex;

            // Entry 0 = current frame.
            writeRgbx(ddrCurrentSlot(pixelIndex), pixel);

            // Entries 1..23 = history samples.
            for (uint32_t sampleIndex = 0; sampleIndex < model->numberOfSamples; ++sampleIndex) {
              uint8_t *historyPtr = ddrHistorySlot(pixelIndex, sampleIndex);
              if (sampleIndex < seededSamples) {
                writeRgbx(historyPtr, pixel);
              } else {
                // OpenCV BGR: src[0]=B, src[1]=G, src[2]=R.
                writeRgbxValues(
                  historyPtr,
                  plusNoise(pixel[2]),  // R
                  plusNoise(pixel[1]),  // G
                  plusNoise(pixel[0])); // B
              }
            }
          }
        }
        else
        {
          // -----------------------------------------------------------------
          // Original shared-memory / simulation path.
          // -----------------------------------------------------------------
          const fpga::SharedLayout layout = fpga::MakeSharedLayout(width, height);
          model->fpgaSharedMemoryBytes = layout.total_bytes;
          model->fpgaSharedFrameOffset = layout.frame_offset;
          model->fpgaSharedHistoryOffset = layout.history_offset;
          model->fpgaSharedOutOffset = layout.out_offset;
          model->fpgaSharedFrameBytes = layout.frame_bytes;
          model->fpgaSharedHistoryBytes = layout.history_bytes;
          model->fpgaSharedOutBytes = layout.out_bytes;
          model->fpgaSharedControlOffset = layout.control_offset;

          if (model->fpgaUseMmio) {
#if defined(__linux__)
            if (!map_fpga_transport(model))
              model->fpgaUseMmio = false;
#else
            model->fpgaUseMmio = false;
#endif
          }

          if (!model->fpgaUseMmio) {
            model->fpgaSharedMemory = (uint8_t*)malloc(model->fpgaSharedMemoryBytes);
            assert(model->fpgaSharedMemory != NULL);
          }

          auto frameSlot = [&](const uint32_t pixelIndex) -> uint8_t* {
            return model->fpgaSharedMemory
              + model->fpgaSharedFrameOffset
              + static_cast<size_t>(pixelIndex) * fpga::kInputPixelBytes;
          };
          auto historySlot = [&](const uint32_t pixelIndex, const uint32_t sampleIndex) -> uint8_t* {
            return model->fpgaSharedMemory
              + model->fpgaSharedHistoryOffset
              + (static_cast<size_t>(sampleIndex) * static_cast<size_t>(width) * static_cast<size_t>(height)
                + static_cast<size_t>(pixelIndex)) * fpga::kInputPixelBytes;
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

          const uint32_t seededSamples = seeded_sample_count(model->numberOfSamples);
          for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
            const uint8_t *pixel = image_data + 3u * pixelIndex;

            writeBgrx(frameSlot(pixelIndex), pixel);

            for (uint32_t sampleIndex = 0; sampleIndex < model->numberOfSamples; ++sampleIndex) {
              uint8_t *historyPtr = historySlot(pixelIndex, sampleIndex);
              if (sampleIndex < seededSamples) {
                writeBgrx(historyPtr, pixel);
              }
              else {
                writeBgrxValues(
                  historyPtr,
                  plusNoise(pixel[0]),
                  plusNoise(pixel[1]),
                  plusNoise(pixel[2])
                );
              }
            }
          }

          fpga::ControlBlock *control = reinterpret_cast<fpga::ControlBlock*>(model->fpgaSharedMemory + model->fpgaSharedControlOffset);
          *control = fpga::MakeControlBlock(
            width,
            height,
            model->matchingThreshold,
            model->matchingNumber,
            static_cast<uint32_t>(model->fpgaFrameSequence),
            model->lastHistorySampleSwapped
          );
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
        assert(model->fpgaUsePdma ? (model->fpgaDdrBuffer != NULL) : (model->fpgaSharedMemory != NULL));
        assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

        /* Some variables. */
        static uint64_t g_c3_seg_debug_frame = 0u;
        uint32_t width = model->width;
        uint32_t height = model->height;
        const uint32_t pixelCount = width * height;

        // Clear the software-visible output map.
        memset(segmentation_map, COLOR_BACKGROUND, pixelCount);

        if (model->fpgaUsePdma) {
          // -----------------------------------------------------------------
          // PDMA pixel_proc path.
          // -----------------------------------------------------------------

          // Stage 1: ensure pixel_proc registers are mapped.
#if defined(__linux__)
          if (model->fpgaPixelProcRegs == NULL)
            map_pdma_pixel_proc_regs(model, model->fpgaPixelProcPhysBase);
#endif

          // Stage 2: refresh the current-frame entry (entry 0) for every
          // pixel in the DDR buffer.  Hardware expects RGBX byte order.
          for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
            const uint8_t *src = image_data + 3u * pixelIndex;
            uint8_t *dst = model->fpgaDdrBuffer
              + fpga::DdrPixelLayout::currentOffset(pixelIndex);
            dst[0] = src[2];  // R = OpenCV BGR byte 2
            dst[1] = src[1];  // G
            dst[2] = src[0];  // B = OpenCV BGR byte 0
            dst[3] = 0u;
          }

          // Stage 3: launch the FPGA via PDMA (or CPU fallback).
          // This writes segmentation_map directly from the RESULT register.
          const uint32_t ddrPhys = model->fpgaDdrPhysBase != 0u
              ? model->fpgaDdrPhysBase
              : 0u;
          const bool launched = (model->fpgaPixelProcRegs != NULL)
              && pdma_launch_pixel_proc(
                   model->fpgaDdrBuffer,
                   ddrPhys,
                   pixelCount,
                   model->fpgaPixelProcRegs,
                   segmentation_map,
                   width);
          if (!launched)
            return(-1);
        }
        else
        {
          // -----------------------------------------------------------------
          // Original shared-memory / simulation path.
          // -----------------------------------------------------------------
          const fpga::SharedLayout layout = fpga::MakeSharedLayout(width, height);
          auto frameSlot = [&](const uint32_t pixelIndex) -> uint8_t* {
            return model->fpgaSharedMemory
              + layout.frame_offset
              + static_cast<size_t>(pixelIndex) * fpga::kInputPixelBytes;
          };
          auto outputWord = [&](const uint32_t pixelIndex) -> uint32_t* {
            return reinterpret_cast<uint32_t*>(
              model->fpgaSharedMemory
              + layout.out_offset
              + static_cast<size_t>(pixelIndex) * fpga::kOutputWordBytes
            );
          };
          auto writeBgrx = [](uint8_t *dst, const uint8_t *src) {
            dst[0] = src[0];
            dst[1] = src[1];
            dst[2] = src[2];
            dst[3] = 0u;
          };

          // Stage 1: clear the shared output area.
          memset(model->fpgaSharedMemory + layout.out_offset, 0, layout.out_bytes);

          // Stage 2: refresh the current frame region.
          for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex)
            writeBgrx(frameSlot(pixelIndex), image_data + 3u * pixelIndex);

          // Stage 3: prepare the control block and launch the compare pass.
          {
            fpga::ControlBlock *control = reinterpret_cast<fpga::ControlBlock*>(model->fpgaSharedMemory + layout.control_offset);
            *control = fpga::MakeControlBlock(
              width,
              height,
              model->matchingThreshold,
              model->matchingNumber,
              static_cast<uint32_t>(model->fpgaFrameSequence),
              model->lastHistorySampleSwapped
            );
          }

          const bool launched = launch_fpga_compare(model);
          if (!launched)
            return(-1);

          // Stage 4: convert compare words to the foreground mask.
          for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
            const uint32_t comparisonWord = *outputWord(pixelIndex);
            segmentation_map[pixelIndex] = fpga::IsForeground(comparisonWord, model->matchingNumber)
              ? COLOR_FOREGROUND
              : COLOR_BACKGROUND;
          }
        }

        ++model->fpgaFrameSequence;

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
        assert(model->fpgaUsePdma ? (model->fpgaDdrBuffer != NULL) : (model->fpgaSharedMemory != NULL));
        assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

        /* Some variables. */
        uint32_t width = model->width;
        uint32_t height = model->height;

        // Update only the history slots. Slot 0 (entry 0 in PDMA DDR) is always
        // reserved for the current frame of the next segmentation pass.
        auto writeHistorySample = [&](const uint32_t pixelIndex, const uint32_t sampleIndex,
                                      const uint8_t r, const uint8_t g, const uint8_t b) {
          uint8_t *slot;
          if (model->fpgaUsePdma) {
            // DDR pixel-interleaved layout: history samples start at entry 1.
            slot = model->fpgaDdrBuffer
              + fpga::DdrPixelLayout::historyOffset(pixelIndex, sampleIndex);
            // Hardware expects RGBX byte order.
            slot[0] = r;
            slot[1] = g;
            slot[2] = b;
            slot[3] = 0u;
          } else {
            // Original plane-major shared memory layout (BGRX).
            const fpga::SharedLayout layout = fpga::MakeSharedLayout(width, height);
            slot = model->fpgaSharedMemory
              + layout.history_offset
              + (static_cast<size_t>(sampleIndex) * static_cast<size_t>(width) * static_cast<size_t>(height)
                + static_cast<size_t>(pixelIndex)) * fpga::kInputPixelBytes;
            slot[0] = r;
            slot[1] = g;
            slot[2] = b;
            slot[3] = 0u;
          }
        };

        /* Updating. */
        uint32_t *jump = model->jump;
        int *neighbor = model->neighbor;
        uint32_t *position = model->position;

        /* All the frame, except the border. */
        uint32_t shift, indX, indY;
        int x, y;

        // Update history slots. Slot 0 is always reserved for the current
        // frame of the next segmentation pass.
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
