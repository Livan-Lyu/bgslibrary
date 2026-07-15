#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "vibe-fpga-shared.h"
#include "vibe-background-sequential.h"

namespace bgslibrary
{
  namespace algorithms
  {
    namespace vibe
    {
      // =========================================================================
      // Utilities
      // =========================================================================
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

      // =========================================================================
      // Model struct
      // =========================================================================
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
        uint32_t lastHistorySampleSwapped;

        /* DDR buffer (pixel-interleaved RGBX, 96 bytes/pixel). */
        uint8_t  *fpgaDdrBuffer;
        size_t    fpgaDdrBufferBytes;
        uint32_t  fpgaDdrPhysBase;        // 0 = heap, else physical address via /dev/mem
        void     *fpgaDdrMapping;
        size_t    fpgaDdrMappingBytes;

        /* pixel_proc registers (mmap'd via UIO or /dev/mem). */
        volatile uint32_t *fpgaPixelProcRegs;
        void              *fpgaPixelProcMapping;
        size_t             fpgaPixelProcMappingBytes;
        int                fpgaDevMemFd;
        uint64_t           fpgaFrameSequence;

        /* Buffers with random values. */
        uint32_t *jump;
        int      *neighbor;
        uint32_t *position;
      };

      // =========================================================================
      // Hardware register I/O
      // =========================================================================
      static inline void regWrite(volatile uint32_t *regs, uint32_t offset, uint32_t value)
      {
        regs[offset >> 2] = value;
        __sync_synchronize();
      }

      static inline uint32_t regRead(volatile uint32_t *regs, uint32_t offset)
      {
        __sync_synchronize();
        return regs[offset >> 2];
      }

      static bool shared_memory_contains(uint32_t physBase, size_t bytes)
      {
        if (bytes == 0u)
          return true;
        const uint64_t begin = static_cast<uint64_t>(physBase);
        const uint64_t end = begin + static_cast<uint64_t>(bytes) - 1u;
        return begin >= kSharedMemoryPhysBase && end <= kSharedMemoryPhysEnd && end >= begin;
      }

      static void dump_pixel_proc_regs(volatile uint32_t *regs, const char *tag)
      {
        if (regs == NULL)
          return;

        const uint32_t startIdle = regRead(regs, REG_START_IDLE);
        const uint32_t inputPtr  = regRead(regs, REG_INPUT_PTR);
        const uint32_t outputPtr = regRead(regs, REG_OUTPUT_PTR);
        const uint32_t count     = regRead(regs, REG_PIXEL_COUNT);

        fprintf(stderr,
          "[pixel_proc:%s] START_IDLE=0x%08X INPUT=0x%08X OUTPUT=0x%08X CNT=%u\n",
          tag, startIdle, inputPtr, outputPtr, count);
      }

      // =========================================================================
      // pixel_proc register mapping — UIO first, /dev/mem fallback
      // =========================================================================
      static const char* get_uio_device()
      {
        const char *uio = getenv("VIBE_FPGA_UIO");
        if (uio != NULL && *uio != '\0')
          return uio;
        return "/dev/uio0";
      }

      static bool map_pixel_proc_regs(vibeModel_Sequential_t *model)
      {
        if (model == NULL)
          return false;

        if (model->fpgaPixelProcRegs != NULL)
          return true;  // already mapped

        int fd = -1;
        void *mapping = MAP_FAILED;
        size_t mapBytes = 0x1000u;

        // ---- Try UIO first ----
        const char *uioPath = get_uio_device();
        fd = open(uioPath, O_RDWR);
        if (fd >= 0) {
          mapping = mmap(NULL, mapBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
          if (mapping != MAP_FAILED) {
            model->fpgaPixelProcRegs = reinterpret_cast<volatile uint32_t*>(mapping);
            goto mapped;
          }
          close(fd);
          fd = -1;
        }

        // ---- Fallback: /dev/mem ----
        {
          const long pageSize = sysconf(_SC_PAGESIZE);
          if (pageSize <= 0)
            return false;

          const uint32_t capeBase = kPixelProcPhysBase;
          const off_t mapBase = static_cast<off_t>(capeBase & ~(static_cast<uint32_t>(pageSize) - 1u));
          const size_t mapDelta = static_cast<size_t>(capeBase - static_cast<uint32_t>(mapBase));
          mapBytes = fpga::AlignUp(0x24u + mapDelta, static_cast<size_t>(pageSize));

          fd = open("/dev/mem", O_RDWR | O_SYNC);
          if (fd < 0)
            return false;

          mapping = mmap(NULL, mapBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mapBase);
          if (mapping == MAP_FAILED) {
            close(fd);
            return false;
          }

          model->fpgaPixelProcRegs = reinterpret_cast<volatile uint32_t*>(
            reinterpret_cast<uint8_t*>(mapping) + mapDelta);
        }

      mapped:
        model->fpgaDevMemFd = fd;
        model->fpgaPixelProcMapping = mapping;
        model->fpgaPixelProcMappingBytes = mapBytes;
        return true;
      }

      static bool map_ddr_buffer(vibeModel_Sequential_t *model, uint32_t physBase)
      {
        if (model == NULL || physBase == 0u)
          return false;

        const long pageSize = sysconf(_SC_PAGESIZE);
        if (pageSize <= 0)
          return false;

        const off_t mapBase = static_cast<off_t>(physBase & ~(static_cast<uint32_t>(pageSize) - 1u));
        const size_t mapDelta = static_cast<size_t>(physBase - static_cast<uint32_t>(mapBase));
        const uint64_t maxSharedBytes = static_cast<uint64_t>(kSharedMemoryPhysEnd) -
          static_cast<uint64_t>(physBase) + 1u;
        if (physBase < kSharedMemoryPhysBase || physBase > kSharedMemoryPhysEnd ||
            static_cast<uint64_t>(model->fpgaDdrBufferBytes) > maxSharedBytes)
          return false;

        const size_t mapBytes = fpga::AlignUp(static_cast<size_t>(maxSharedBytes) + mapDelta,
          static_cast<size_t>(pageSize));

        int fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (fd < 0)
          return false;

        void *mapping = mmap(NULL, mapBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mapBase);
        if (mapping == MAP_FAILED) {
          close(fd);
          return false;
        }

        // Copy heap buffer content to physically-mapped region, then free heap.
        uint8_t *physBuffer = reinterpret_cast<uint8_t*>(mapping) + mapDelta;
        memcpy(physBuffer, model->fpgaDdrBuffer, model->fpgaDdrBufferBytes);
        free(model->fpgaDdrBuffer);

        model->fpgaDdrBuffer = physBuffer;
        model->fpgaDdrPhysBase = physBase;
        model->fpgaDdrMapping = mapping;
        model->fpgaDdrMappingBytes = mapBytes;
        return true;
      }

      static void unmap_pixel_proc(vibeModel_Sequential_t *model)
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
        } else {
          free(model->fpgaDdrBuffer);
          model->fpgaDdrBuffer = NULL;
        }
        model->fpgaDdrBufferBytes = 0u;
        model->fpgaDdrPhysBase = 0u;

        if (model->fpgaDevMemFd >= 0) {
          close(model->fpgaDevMemFd);
          model->fpgaDevMemFd = -1;
        }
      }

      // =========================================================================
      // Print parameters
      // =========================================================================
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

      // =========================================================================
      // Creates the data structure
      // =========================================================================
      vibeModel_Sequential_t *libvibeModel_Sequential_New()
      {
        vibeModel_Sequential_t *model = (vibeModel_Sequential_t*)calloc(1, sizeof(*model));
        assert(model != NULL);

        /* Default parameters values. */
        model->numberOfSamples   = fpga::kHistoryFrames;
        model->matchingThreshold = 20;
        model->matchingNumber    = 2;
        model->updateFactor      = 16;

        /* Storage for the history. */
        model->lastHistorySampleSwapped = 0;

        /* pixel_proc / DDR */
        model->fpgaDdrBuffer            = NULL;
        model->fpgaDdrBufferBytes       = 0u;
        model->fpgaDdrPhysBase          = 0u;
        model->fpgaDdrMapping           = NULL;
        model->fpgaDdrMappingBytes      = 0u;
        model->fpgaPixelProcRegs        = NULL;
        model->fpgaPixelProcMapping     = NULL;
        model->fpgaPixelProcMappingBytes= 0u;
        model->fpgaDevMemFd             = -1;
        model->fpgaFrameSequence        = 0u;

        /* Buffers with random values. */
        model->jump     = NULL;
        model->neighbor = NULL;
        model->position = NULL;

        return(model);
      }

      // =========================================================================
      // Getters
      // =========================================================================
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

      // =========================================================================
      // Setters
      // =========================================================================
      int32_t libvibeModel_Sequential_SetMatchingThreshold(
        vibeModel_Sequential_t *model,
        const uint32_t matchingThreshold
      ) {
        assert(model != NULL);
        assert(matchingThreshold > 0);
        model->matchingThreshold = matchingThreshold;
        return(0);
      }

      int32_t libvibeModel_Sequential_SetMatchingNumber(
        vibeModel_Sequential_t *model,
        const uint32_t matchingNumber
      ) {
        assert(model != NULL);
        assert(matchingNumber > 0);
        model->matchingNumber = matchingNumber;
        return(0);
      }

      int32_t libvibeModel_Sequential_SetUpdateFactor(
        vibeModel_Sequential_t *model,
        const uint32_t updateFactor
      ) {
        assert(model != NULL);
        assert(updateFactor > 0);
        model->updateFactor = updateFactor;

        assert(model->jump != NULL);
        int size = (model->width > model->height) ? 2 * model->width + 1 : 2 * model->height + 1;
        for (int i = 0; i < size; ++i)
          model->jump[i] = (updateFactor == 1) ? 1 : (rand() % (2 * model->updateFactor)) + 1;
        return(0);
      }

      // =========================================================================
      // Free
      // =========================================================================
      int32_t libvibeModel_Sequential_Free(vibeModel_Sequential_t *model)
      {
        if (model == NULL)
          return(-1);

        unmap_pixel_proc(model);
        free(model->jump);
        free(model->neighbor);
        free(model->position);
        free(model);

        return(0);
      }

      // =========================================================================
      // AllocInit C3R — MMIO path with DDR buffer
      // =========================================================================
      int32_t libvibeModel_Sequential_AllocInit_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        const uint32_t width,
        const uint32_t height
      ) {
        assert((image_data != NULL) && (model != NULL));
        assert((width > 0) && (height > 0));

        model->width  = width;
        model->height = height;
        model->lastHistorySampleSwapped = 0u;

        // Override defaults with hardware-fixed parameters.
        model->numberOfSamples   = kHistoryFramesHw;
        model->matchingThreshold = kHardwareSadThreshold;
        model->matchingNumber    = kHardwareMatchingNumber;

        const uint32_t pixelCount = width * height;

        // ---- Allocate DDR buffer (pixel-interleaved RGBX, 96 bytes/pixel) ----
        model->fpgaDdrBufferBytes = DdrPixelLayout::totalBytes(pixelCount);
        model->fpgaDdrBuffer = (uint8_t*)malloc(model->fpgaDdrBufferBytes);
        assert(model->fpgaDdrBuffer != NULL);

        // ---- Map pixel_proc registers ----
        if (!map_pixel_proc_regs(model)) {
          fprintf(stderr, "[ViBe] FATAL: failed to map pixel_proc registers\n");
          assert(0 && "map_pixel_proc_regs failed");
        }
        dump_pixel_proc_regs(model->fpgaPixelProcRegs, "after-map");

        // Remap DDR buffer to the default shared-memory physical address.
        if (!map_ddr_buffer(model, kSharedMemoryPhysBase)) {
          fprintf(stderr, "[ViBe] FATAL: failed to map shared memory at 0x%08X\n",
            kSharedMemoryPhysBase);
          assert(0 && "map_ddr_buffer failed");
        }

        // ---- Initialize DDR buffer ----
        auto writeRgbxEntry = [](uint8_t *dst, const uint8_t *src) {
          dst[0] = src[2];  // R
          dst[1] = src[1];  // G
          dst[2] = src[0];  // B
          dst[3] = 0u;      // X
        };

        auto writeRgbxValues = [](uint8_t *dst, uint8_t r, uint8_t g, uint8_t b) {
          dst[0] = r; dst[1] = g; dst[2] = b; dst[3] = 0u;
        };

        auto plusNoise = [](uint8_t value) -> uint8_t {
          int n = value + rand() % 20 - 10;
          if (n < 0) n = 0;
          if (n > 255) n = 255;
          return static_cast<uint8_t>(n);
        };

        memset(model->fpgaDdrBuffer, 0, model->fpgaDdrBufferBytes);

        for (uint32_t pi = 0; pi < pixelCount; ++pi) {
          const uint8_t *pixel = image_data + 3u * pi;

          // Entry 0: current frame
          writeRgbxEntry(model->fpgaDdrBuffer + DdrPixelLayout::currentOffset(pi), pixel);

          // Entries 1..N: history (first 2 seeded from frame, rest with noise)
          for (uint32_t s = 0; s < model->numberOfSamples; ++s) {
            uint8_t *hist = model->fpgaDdrBuffer + DdrPixelLayout::historyOffset(pi, s);
            if (s < NUMBER_OF_HISTORY_IMAGES)
              writeRgbxEntry(hist, pixel);
            else
              writeRgbxValues(hist, plusNoise(pixel[0]), plusNoise(pixel[1]), plusNoise(pixel[2]));
          }
        }

        // ---- Allocate random buffers ----
        int size = (width > height) ? 2 * width + 1 : 2 * height + 1;
        model->jump     = (uint32_t*)malloc(size * sizeof(*(model->jump)));
        model->neighbor = (int*)malloc(size * sizeof(*(model->neighbor)));
        model->position = (uint32_t*)malloc(size * sizeof(*(model->position)));
        assert(model->jump != NULL && model->neighbor != NULL && model->position != NULL);

        for (int i = 0; i < size; ++i) {
          model->jump[i]     = (rand() % (2 * model->updateFactor)) + 1;
          model->neighbor[i] = ((rand() % 3) - 1) + ((rand() % 3) - 1) * width;
          model->position[i] = rand() % model->numberOfSamples;
        }

        return(0);
      }

      // =========================================================================
      // Segmentation C3R — MMIO shared-memory pipeline
      // =========================================================================
      int32_t libvibeModel_Sequential_Segmentation_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *segmentation_map
      ) {
        assert((image_data != NULL) && (model != NULL) && (segmentation_map != NULL));
        assert((model->width > 0) && (model->height > 0));
        assert(model->fpgaDdrBuffer != NULL);
        assert(model->fpgaPixelProcRegs != NULL);
        assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

        const uint32_t width  = model->width;
        const uint32_t height = model->height;
        const uint32_t pixelCount = width * height;
        const uint32_t numberOfSamples = model->numberOfSamples;
        volatile uint32_t *regs = model->fpgaPixelProcRegs;

        memset(segmentation_map, COLOR_BACKGROUND, pixelCount);

        // ---- Stage 1: Refresh entry 0 (current frame) in DDR buffer ----
        for (uint32_t pi = 0; pi < pixelCount; ++pi) {
          uint8_t *cur = model->fpgaDdrBuffer + DdrPixelLayout::currentOffset(pi);
          const uint8_t *src = image_data + 3u * pi;
          cur[0] = src[2];  // R
          cur[1] = src[1];  // G
          cur[2] = src[0];  // B
          cur[3] = 0u;      // X
        }

        const uint32_t inputAddr = model->fpgaDdrPhysBase;
        if (inputAddr != kSharedMemoryPhysBase ||
            model->fpgaDdrBufferBytes > static_cast<size_t>(UINT32_MAX - inputAddr)) {
          fprintf(stderr, "[ViBe] FATAL: invalid shared-memory input address/size\n");
          return(-1);
        }

        const uint32_t outputAddr = inputAddr + static_cast<uint32_t>(model->fpgaDdrBufferBytes);
        const size_t outputBytes = static_cast<size_t>(pixelCount) * kOutputBytesPerPixel;
        if (!shared_memory_contains(inputAddr, model->fpgaDdrBufferBytes) ||
            !shared_memory_contains(outputAddr, outputBytes)) {
          fprintf(stderr,
            "[ViBe] FATAL: input/output buffers exceed shared memory range 0x%08X-0x%08X\n",
            kSharedMemoryPhysBase, kSharedMemoryPhysEnd);
          return(-1);
        }

        uint8_t *fpgaOutputBuffer = model->fpgaDdrBuffer + model->fpgaDdrBufferBytes;
        memset(fpgaOutputBuffer, COLOR_BACKGROUND, outputBytes);

        // ---- Stage 2: Write input/output/count, then start FPGA ----
        regWrite(regs, REG_INPUT_PTR, inputAddr);
        regWrite(regs, REG_OUTPUT_PTR, outputAddr);
        regWrite(regs, REG_PIXEL_COUNT, pixelCount);
        regWrite(regs, REG_START_IDLE, 1u);
        dump_pixel_proc_regs(regs, "after-start");

        // ---- Stage 3: Poll start/idle until hardware returns 0 ----
        const uint32_t maxPolls = env_u32_hex_or_dec("VIBE_FPGA_POLL_LIMIT", 200000u);
        uint32_t poll = 0u;
        while (regRead(regs, REG_START_IDLE) != 0u) {
          if ((poll == 0u) || ((poll % 4096u) == 0u))
            dump_pixel_proc_regs(regs, "poll");
          if (++poll >= maxPolls) {
            dump_pixel_proc_regs(regs, "timeout");
            assert(0 && "FPGA polling timed out");
            return(-1);
          }
          if ((poll % 128u) == 0u) usleep(10);
        }

        // ---- Stage 4: Read output buffer and update software history ----
        auto updateHistoryForPixel = [&](uint32_t pixelIndex) {
          uint32_t slot   = model->position[pixelIndex % (2 * width + 1)];
          uint32_t sampleIndex = slot % numberOfSamples;

          const uint8_t *src = image_data + 3u * pixelIndex;
          uint8_t *hist = model->fpgaDdrBuffer + DdrPixelLayout::historyOffset(pixelIndex, sampleIndex);
          hist[0] = src[2];  // R
          hist[1] = src[1];  // G
          hist[2] = src[0];  // B
          hist[3] = 0u;      // X
        };

        for (uint32_t pi = 0u; pi < pixelCount; ++pi) {
          segmentation_map[pi] = fpgaOutputBuffer[pi];
          if (segmentation_map[pi] == COLOR_BACKGROUND)
            updateHistoryForPixel(pi);
        }

        ++model->fpgaFrameSequence;

        return(0);
      }

      // =========================================================================
      // Update C3R — no-op. History updates happen inline during Segmentation.
      // =========================================================================
      int32_t libvibeModel_Sequential_Update_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *updating_mask
      ) {
        (void)model;
        (void)image_data;
        (void)updating_mask;
        return(0);
      }
    }
  }
}
