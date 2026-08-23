#include <assert.h>
#include <chrono>
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

        /* The one and only background model maintained by software. */
        uint8_t       *backgroundModel;
        size_t         backgroundModelBytes;

        /* Two independent work slots used as ping-pong staging buffers. */
        DdrFrameBuffer fpgaDdrBuffers[kFpgaBufferCount];
        size_t         fpgaDdrSlotBytes;
        int            fpgaDdrMemFd;

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
        const uint32_t sharedMemoryEnd = env_u32_hex_or_dec(
          "VIBE_FPGA_DDR_END", kSharedMemoryPhysEnd);
        return begin >= kSharedMemoryPhysBase &&
          end <= static_cast<uint64_t>(sharedMemoryEnd) && end >= begin;
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

      static bool map_ddr_buffer(
        vibeModel_Sequential_t *model,
        const uint32_t slotIndex,
        const uint32_t physBase
      )
      {
        if (model == NULL || slotIndex >= kFpgaBufferCount || physBase == 0u)
          return false;

        const long pageSize = sysconf(_SC_PAGESIZE);
        if (pageSize <= 0)
          return false;

        DdrFrameBuffer &buffer = model->fpgaDdrBuffers[slotIndex];
        const off_t mapBase = static_cast<off_t>(physBase & ~(static_cast<uint32_t>(pageSize) - 1u));
        const size_t mapDelta = static_cast<size_t>(physBase - static_cast<uint32_t>(mapBase));
        if (!shared_memory_contains(physBase, model->fpgaDdrSlotBytes))
          return false;

        const size_t mapBytes = fpga::AlignUp(model->fpgaDdrSlotBytes + mapDelta,
          static_cast<size_t>(pageSize));

        if (model->fpgaDdrMemFd < 0) {
          model->fpgaDdrMemFd = open("/dev/mem", O_RDWR | O_SYNC);
          if (model->fpgaDdrMemFd < 0)
            return false;
        }

        void *mapping = mmap(
          NULL, mapBytes, PROT_READ | PROT_WRITE, MAP_SHARED, model->fpgaDdrMemFd, mapBase);
        if (mapping == MAP_FAILED) {
          return false;
        }

        buffer.physBase = physBase;
        buffer.outputPhysBase = physBase + static_cast<uint32_t>(buffer.modelBytes);
        buffer.mapping = reinterpret_cast<uint8_t*>(mapping);
        buffer.mappingBytes = mapBytes;
        buffer.model = buffer.mapping + mapDelta;
        buffer.output = buffer.model + buffer.modelBytes;
        buffer.command.inputPtr = buffer.physBase;
        buffer.command.outputPtr = buffer.outputPhysBase;
        buffer.command.pixelCount = buffer.pixelCount;
        buffer.command.startIdle = 0u;
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

        for (uint32_t i = 0u; i < kFpgaBufferCount; ++i) {
          DdrFrameBuffer &buffer = model->fpgaDdrBuffers[i];
          if (buffer.mapping != NULL) {
            munmap(buffer.mapping, buffer.mappingBytes);
            buffer.mapping = NULL;
          }
          buffer.model = NULL;
          buffer.output = NULL;
          buffer.mappingBytes = 0u;
        }
        free(model->backgroundModel);
        model->backgroundModel = NULL;
        model->backgroundModelBytes = 0u;

        if (model->fpgaDevMemFd >= 0) {
          close(model->fpgaDevMemFd);
          model->fpgaDevMemFd = -1;
        }
        if (model->fpgaDdrMemFd >= 0) {
          close(model->fpgaDdrMemFd);
          model->fpgaDdrMemFd = -1;
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
        memset(model->fpgaDdrBuffers, 0, sizeof(model->fpgaDdrBuffers));
        model->backgroundModel         = NULL;
        model->backgroundModelBytes    = 0u;
        model->fpgaDdrSlotBytes          = 0u;
        model->fpgaDdrMemFd              = -1;
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

      static void write_current_frame(
        DdrFrameBuffer &buffer,
        const uint8_t *image_data,
        const uint32_t pixelCount
      )
      {
        for (uint32_t pi = 0u; pi < pixelCount; ++pi) {
          uint8_t *cur = buffer.model + DdrPixelLayout::currentOffset(pi);
          const uint8_t *src = image_data + 3u * pi;
          cur[0] = src[2];  // R
          cur[1] = src[1];  // G
          cur[2] = src[0];  // B
        }
      }

      static void initialize_slot(
        uint8_t *backgroundModel,
        const uint8_t *image_data,
        const uint32_t pixelCount,
        const uint32_t numberOfSamples
      )
      {
        auto writeRgbEntry = [](uint8_t *dst, const uint8_t *src) {
          dst[0] = src[2];  // R
          dst[1] = src[1];  // G
          dst[2] = src[0];  // B
        };

        auto writeRgbValues = [](uint8_t *dst, uint8_t r, uint8_t g, uint8_t b) {
          dst[0] = r; dst[1] = g; dst[2] = b;
        };

        auto plusNoise = [](uint8_t value) -> uint8_t {
          int n = value + rand() % 20 - 10;
          if (n < 0) n = 0;
          if (n > 255) n = 255;
          return static_cast<uint8_t>(n);
        };

        const size_t backgroundBytes = DdrPixelLayout::totalBytes(pixelCount);
        memset(backgroundModel, 0, backgroundBytes);

        for (uint32_t pi = 0u; pi < pixelCount; ++pi) {
          const uint8_t *pixel = image_data + 3u * pi;
          writeRgbEntry(backgroundModel + DdrPixelLayout::currentOffset(pi), pixel);

          for (uint32_t s = 0u; s < numberOfSamples; ++s) {
            uint8_t *hist = backgroundModel + DdrPixelLayout::historyOffset(pi, s);
            if (s < NUMBER_OF_HISTORY_IMAGES) {
              writeRgbEntry(hist, pixel);
            } else {
              writeRgbValues(
                hist,
                plusNoise(pixel[0]),
                plusNoise(pixel[1]),
                plusNoise(pixel[2]));
            }
          }
        }
      }

      static void copy_background_model_to_slot(
        const vibeModel_Sequential_t *model,
        DdrFrameBuffer &buffer
      )
      {
        assert(model != NULL);
        assert(model->backgroundModel != NULL);
        assert(buffer.model != NULL);
        memcpy(buffer.model, model->backgroundModel, model->backgroundModelBytes);
      }

      // =========================================================================
      // AllocInit C3R — two MMIO slots for asynchronous ping-pong processing
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

        const uint64_t pixelCount64 =
          static_cast<uint64_t>(width) * static_cast<uint64_t>(height);
        if (pixelCount64 == 0u || pixelCount64 > UINT32_MAX)
          return(-1);
        const uint32_t pixelCount = static_cast<uint32_t>(pixelCount64);
        const size_t modelBytes = DdrPixelLayout::totalBytes(pixelCount);
        const size_t outputBytes =
          static_cast<size_t>(pixelCount) * kOutputBytesPerPixel;
        if (modelBytes > SIZE_MAX - outputBytes)
          return(-1);
        model->backgroundModelBytes = modelBytes;
        model->backgroundModel = static_cast<uint8_t*>(malloc(model->backgroundModelBytes));
        if (model->backgroundModel == NULL)
          return(-1);
        model->fpgaDdrSlotBytes = modelBytes + outputBytes;

        const long pageSize = sysconf(_SC_PAGESIZE);
        if (pageSize <= 0)
          return(-1);
        const size_t slotStride = fpga::AlignUp(
          model->fpgaDdrSlotBytes, static_cast<size_t>(pageSize));
        const uint32_t bufferABase = env_u32_hex_or_dec(
          "VIBE_FPGA_DDR_A_BASE", kSharedMemoryPhysBase);
        const uint64_t defaultBBase64 =
          static_cast<uint64_t>(bufferABase) + static_cast<uint64_t>(slotStride);
        if (defaultBBase64 > UINT32_MAX)
          return(-1);
        const uint32_t bufferBBase = env_u32_hex_or_dec(
          "VIBE_FPGA_DDR_B_BASE", static_cast<uint32_t>(defaultBBase64));

        const uint64_t aEnd = static_cast<uint64_t>(bufferABase) +
          static_cast<uint64_t>(model->fpgaDdrSlotBytes);
        const uint64_t bEnd = static_cast<uint64_t>(bufferBBase) +
          static_cast<uint64_t>(model->fpgaDdrSlotBytes);
        if (bufferABase == 0u || bufferBBase == 0u ||
            (bufferABase < bufferBBase && aEnd > bufferBBase) ||
            (bufferBBase < bufferABase && bEnd > bufferABase) ||
            !shared_memory_contains(bufferABase, model->fpgaDdrSlotBytes) ||
            !shared_memory_contains(bufferBBase, model->fpgaDdrSlotBytes)) {
          const uint32_t sharedMemoryEnd = env_u32_hex_or_dec(
            "VIBE_FPGA_DDR_END", kSharedMemoryPhysEnd);
          fprintf(stderr,
            "[ViBe] FATAL: double-buffer DDR layout does not fit: "
            "slot=%zu bytes, A=0x%08X, B=0x%08X, end=0x%08X\n",
            model->fpgaDdrSlotBytes, bufferABase, bufferBBase, sharedMemoryEnd);
          return(-1);
        }

        // ---- Map pixel_proc registers ----
        if (!map_pixel_proc_regs(model)) {
          fprintf(stderr, "[ViBe] FATAL: failed to map pixel_proc registers\n");
          return(-1);
        }
        dump_pixel_proc_regs(model->fpgaPixelProcRegs, "after-map");

        for (uint32_t i = 0u; i < kFpgaBufferCount; ++i) {
          DdrFrameBuffer &buffer = model->fpgaDdrBuffers[i];
          buffer.physBase = (i == 0u) ? bufferABase : bufferBBase;
          buffer.pixelCount = pixelCount;
          buffer.modelBytes = modelBytes;
          buffer.outputBytes = outputBytes;

          if (!map_ddr_buffer(model, i, buffer.physBase)) {
            fprintf(stderr,
              "[ViBe] FATAL: failed to map DDR slot %u at 0x%08X\n",
              i, buffer.physBase);
            unmap_pixel_proc(model);
            return(-1);
          }
        }

        initialize_slot(
          model->backgroundModel,
          image_data,
          pixelCount,
          model->numberOfSamples);

        // Warm-up: both slots start from the same unique model snapshot.
        for (uint32_t i = 0u; i < kFpgaBufferCount; ++i) {
          copy_background_model_to_slot(model, model->fpgaDdrBuffers[i]);
          memset(
            model->fpgaDdrBuffers[i].output,
            COLOR_BACKGROUND,
            model->fpgaDdrBuffers[i].outputBytes);
        }

        fprintf(stdout,
          "[ViBe] double buffer ready: slotBytes=%zu, A=0x%08X, B=0x%08X\n",
          model->fpgaDdrSlotBytes,
          model->fpgaDdrBuffers[0].physBase,
          model->fpgaDdrBuffers[1].physBase);

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
      // Slot preparation — CPU side of the asynchronous pipeline
      // =========================================================================
      int32_t libvibeModel_Sequential_PrepareSlot_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint32_t slotIndex,
        const uint8_t *image_data
      ) {
        if (model == NULL || image_data == NULL || slotIndex >= kFpgaBufferCount)
          return(-1);

        DdrFrameBuffer &buffer = model->fpgaDdrBuffers[slotIndex];
        if (buffer.model == NULL || buffer.output == NULL ||
            model->backgroundModel == NULL)
          return(-1);

        copy_background_model_to_slot(model, buffer);
        write_current_frame(buffer, image_data, buffer.pixelCount);
        memset(buffer.output, COLOR_BACKGROUND, buffer.outputBytes);
        return(0);
      }

      // =========================================================================
      // Slot segmentation — FPGA side of the asynchronous pipeline
      // =========================================================================
      int32_t libvibeModel_Sequential_SegmentSlot_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint32_t slotIndex,
        uint8_t *segmentation_map,
        double *fpga_latency_ms
      ) {
        if (model == NULL || slotIndex >= kFpgaBufferCount)
          return(-1);

        DdrFrameBuffer &buffer = model->fpgaDdrBuffers[slotIndex];
        if (buffer.model == NULL || buffer.output == NULL ||
            model->fpgaPixelProcRegs == NULL)
          return(-1);

        volatile uint32_t *regs = model->fpgaPixelProcRegs;

        if (!shared_memory_contains(buffer.physBase, model->fpgaDdrSlotBytes) ||
            !shared_memory_contains(buffer.outputPhysBase, buffer.outputBytes)) {
          fprintf(stderr,
            "[ViBe] FATAL: slot %u is outside the configured shared memory\n",
            slotIndex);
          return(-1);
        }

          // ---- Stage 1: Submit this slot to pixel_proc ----
        regWrite(regs, REG_INPUT_PTR, buffer.command.inputPtr);
        regWrite(regs, REG_OUTPUT_PTR, buffer.command.outputPtr);
        regWrite(regs, REG_PIXEL_COUNT, buffer.command.pixelCount);
          const auto startTime = std::chrono::steady_clock::now();
        regWrite(regs, REG_START_IDLE, 1u);
        dump_pixel_proc_regs(regs, "after-start");

        // ---- Stage 2: Poll start/idle until hardware returns 0 ----
        const uint32_t maxPolls = env_u32_hex_or_dec("VIBE_FPGA_POLL_LIMIT", 1000000u);
        uint32_t poll = 0u;
        while (regRead(regs, REG_START_IDLE) != 0u) {
          if ((poll == 0u) || ((poll % 4096u) == 0u))
            dump_pixel_proc_regs(regs, "poll");
          if (++poll >= maxPolls) {
            dump_pixel_proc_regs(regs, "timeout");
            return(-1);
          }
          if ((poll % 128u) == 0u)
            usleep(10);
        }
        __sync_synchronize();

          if (fpga_latency_ms != NULL) {
            const auto endTime = std::chrono::steady_clock::now();
            *fpga_latency_ms = std::chrono::duration<double, std::milli>(
              endTime - startTime).count();
          }

        ++model->fpgaFrameSequence;
        return(0);
      }

      // =========================================================================
      // Slot commit — update the unique background model from a completed slot
      // =========================================================================
      int32_t libvibeModel_Sequential_CommitSlot_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint32_t slotIndex,
        uint8_t *segmentation_map
      ) {
        if (model == NULL || slotIndex >= kFpgaBufferCount)
          return(-1);

        DdrFrameBuffer &buffer = model->fpgaDdrBuffers[slotIndex];
        if (buffer.model == NULL || buffer.output == NULL ||
            model->backgroundModel == NULL)
          return(-1);

        const uint32_t width = model->width;
        const uint32_t pixelCount = buffer.pixelCount;
        const uint32_t numberOfSamples = model->numberOfSamples;

        for (uint32_t pi = 0u; pi < pixelCount; ++pi) {
          const uint8_t result = buffer.output[pi];
          if (segmentation_map != NULL)
            segmentation_map[pi] = result;

          if (result == COLOR_BACKGROUND) {
            const uint32_t randomSlot =
              model->position[pi % (2u * width + 1u)];
            const uint32_t sampleIndex = randomSlot % numberOfSamples;
            const uint8_t *src =
              buffer.model + DdrPixelLayout::currentOffset(pi);
            uint8_t *dstCurrent =
              model->backgroundModel + DdrPixelLayout::currentOffset(pi);
            uint8_t *hist =
              model->backgroundModel + DdrPixelLayout::historyOffset(pi, sampleIndex);
            memcpy(dstCurrent, src, DdrPixelLayout::kBytesPerEntry);
            memcpy(hist, src, DdrPixelLayout::kBytesPerEntry);
          }
        }

        return(0);
      }

      // Compatibility adapter for the original synchronous API.
      int32_t libvibeModel_Sequential_Segmentation_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *segmentation_map
      ) {
        if (libvibeModel_Sequential_PrepareSlot_8u_C3R(
              model, 0u, image_data) != 0)
          return(-1);
        if (libvibeModel_Sequential_SegmentSlot_8u_C3R(
            model, 0u, segmentation_map, NULL) != 0)
          return(-1);
        return libvibeModel_Sequential_CommitSlot_8u_C3R(
          model, 0u, segmentation_map);
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
