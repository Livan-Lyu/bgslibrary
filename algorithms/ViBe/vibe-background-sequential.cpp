#include <assert.h>
#include <iostream>
#include <stdio.h>
#include <string.h>

#if defined(__linux__)
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

#include "vibe-fpga-sim.h"
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

        // ---- pixel_proc / DDR buffer (MMIO + sim paths) ----
        uint8_t  *fpgaDdrBuffer;
        size_t    fpgaDdrBufferBytes;
        uint32_t  fpgaDdrPhysBase;        // 0 = heap, else physical address via /dev/mem
        void     *fpgaDdrMapping;
        size_t    fpgaDdrMappingBytes;

        // ---- pixel_proc registers ----
        bool               fpgaUseMmio;
        volatile uint32_t *fpgaPixelProcRegs;   // mmap'd hardware regs (MMIO path, NULL for sim)
        void              *fpgaPixelProcMapping;
        size_t             fpgaPixelProcMappingBytes;
        int                fpgaDevMemFd;
        uint64_t           fpgaFrameSequence;

        // ---- Software register file (sim path only) ----
        PixelProcSimRegs   fpgaSimRegs;

        /* Buffers with random values. */
        uint32_t *jump;
        int      *neighbor;
        uint32_t *position;
      };

      // =========================================================================
      // pixel_proc register I/O
      // =========================================================================
#if defined(__linux__)
      static inline void hw_reg_write(volatile uint32_t *regs, uint32_t offset, uint32_t value)
      {
        regs[offset >> 2] = value;
        __sync_synchronize();
      }

      static inline uint32_t hw_reg_read(volatile uint32_t *regs, uint32_t offset)
      {
        __sync_synchronize();
        return regs[offset >> 2];
      }
#endif

      // Unified register accessors (dispatch on MMIO vs sim).
      static inline void regWrite(vibeModel_Sequential_t *model, uint32_t offset, uint32_t value)
      {
        if (model->fpgaUseMmio) {
#if defined(__linux__)
          hw_reg_write(model->fpgaPixelProcRegs, offset, value);
#endif
        } else {
          switch (offset) {
            case REG_CONTROL:      model->fpgaSimRegs.control = value; break;
            case REG_STATUS:       /* RO */ break;
            case REG_SRC_ADDR_LO:  /* sim ignores */ break;
            case REG_SRC_ADDR_HI:  /* sim ignores */ break;
            case REG_PIXEL_COUNT:  model->fpgaSimRegs.pixelCount = value; break;
            case REG_RESULT:       /* RO */ break;
            default: break;
          }
        }
      }

      static inline uint32_t regRead(vibeModel_Sequential_t *model, uint32_t offset)
      {
        if (model->fpgaUseMmio) {
#if defined(__linux__)
          return hw_reg_read(model->fpgaPixelProcRegs, offset);
#else
          return 0u;
#endif
        } else {
          switch (offset) {
            case REG_CONTROL:      return model->fpgaSimRegs.control;
            case REG_STATUS:       return model->fpgaSimRegs.status;
            case REG_SRC_ADDR_LO:  return 0u;
            case REG_SRC_ADDR_HI:  return 0u;
            case REG_PIXEL_COUNT:  return model->fpgaSimRegs.pixelCount;
            case REG_RESULT:       return model->fpgaSimRegs.result;
            default: return 0u;
          }
        }
      }

      // =========================================================================
      // pixel_proc register mapping — UIO first, /dev/mem fallback
      // =========================================================================
#if defined(__linux__)
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
        size_t mapBytes = 0x1000u;  // UIO: 4 KB region; /dev/mem: at least 0x100

        // ---- Try UIO first ----
        const char *uioPath = get_uio_device();
        fd = open(uioPath, O_RDWR);
        if (fd < 0) {
            std::cerr << "UIO open failed: " << strerror(errno) << std::endl;
        }
        if (fd >= 0) {
          // UIO mmap: offset=0 maps the entire device region starting at the
          // physical base address (0x41300000).  No page alignment math needed.
          // regRead/regWrite use absolute offsets (0x80, 0x84, ...) from CAPE
          // base, so fpgaPixelProcRegs must point to the UIO mapping start.
          mapping = mmap(NULL, mapBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
          if (mapping != MAP_FAILED) {
            model->fpgaPixelProcRegs = reinterpret_cast<volatile uint32_t*>(mapping);

            std::cout << "[ViBe] pixel_proc mapped via " << uioPath
                      << " (base 0x" << std::hex << kPixelProcPhysBase << std::dec << ")"
                      << std::endl;
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
          mapBytes = fpga::AlignUp(0x100u + mapDelta, static_cast<size_t>(pageSize));

          fd = open("/dev/mem", O_RDWR | O_SYNC);
          if (fd < 0)
            return false;

          mapping = mmap(NULL, mapBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, mapBase);
          if (mapping == MAP_FAILED) {
            close(fd);
            return false;
          }

          // /dev/mem: fpgaPixelProcRegs points to CAPE base + delta.
          // regRead/regWrite use absolute offsets (0x80, 0x84, ...).
          model->fpgaPixelProcRegs = reinterpret_cast<volatile uint32_t*>(
            reinterpret_cast<uint8_t*>(mapping) + mapDelta);

          std::cout << "[ViBe] pixel_proc mapped via /dev/mem (fallback)" << std::endl;
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
        const size_t mapBytes = fpga::AlignUp(model->fpgaDdrBufferBytes + mapDelta, static_cast<size_t>(pageSize));

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
#endif

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
        model->fpgaUseMmio              = fpga_transport_use_mmio();

        /* Sim register file (always zeroed; only used when !fpgaUseMmio). */
        memset(&model->fpgaSimRegs, 0, sizeof(model->fpgaSimRegs));

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

#if defined(__linux__)
        unmap_pixel_proc(model);
#else
        free(model->fpgaDdrBuffer);
#endif
        free(model->jump);
        free(model->neighbor);
        free(model->position);
        free(model);

        return(0);
      }

      // =========================================================================
      // AllocInit C3R — unified MMIO + sim path with DDR buffer
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

        // If MMIO, override software defaults with hardware-fixed parameters.
        if (model->fpgaUseMmio) {
          model->numberOfSamples   = kHistoryFramesHw;
          model->matchingThreshold = kHardwareSadThreshold;
          model->matchingNumber    = kHardwareMatchingNumber;
        }

        const uint32_t pixelCount = width * height;

        // ---- Allocate DDR buffer (pixel-interleaved RGBX, 96 bytes/pixel) ----
        model->fpgaDdrBufferBytes = DdrPixelLayout::totalBytes(pixelCount);
        model->fpgaDdrBuffer = (uint8_t*)malloc(model->fpgaDdrBufferBytes);
        assert(model->fpgaDdrBuffer != NULL);
        
        // ---- MMIO: map pixel_proc registers, optionally remap DDR to physical ----
        if (model->fpgaUseMmio) {
          std::cout << "checking for UIO device..." << std::endl;
#if defined(__linux__)
          if (!map_pixel_proc_regs(model)) {
            model->fpgaUseMmio = false;  // fall back to sim
          } else {
            uint32_t ddrPhys = env_u32_hex_or_dec("VIBE_FPGA_DDR_BASE", 0u);
            if (ddrPhys != 0u) {
              map_ddr_buffer(model, ddrPhys);
            }
          }
          std::cout << "check done" << std::endl;
#else
          model->fpgaUseMmio = false;
#endif
        }

        // ---- Initialize DDR buffer ----
        // Helper: write RGBX entry (OpenCV BGR → RGBX)
        // change data of one pixel to 32bit, and the order is R G B X, X is 0. Easier for AXI to read.
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

        const uint32_t seededSamples = seeded_sample_count(model->numberOfSamples);
        for (uint32_t pi = 0; pi < pixelCount; ++pi) {
          const uint8_t *pixel = image_data + 3u * pi;

          // Entry 0: current frame
          writeRgbxEntry(model->fpgaDdrBuffer + DdrPixelLayout::currentOffset(pi), pixel);

          // Entries 1..N: history
          for (uint32_t s = 0; s < model->numberOfSamples; ++s) {
            uint8_t *hist = model->fpgaDdrBuffer + DdrPixelLayout::historyOffset(pi, s);
            if (s < seededSamples)
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

        std::cout << "AllocInit " << width << "x" << height
                  << " pixels=" << pixelCount
                  << " ddrBytes=" << model->fpgaDdrBufferBytes
                  << "Allocated DDR buffer start address: " << model->fpgaDdrBuffer
                  << " mmio=" << (model->fpgaUseMmio ? "yes" : "no(sim)")
                  << " samples=" << model->numberOfSamples
                  << " threshold=" << model->matchingThreshold
                  << " matchNum=" << model->matchingNumber
                  << std::endl;

        return(0);
      }

      // =========================================================================
      // Segmentation C3R — unified batched pipeline (MMIO + sim)
      // =========================================================================
      int32_t libvibeModel_Sequential_Segmentation_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *segmentation_map
      ) {
        assert((image_data != NULL) && (model != NULL) && (segmentation_map != NULL));
        assert((model->width > 0) && (model->height > 0));
        assert(model->fpgaDdrBuffer != NULL);
        assert((model->jump != NULL) && (model->neighbor != NULL) && (model->position != NULL));

        const uint32_t width  = model->width;
        const uint32_t height = model->height;
        const uint32_t pixelCount = width * height;
        const uint32_t numberOfSamples = model->numberOfSamples;

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

        // ---- Stage 2: Write DDR address + start FPGA ----
        // Order of reg: SRC_ADDR_LO, SRC_ADDR_HI, PIXEL_COUNT, START.
        if (model->fpgaUseMmio) {
          uint64_t ddrPhys = static_cast<uint64_t>(model->fpgaDdrPhysBase);
          regWrite(model, REG_SRC_ADDR_LO, static_cast<uint32_t>(ddrPhys & 0xFFFFFFFFu));
          regWrite(model, REG_SRC_ADDR_HI, static_cast<uint32_t>(ddrPhys >> 32));
        }
        regWrite(model, REG_PIXEL_COUNT, pixelCount);
        regWrite(model, REG_CONTROL, kControlStart);

        if (!model->fpgaUseMmio) {
          simPixelProcInit(&model->fpgaSimRegs,
                           model->fpgaDdrBuffer, pixelCount, width, height,
                           segmentation_map,
                           model->matchingThreshold, model->matchingNumber);
          simPixelProcTick(&model->fpgaSimRegs);  // first batch
        }

        // ---- Stage 3: Batch loop (matches API.md §四 step 4) ----
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

        uint32_t done = 0u;
        uint32_t left = pixelCount;

        while (!done) {
          // Wait READY (API.md: while ((STATUS & 0x02) == 0))
          {
            uint32_t poll = 0u;
            const uint32_t maxPolls = model->fpgaUseMmio
              ? env_u32_hex_or_dec("VIBE_FPGA_POLL_LIMIT", 200000u)
              : 1u;
            uint32_t status;
            do {
              status = regRead(model, REG_STATUS);
              if (status & fpga::kStatusError) { done = 1u; break; }
              if (model->fpgaUseMmio && (++poll >= maxPolls)) { done = 1u; break; }
#if defined(__linux__)
              if (model->fpgaUseMmio && (poll % 128u) == 0u) usleep(10);
#endif
            } while (!(status & kStatusReady));
            if (done) break;
          }

          // Read RESULT + ACK (API.md: single write of 0x80).
          uint32_t r = regRead(model, REG_RESULT);
          regWrite(model, REG_CONTROL, kControlAck);

          if (!model->fpgaUseMmio)
            simPixelProcClearReady(&model->fpgaSimRegs);

          // Process batch (API.md: for i < 32 && left > 0).
          for (uint32_t i = 0u; i < kPixelsPerBatch && left > 0u; ++i, --left) {
            uint32_t pixelIndex = pixelCount - left;
            if (r & (1u << i)) {
              segmentation_map[pixelIndex] = COLOR_FOREGROUND;
            } else {
              segmentation_map[pixelIndex] = COLOR_BACKGROUND;
              updateHistoryForPixel(pixelIndex);
            }
          }

          // Advance sim.
          if (!model->fpgaUseMmio)
            simPixelProcTick(&model->fpgaSimRegs);

          // Check DONE (API.md: after inner loop).
          if (regRead(model, REG_STATUS) & kStatusDone)
            done = 1u;
        }

        // Clear START (API.md §四 step 5).
        regWrite(model, REG_CONTROL, 0u);

        ++model->fpgaFrameSequence;

        // Debug output.
        {
          uint32_t fgCount = 0u;
          for (uint32_t i = 0; i < pixelCount; ++i)
            if (segmentation_map[i] == COLOR_FOREGROUND) ++fgCount;

          std::cerr << "[ViBe C3R Seg] frame=" << (model->fpgaFrameSequence - 1)
                    << " fg=" << fgCount
                    << " bg=" << (pixelCount - fgCount)
                    << " mmio=" << (model->fpgaUseMmio ? "yes" : "sim")
                    << std::endl;
        }

        return(0);
      }

      // =========================================================================
      // Update C3R — no-op. History updates happen inline during the
      // Segmentation batch loop (both MMIO and SIM paths).
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
