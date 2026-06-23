#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "vibe-background-sequential.h"
#include "vibe-fpga-pdma.h"

namespace bgslibrary
{
  namespace algorithms
  {
    namespace vibe
    {
      namespace
      {
        // -----------------------------------------------------------------------
        // Low-level register I/O (same pattern as existing fpga_reg_read/write)
        // -----------------------------------------------------------------------
#if defined(__linux__)
        static inline void reg_write(volatile uint32_t *base, const fpga::PixelProcReg offset, const uint32_t value)
        {
          base[static_cast<uint32_t>(offset) >> 2] = value;
          __sync_synchronize();
        }

        static inline uint32_t reg_read(volatile uint32_t *base, const fpga::PixelProcReg offset)
        {
          __sync_synchronize();
          return base[static_cast<uint32_t>(offset) >> 2];
        }
#endif

        // -----------------------------------------------------------------------
        // CPU fallback: write the entire DDR buffer to PIXEL_DATA word by word.
        // This is correct but slow — PDMA should be used for real-time video.
        // -----------------------------------------------------------------------
        static void cpu_feed_pixels(
            const uint8_t *ddr_buffer,
            const uint32_t total_pixels,
            volatile uint32_t *regs)
        {
          const uint32_t total_entries = total_pixels * fpga::kEntriesPerPixel;
          const uint32_t report_step = total_entries / 10u;  // 10 progress steps

          std::cerr << "[FPGA PDMA] CPU feeding " << total_entries
                    << " entries (" << total_pixels << " pixels × "
                    << fpga::kEntriesPerPixel << " entries/pixel)..." << std::endl;

          for (uint32_t i = 0u; i < total_entries; ++i) {
            // Read 4 bytes (little-endian RGBX) and write to PIXEL_DATA.
            const uint32_t word =
              (static_cast<uint32_t>(ddr_buffer[i * 4u + 0u]) << 0)  |
              (static_cast<uint32_t>(ddr_buffer[i * 4u + 1u]) << 8)  |
              (static_cast<uint32_t>(ddr_buffer[i * 4u + 2u]) << 16) |
              (static_cast<uint32_t>(ddr_buffer[i * 4u + 3u]) << 24);
            reg_write(regs, fpga::PixelProcReg::PIXEL_DATA, word);

            if (report_step > 0u && (i % report_step) == 0u)
              std::cerr << "[FPGA PDMA]   progress: " << (i / report_step * 10) << "%"
                        << std::endl;
          }

          std::cerr << "[FPGA PDMA] CPU feeding done." << std::endl;
        }

        // -----------------------------------------------------------------------
        // PDMA stub — replace with real PDMA controller register writes.
        //
        // The caller provides the DDR physical address; the implementation
        // should configure:
        //   src_addr     = ddr_phys_addr
        //   dst_addr     = 0x4110008C (PIXEL_DATA, fixed)
        //   src_inc      = 1 (byte-incrementing)
        //   dst_inc      = 0 (fixed address)
        //   transfer_len = total_pixels * 96 bytes
        //   trigger      = start transfer
        //
        // Returns true if PDMA was successfully started, false to fall back.
        // -----------------------------------------------------------------------
        static bool pdma_start_transfer(
            const uint32_t /*ddr_phys_addr*/,
            const uint32_t /*total_pixels*/)
        {
          // TODO: write PDMA controller registers for the target platform.
          // Returning false causes the CPU fallback to be used.
          return false;
        }

        // -----------------------------------------------------------------------
        // Poll until PDMA transfer completes (if PDMA was used).
        // -----------------------------------------------------------------------
        static bool pdma_wait_done()
        {
          // TODO: poll PDMA status register or wait for interrupt.
          // For the CPU-fallback path this is a no-op because cpu_feed_pixels
          // is synchronous.
          return true;
        }

      } // anonymous namespace

      // =======================================================================
      // Public API
      // =======================================================================

      bool pdma_launch_pixel_proc(
          const uint8_t *ddr_buffer,
          const uint32_t ddr_phys_addr,
          const uint32_t total_pixels,
          volatile uint32_t *pixel_proc_regs,
          uint8_t *segmentation_map,
          const uint32_t width)
      {
        if (ddr_buffer == nullptr || pixel_proc_regs == nullptr || segmentation_map == nullptr)
          return false;

        if (total_pixels == 0u)
          return true;  // nothing to do

#if !defined(__linux__)
        (void)ddr_phys_addr;
        (void)width;
        return false;
#else
        // 1. Write total pixel count.
        reg_write(pixel_proc_regs, fpga::PixelProcReg::PIXEL_COUNT, total_pixels);

        // 2. Assert START.
        reg_write(pixel_proc_regs, fpga::PixelProcReg::CONTROL,
                  static_cast<uint32_t>(fpga::PixelProcControl::START));

        // 3. Feed pixel data — try PDMA first, fall back to CPU writes.
        const bool pdma_ok = pdma_start_transfer(ddr_phys_addr, total_pixels);
        if (!pdma_ok) {
          cpu_feed_pixels(ddr_buffer, total_pixels, pixel_proc_regs);
        }

        // 4. Process results in batches of 32 pixels.
        uint32_t pixels_processed = 0u;
        const uint32_t full_batches = total_pixels / fpga::kResultBatchSize;
        const uint32_t remainder   = total_pixels % fpga::kResultBatchSize;
        const uint32_t total_batches = full_batches + (remainder > 0u ? 1u : 0u);

        // Read poll limit from env (same convention as VIBE_FPGA_POLL_LIMIT).
        const uint32_t pollLimit = []() -> uint32_t {
          const char *v = getenv("VIBE_FPGA_POLL_LIMIT");
          if (v == nullptr || *v == '\0') return 200000u;
          return static_cast<uint32_t>(strtoul(v, nullptr, 0));
        }();

        for (uint32_t batch = 0u; batch < total_batches; ++batch) {
          // Wait for READY with timeout.
          {
            uint32_t poll = 0u;
            for (; poll < pollLimit; ++poll) {
              const uint32_t status = reg_read(pixel_proc_regs, fpga::PixelProcReg::STATUS);
              if ((status & static_cast<uint32_t>(fpga::PixelProcStatus::READY)) != 0u)
                break;
            }
            if (poll >= pollLimit) {
              std::cerr << "[FPGA PDMA] ERROR: READY timeout after " << pollLimit
                        << " polls (batch " << batch << "/" << total_batches
                        << "). Is the FPGA loaded and clocked?" << std::endl;
              return false;
            }
          }

          // Read the 32 packed foreground bits.
          const uint32_t result = reg_read(pixel_proc_regs, fpga::PixelProcReg::RESULT);

          // Acknowledge: write ACK then clear it.
          reg_write(pixel_proc_regs, fpga::PixelProcReg::CONTROL,
                    static_cast<uint32_t>(fpga::PixelProcControl::ACK));
          reg_write(pixel_proc_regs, fpga::PixelProcReg::CONTROL, 0u);

          // Unpack bits into the segmentation map.
          const uint32_t batch_count = (batch < full_batches)
              ? fpga::kResultBatchSize
              : remainder;
          for (uint32_t i = 0u; i < batch_count; ++i) {
            const uint32_t pixelIndex = pixels_processed + i;
            segmentation_map[pixelIndex] = ((result >> i) & 1u) != 0u
                ? static_cast<uint8_t>(COLOR_FOREGROUND)
                : static_cast<uint8_t>(COLOR_BACKGROUND);
          }

          pixels_processed += batch_count;
        }

        // 5. Wait for DONE with timeout.
        {
          uint32_t poll = 0u;
          for (; poll < pollLimit; ++poll) {
            const uint32_t status = reg_read(pixel_proc_regs, fpga::PixelProcReg::STATUS);
            if ((status & static_cast<uint32_t>(fpga::PixelProcStatus::DONE)) != 0u)
              break;
          }
          if (poll >= pollLimit) {
            std::cerr << "[FPGA PDMA] ERROR: DONE timeout after " << pollLimit
                      << " polls. Pixels processed: " << pixels_processed
                      << " / " << total_pixels << std::endl;
            return false;
          }
        }

        // 6. If PDMA was used, wait for it to finish.
        if (pdma_ok)
          pdma_wait_done();

        // 7. Clear START.
        reg_write(pixel_proc_regs, fpga::PixelProcReg::CONTROL, 0u);

        (void)width;  // reserved for debug logging
        return true;
#endif
      }

    } // namespace vibe
  }   // namespace algorithms
}     // namespace bgslibrary
