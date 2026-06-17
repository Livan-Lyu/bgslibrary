#pragma once

#include <stddef.h>
#include <stdint.h>

namespace bgslibrary
{
  namespace algorithms
  {
    namespace vibe
    {
      namespace fpga
      {
        // ---------------------------------------------------------------------------
        // Pixel-proc PDMA FPGA constants (see FPGA_API.md for the hardware spec)
        // ---------------------------------------------------------------------------

        // Physical base of the CAPE APB slot that contains pixel_proc.
        constexpr uint32_t kPixelProcPhysBase = 0x41100000u;

        // pixel_proc registers start at offset 0x80 within the APB slot
        // (paddr[7] == 1 selects pixel_proc rather than apb_ctrl_status).
        constexpr uint32_t kPixelProcRegOffset = 0x80u;

        // Hardware-defined frame dimensions.
        constexpr uint32_t kHistoryFramesPdma = 23u;    // 23 history samples per pixel
        constexpr uint32_t kEntriesPerPixel = 24u;      // 1 current + 23 history
        constexpr uint32_t kDdrBytesPerPixel = 96u;     // 24 entries × 4 bytes each
        constexpr uint32_t kInputPixelBytesPdma = 4u;   // R, G, B, X (padding)

        // The FPGA returns foreground bits in batches of 32.
        constexpr uint32_t kResultBatchSize = 32u;

        // Hard-coded by the FPGA RTL — software cannot change these.
        // Threshold 4.5 × T where T = 10  →  SAD ≤ 45 for a match.
        constexpr uint32_t kHardwareSadThreshold = 45u;
        // Foreground when match_count > 2, i.e. matching_number = 3.
        constexpr uint32_t kHardwareMatchingNumber = 3u;

        // ---------------------------------------------------------------------------
        // pixel_proc register map (offsets relative to pixel_proc base = 0x41100080)
        // ---------------------------------------------------------------------------
        enum class PixelProcReg : uint32_t
        {
          CONTROL     = 0x00u,   // RW  [0]=START, [7]=ACK
          STATUS      = 0x04u,   // RO  [0]=BUSY, [1]=READY, [2]=DONE
          // 0x08 reserved
          PIXEL_DATA  = 0x0Cu,   // WO  [23:0] RGB – PDMA writes here
          PIXEL_COUNT = 0x10u,   // RW  total pixel count N
          RESULT      = 0x14u,   // RO  packed 32 foreground bits ([0]=first pixel)
          DEBUG       = 0x18u    // RO  fixed 0xDEADBEEF
        };

        // ---------------------------------------------------------------------------
        // STATUS register bit definitions
        // ---------------------------------------------------------------------------
        enum class PixelProcStatus : uint32_t
        {
          BUSY  = 1u << 0,
          READY = 1u << 1,
          DONE  = 1u << 2
        };

        // ---------------------------------------------------------------------------
        // CONTROL register bit definitions
        // ---------------------------------------------------------------------------
        enum class PixelProcControl : uint32_t
        {
          START = 0x01u,
          ACK   = 0x80u
        };

        // ---------------------------------------------------------------------------
        // DDR buffer layout helpers
        //
        // The DDR buffer is pixel-interleaved:
        //   [pixel0_entry0] [pixel0_entry1] ... [pixel0_entry23]
        //   [pixel1_entry0] [pixel1_entry1] ... [pixel1_entry23]
        //   ...
        //
        // Entry 0  = current frame pixel
        // Entry 1..23 = history samples
        //
        // Each entry is 4 bytes: [R(7:0), G(15:8), B(23:16), 0x00]
        // ---------------------------------------------------------------------------
        struct DdrPixelLayout
        {
          static constexpr size_t currentEntry() { return 0u; }

          static constexpr size_t historyEntry(const uint32_t sampleIndex)
          {
            return static_cast<size_t>(1u + sampleIndex);
          }

          static size_t pixelBlockOffset(const uint32_t pixelIndex)
          {
            return static_cast<size_t>(pixelIndex) * kDdrBytesPerPixel;
          }

          static size_t entryOffset(const uint32_t pixelIndex, const uint32_t entryIndex)
          {
            return pixelBlockOffset(pixelIndex)
              + static_cast<size_t>(entryIndex) * kInputPixelBytesPdma;
          }

          static size_t currentOffset(const uint32_t pixelIndex)
          {
            return entryOffset(pixelIndex, currentEntry());
          }

          static size_t historyOffset(const uint32_t pixelIndex, const uint32_t sampleIndex)
          {
            return entryOffset(pixelIndex, historyEntry(sampleIndex));
          }

          static size_t totalBytes(const uint32_t totalPixels)
          {
            return static_cast<size_t>(totalPixels) * kDdrBytesPerPixel;
          }
        };

        // ---------------------------------------------------------------------------
        // Pack a pixel into the RGBX32 little-endian word expected by the hardware.
        // Byte 0 = R, Byte 1 = G, Byte 2 = B, Byte 3 = 0x00.
        // ---------------------------------------------------------------------------
        inline uint32_t PackRgbx32(const uint8_t r, const uint8_t g, const uint8_t b)
        {
          return static_cast<uint32_t>(r)
            | (static_cast<uint32_t>(g) << 8)
            | (static_cast<uint32_t>(b) << 16);
        }

        // ---------------------------------------------------------------------------
        // Unpack helpers – extract channel values from a packed RGBX32 word.
        // ---------------------------------------------------------------------------
        inline uint8_t UnpackR(const uint32_t rgbx)
        {
          return static_cast<uint8_t>(rgbx & 0xFFu);
        }

        inline uint8_t UnpackG(const uint32_t rgbx)
        {
          return static_cast<uint8_t>((rgbx >> 8) & 0xFFu);
        }

        inline uint8_t UnpackB(const uint32_t rgbx)
        {
          return static_cast<uint8_t>((rgbx >> 16) & 0xFFu);
        }

      } // namespace fpga

      // -----------------------------------------------------------------------
      // PDMA transport function (defined in vibe-fpga-pdma.cpp).
      // -----------------------------------------------------------------------

      /**
       * Launch the pixel_proc FPGA compare pass via PDMA (or CPU fallback).
       *
       * Feeds pixel data from the DDR buffer (96 bytes/pixel, pixel-interleaved,
       * RGBX byte order) into the FPGA, then polls for batched 32-bit foreground
       * results and writes them directly into @p segmentation_map.
       *
       * @param ddr_buffer       Virtual address of the DDR source buffer (96 B/pixel).
       * @param ddr_phys_addr    Physical address of the DDR buffer (for PDMA).
       * @param total_pixels     Total pixel count (width × height).
       * @param pixel_proc_regs  Mapped pixel_proc registers (base 0x41100080).
       * @param segmentation_map Output foreground mask (1 byte/pixel, 0 or 255).
       * @param width            Image width (used only for debug logging).
       * @return true on success, false on error or timeout.
       */
      bool pdma_launch_pixel_proc(
          const uint8_t *ddr_buffer,
          uint32_t ddr_phys_addr,
          uint32_t total_pixels,
          volatile uint32_t *pixel_proc_regs,
          uint8_t *segmentation_map,
          uint32_t width);

    }   // namespace vibe
  }     // namespace algorithms
}       // namespace bgslibrary
