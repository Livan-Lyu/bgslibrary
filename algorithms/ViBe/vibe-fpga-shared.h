#pragma once

#include <stddef.h>
#include <stdint.h>

namespace bgslibrary
{
  namespace algorithms
  {
    namespace vibe
    {
      // =======================================================================
      // pixel_proc hardware-fixed parameters
      // =======================================================================
      constexpr uint32_t kHistoryFramesHw       = 23u;   // 24 entries = 1 current + 23 history
      constexpr size_t   kDdrBytesPerPixel      = 96u;   // 24 entries × 4 bytes (RGBX)
      constexpr uint32_t kHardwareSadThreshold  = 45u;   // SAD ≤ 45 is a match
      constexpr uint32_t kHardwareMatchingNumber = 2u;   // match_count ≥ 2 → background
      constexpr uint32_t kPixelsPerBatch        = 32u;   // 32 results per RESULT read

      // =======================================================================
      // CAPE base address
      // =======================================================================
      constexpr uint32_t kPixelProcPhysBase  = 0x41300000u;
      constexpr uint32_t kPixelProcRegOffset = 0x80u;

      // =======================================================================
      // Register offsets (absolute, relative to CAPE base)
      // =======================================================================
      enum PixelProcReg : uint32_t
      {
        REG_CONTROL     = 0x80,  // RW: [0]=START, [7]=ACK
        REG_STATUS      = 0x84,  // RO: [0]=BUSY, [1]=IRQ/data ready, [2]=DONE
        REG_SRC_ADDR_LO = 0x88,  // RW: DDR physical address low 32 bits
        REG_SRC_ADDR_HI = 0x8C,  // RW: DDR physical address high 32 bits
        REG_PIXEL_COUNT = 0x90,  // RW: total pixel count
        REG_RESULT      = 0x94,  // RO: 32 packed foreground bits
        REG_DEBUG       = 0x98,  // RO: 0xDEADBEEF
      };

      // =======================================================================
      // CONTROL register bits
      // =======================================================================
      constexpr uint32_t kControlStart = 0x01;
      constexpr uint32_t kControlAck   = 0x80;

      // =======================================================================
      // STATUS register bits
      // =======================================================================
      constexpr uint32_t kStatusBusy = 0x01;
      constexpr uint32_t kStatusIrq  = 0x02;
      constexpr uint32_t kStatusDone = 0x04;

      // =======================================================================
      // DDR buffer layout (pixel-interleaved, 96 bytes/pixel, 24 entries)
      // =======================================================================
      struct DdrPixelLayout
      {
        static constexpr uint32_t kEntriesPerPixel = 24;
        static constexpr uint32_t kBytesPerEntry   = 4;

        static size_t pixelBlockOffset(uint32_t pixelIndex)
        {
          return static_cast<size_t>(pixelIndex) * kDdrBytesPerPixel;
        }

        static size_t currentOffset(uint32_t pixelIndex)
        {
          return pixelBlockOffset(pixelIndex);
        }

        static size_t historyOffset(uint32_t pixelIndex, uint32_t sampleIndex)
        {
          return pixelBlockOffset(pixelIndex) + (1u + sampleIndex) * kBytesPerEntry;
        }

        static size_t totalBytes(uint32_t totalPixels)
        {
          return static_cast<size_t>(totalPixels) * kDdrBytesPerPixel;
        }
      };

      // =======================================================================
      // Shared FPGA utilities (legacy namespace)
      // =======================================================================
      namespace fpga
      {
        constexpr uint32_t kHistoryFrames = 20u;  // default sample count

        inline size_t AlignUp(size_t value, size_t alignment)
        {
          return (value + alignment - 1u) / alignment * alignment;
        }
      }
    }
  }
}
