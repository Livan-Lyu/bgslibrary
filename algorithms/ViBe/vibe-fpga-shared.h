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
      constexpr size_t   kDdrBytesPerPixel      = 72u;   // 24 entries × 3 bytes (RGB)
      constexpr uint32_t kHardwareSadThreshold  = 45u;   // SAD ≤ 45 is a match
      constexpr uint32_t kHardwareMatchingNumber = 2u;   // match_count ≥ 2 → background
      // =======================================================================
      // FPGA hardware interface
      // =======================================================================
      constexpr uint32_t kPixelProcPhysBase       = 0x60000000u;
      constexpr uint32_t kSharedMemoryPhysBase    = 0xC4000000u;
      constexpr uint32_t kSharedMemoryPhysEnd     = 0xC9FFFFFFu;
      constexpr size_t   kOutputBytesPerPixel     = 1u;
      constexpr uint32_t kFpgaBufferCount         = 2u;

      // =======================================================================
      // Register offsets (relative to FPGA base)
      // =======================================================================
      enum PixelProcReg : uint32_t
      {
        REG_START_IDLE  = 0x08,  // RW: write 1 to start, poll until read value is 0
        REG_INPUT_PTR   = 0x10,  // RW: input buffer physical address
        REG_OUTPUT_PTR  = 0x18,  // RW: output buffer physical address
        REG_PIXEL_COUNT = 0x20,  // RW: total pixel count
      };

      // Values written to pixel_proc for one prepared slot.
      struct PixelProcCommand
      {
        uint32_t startIdle;
        uint32_t inputPtr;
        uint32_t outputPtr;
        uint32_t pixelCount;
      };

      // =======================================================================
      // DDR buffer layout (pixel-interleaved, 72 bytes/pixel, 24 RGB entries)
      // =======================================================================
      struct DdrPixelLayout
      {
        static constexpr uint32_t kEntriesPerPixel = 24;
        static constexpr uint32_t kBytesPerEntry   = 3;

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

      // One complete ping-pong slot: model input followed by the output map.
      struct DdrFrameBuffer
      {
        uint32_t physBase;
        uint32_t outputPhysBase;
        uint32_t pixelCount;
        size_t modelBytes;
        size_t outputBytes;
        size_t mappingBytes;
        uint8_t *mapping;
        uint8_t *model;
        uint8_t *output;
        PixelProcCommand command;
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
