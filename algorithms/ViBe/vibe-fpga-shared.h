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
        // Frozen CPU/FPGA contract for the first offload pass.
        // The compare engine consumes 20 logical history frames and returns one
        // 32-bit word per pixel where bits [19:0] carry the compare results.
        // CPU may still keep a ring buffer internally, but the shared history
        // region exposed to FPGA is linearized before each launch.
        constexpr uint32_t kSharedMemoryPhysBase = 0xC4000000u;
        constexpr uint32_t kHistoryFrames = 20u;
        constexpr uint32_t kInputPixelBytes = 4u;   // B, G, R, X
        constexpr uint32_t kOutputWordBytes = 4u;   // uint32_t / pixel
        constexpr uint32_t kValidOutputBits = kHistoryFrames;
        constexpr uint32_t kDmaAlignmentBytes = 64u;
        constexpr uint32_t kControlMagic = 0x56494245u;   // "VIBE"
        constexpr uint32_t kControlVersion = 1u;
        constexpr uint32_t kApbRegisterSpanBytes = 0x1000u;

        enum RegisterOffsets : uint32_t
        {
          kRegControl = 0x00u,
          kRegControlBlockAddr = 0x04u,
          kRegDebugPixelIndex = 0x08u,
          kRegDebugHistoryIndex = 0x0Cu,
          kRegDebugCompareWord = 0x10u,
          kRegDebugErrorCode = 0x14u
        };

        enum StatusFlags : uint32_t
        {
          kStatusIdle = 0u,
          kStatusBusy = 1u << 0,
          kStatusDone = 1u << 1,
          kStatusError = 1u << 2
        };

        struct ControlBlock
        {
          uint32_t magic;
          uint32_t version;
          uint32_t width;
          uint32_t height;
          uint32_t threshold;
          uint32_t history_frames;
          uint32_t matching_number;
          uint32_t frame_sequence;
          uint32_t start;
          uint32_t done;
          uint32_t status;
          uint32_t error_code;
          uint32_t frame_offset;
          uint32_t history_offset;
          uint32_t out_offset;
          uint32_t control_bytes;
          uint32_t frame_bytes;
          uint32_t history_bytes;
          uint32_t out_bytes;
          uint32_t history_write_index;
          uint32_t reserved[12];
        };

        struct SharedLayout
        {
          size_t frame_offset;
          size_t history_offset;
          size_t out_offset;
          size_t control_offset;
          size_t frame_bytes;
          size_t history_bytes;
          size_t out_bytes;
          size_t total_bytes;
        };

        inline size_t AlignUp(size_t value, size_t alignment)
        {
          return (value + alignment - 1u) / alignment * alignment;
        }

        inline size_t FrameBytes(uint32_t width, uint32_t height)
        {
          return static_cast<size_t>(width) * static_cast<size_t>(height) * kInputPixelBytes;
        }

        inline size_t HistoryBytes(uint32_t width, uint32_t height)
        {
          return FrameBytes(width, height) * kHistoryFrames;
        }

        inline size_t OutputBytes(uint32_t width, uint32_t height)
        {
          return static_cast<size_t>(width) * static_cast<size_t>(height) * kOutputWordBytes;
        }

        inline SharedLayout MakeSharedLayout(uint32_t width, uint32_t height)
        {
          SharedLayout layout{};
          layout.frame_offset = 0u;
          layout.frame_bytes = FrameBytes(width, height);
          layout.history_offset = AlignUp(layout.frame_offset + layout.frame_bytes, kDmaAlignmentBytes);
          layout.history_bytes = HistoryBytes(width, height);
          layout.out_offset = AlignUp(layout.history_offset + layout.history_bytes, kDmaAlignmentBytes);
          layout.out_bytes = OutputBytes(width, height);
          layout.control_offset = AlignUp(layout.out_offset + layout.out_bytes, kDmaAlignmentBytes);
          layout.total_bytes = AlignUp(layout.control_offset + sizeof(ControlBlock), kDmaAlignmentBytes);
          return layout;
        }

        inline uint32_t PackBgrx32(uint8_t b, uint8_t g, uint8_t r)
        {
          return static_cast<uint32_t>(b)
            | (static_cast<uint32_t>(g) << 8)
            | (static_cast<uint32_t>(r) << 16);
        }

        inline uint32_t ValidCompareMask()
        {
          return (1u << kValidOutputBits) - 1u;
        }

        inline bool GetCompareBit(uint32_t comparison_word, uint32_t history_index)
        {
          return history_index < kHistoryFrames
            && ((comparison_word >> history_index) & 0x1u) != 0u;
        }

        inline uint32_t CountMatches(uint32_t comparison_word)
        {
          uint32_t value = comparison_word & ValidCompareMask();
          uint32_t count = 0u;

          while (value != 0u) {
            value &= (value - 1u);
            ++count;
          }

          return count;
        }

        inline bool IsBackground(uint32_t comparison_word, uint32_t matching_number)
        {
          return CountMatches(comparison_word) >= matching_number;
        }

        inline bool IsForeground(uint32_t comparison_word, uint32_t matching_number)
        {
          return !IsBackground(comparison_word, matching_number);
        }

        inline ControlBlock MakeControlBlock(
          uint32_t width,
          uint32_t height,
          uint32_t threshold,
          uint32_t matching_number,
          uint32_t frame_sequence,
          uint32_t history_write_index
        ) {
          const SharedLayout layout = MakeSharedLayout(width, height);

          ControlBlock control{};
          control.magic = kControlMagic;
          control.version = kControlVersion;
          control.width = width;
          control.height = height;
          control.threshold = threshold;
          control.history_frames = kHistoryFrames;
          control.matching_number = matching_number;
          control.frame_sequence = frame_sequence;
          control.start = 0u;
          control.done = 0u;
          control.status = kStatusIdle;
          control.error_code = 0u;
          control.frame_offset = static_cast<uint32_t>(layout.frame_offset);
          control.history_offset = static_cast<uint32_t>(layout.history_offset);
          control.out_offset = static_cast<uint32_t>(layout.out_offset);
          control.control_bytes = static_cast<uint32_t>(sizeof(ControlBlock));
          control.frame_bytes = static_cast<uint32_t>(layout.frame_bytes);
          control.history_bytes = static_cast<uint32_t>(layout.history_bytes);
          control.out_bytes = static_cast<uint32_t>(layout.out_bytes);
          control.history_write_index = history_write_index;
          return control;
        }
      }
    }
  }
}
