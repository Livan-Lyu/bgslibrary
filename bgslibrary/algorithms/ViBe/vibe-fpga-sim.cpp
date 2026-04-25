#include <iostream>

#include "vibe-fpga-shared.h"
#include "vibe-fpga-sim.h"

namespace bgslibrary
{
  namespace algorithms
  {
    namespace vibe
    {
      namespace
      {
        constexpr uint32_t kDebugPixels = 4u;
        constexpr uint32_t kDebugSamples = 6u;
        static uint64_t g_fpga_sim_frame_counter = 0u;

        static inline int abs_uint(const int value)
        {
          return (value >= 0) ? value : -value;
        }

        static inline size_t foreground_bytes(const uint32_t width, const uint32_t height)
        {
          return static_cast<size_t>(width) * static_cast<size_t>(height);
        }

        static inline uint8_t *pixel_slot_ptr(
          const vibeFpgaSharedMemoryView_Sequential_t *view,
          const uint32_t pixelIndex,
          const uint32_t slotIndex
        ) {
          return view->buffer
            + view->foregroundBytes
            + (pixelIndex * (view->numberOfSamples + 1u) + slotIndex) * fpga::kInputPixelBytes;
        }

        static inline void set_foreground_flag(
          const vibeFpgaSharedMemoryView_Sequential_t *view,
          const uint32_t pixelIndex,
          const bool isForeground
        ) {
          view->buffer[pixelIndex] = isForeground ? 1u : 0u;
        }

        static inline bool get_foreground_flag(
          const vibeFpgaSharedMemoryView_Sequential_t *view,
          const uint32_t pixelIndex
        ) {
          return view->buffer[pixelIndex] != 0u;
        }
      }

      size_t libvibeModel_Sequential_FpgaSharedMemoryBytes_8u_C3R(
        uint32_t width,
        uint32_t height,
        uint32_t numberOfSamples
      ) {
        const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
        const size_t maskBytes = foreground_bytes(width, height);
        const size_t pixelBytes = pixelCount
          * static_cast<size_t>(numberOfSamples + 1u)
          * fpga::kInputPixelBytes;
        return maskBytes + pixelBytes;
      }

      int32_t libvibeModel_Sequential_SimulateFpgaSharedMemory_8u_C3R(
        const vibeFpgaSharedMemoryView_Sequential_t *view
      ) {
        if (view == NULL || view->buffer == NULL)
          return(-1);

        const uint64_t frameIndex = g_fpga_sim_frame_counter++;
        const bool debugEnabled = true;
        const uint32_t pixelCount = view->width * view->height;
        const uint32_t sampleCount = (view->numberOfSamples < fpga::kHistoryFrames)
          ? view->numberOfSamples
          : fpga::kHistoryFrames;
        uint32_t foregroundCount = 0u;

        std::cerr
          << "[FPGA-SIM] enter frame=" << frameIndex
          << " width=" << view->width
          << " height=" << view->height
          << " samples=" << sampleCount
          << " threshold=" << view->matchingThreshold
          << " matchingNumber=" << view->matchingNumber
          << std::endl;

        for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
          const uint8_t *currentPixel = pixel_slot_ptr(view, pixelIndex, 0u);
          uint32_t matchCount = 0u;
          const bool pixelDebug = (frameIndex < 3u) && pixelIndex < kDebugPixels;

          if (pixelDebug) {
            std::cerr
              << "[FPGA-SIM] pixel=" << pixelIndex
              << " current=("
              << static_cast<int>(currentPixel[0]) << ","
              << static_cast<int>(currentPixel[1]) << ","
              << static_cast<int>(currentPixel[2]) << ")"
              << std::endl;
          }

          for (uint32_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex) {
            const uint8_t *historyPixel = pixel_slot_ptr(view, pixelIndex, sampleIndex + 1u);
            const uint32_t sum = static_cast<uint32_t>(abs_uint(currentPixel[0] - historyPixel[0]))
              + static_cast<uint32_t>(abs_uint(currentPixel[1] - historyPixel[1]))
              + static_cast<uint32_t>(abs_uint(currentPixel[2] - historyPixel[2]));
            const bool matched = (sum <= view->matchingThreshold);

            if (pixelDebug && sampleIndex < kDebugSamples) {
              std::cerr
                << "  sample=" << sampleIndex
                << " history=("
                << static_cast<int>(historyPixel[0]) << ","
                << static_cast<int>(historyPixel[1]) << ","
                << static_cast<int>(historyPixel[2]) << ")"
                << " sad=" << sum
                << " matched=" << matched
                << std::endl;
            }

            if (matched) {
              ++matchCount;
              if (matchCount >= view->matchingNumber)
                break;
            }
          }

          const bool isForeground = (matchCount < view->matchingNumber);
          set_foreground_flag(view, pixelIndex, isForeground);
          if (isForeground)
            ++foregroundCount;

          if (pixelDebug) {
            std::cerr
              << "  matchCount=" << matchCount
              << " foreground=" << isForeground
              << std::endl;
          }
        }

        std::cerr
          << "[FPGA-SIM] leave frame=" << frameIndex
          << " foregroundCount=" << foregroundCount
          << " backgroundCount=" << (pixelCount - foregroundCount)
          << std::endl;

        return(0);
      }

      int32_t libvibeModel_Sequential_ReadFpgaForegroundMask_8u_C1R(
        const vibeFpgaSharedMemoryView_Sequential_t *view,
        uint8_t *segmentation_map
      ) {
        if (view == NULL || view->buffer == NULL || segmentation_map == NULL)
          return(-1);

        const uint32_t pixelCount = view->width * view->height;
        for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex)
          segmentation_map[pixelIndex] = get_foreground_flag(view, pixelIndex)
            ? COLOR_FOREGROUND
            : COLOR_BACKGROUND;

        return(0);
      }
    }
  }
}
