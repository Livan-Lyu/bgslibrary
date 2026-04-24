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
        static inline int abs_uint(const int value)
        {
          return (value >= 0) ? value : -value;
        }

        static inline size_t foreground_bytes(const uint32_t width, const uint32_t height)
        {
          const size_t pixelCount = static_cast<size_t>(width) * static_cast<size_t>(height);
          return (pixelCount + 7u) / 8u;
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

        static inline void set_foreground_bit(
          const vibeFpgaSharedMemoryView_Sequential_t *view,
          const uint32_t pixelIndex,
          const bool isForeground
        ) {
          const uint32_t byteIndex = pixelIndex >> 3;
          const uint32_t bitIndex = pixelIndex & 0x7u;
          const uint8_t bitMask = static_cast<uint8_t>(1u << bitIndex);

          if (isForeground)
            view->buffer[byteIndex] |= bitMask;
          else
            view->buffer[byteIndex] &= static_cast<uint8_t>(~bitMask);
        }

        static inline bool get_foreground_bit(
          const vibeFpgaSharedMemoryView_Sequential_t *view,
          const uint32_t pixelIndex
        ) {
          const uint32_t byteIndex = pixelIndex >> 3;
          const uint32_t bitIndex = pixelIndex & 0x7u;
          return ((view->buffer[byteIndex] >> bitIndex) & 0x1u) != 0u;
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

        const uint32_t pixelCount = view->width * view->height;
        const uint32_t sampleCount = (view->numberOfSamples < fpga::kHistoryFrames)
          ? view->numberOfSamples
          : fpga::kHistoryFrames;

        for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
          const uint8_t *currentPixel = pixel_slot_ptr(view, pixelIndex, 0u);
          uint32_t matchCount = 0u;

          for (uint32_t sampleIndex = 0; sampleIndex < sampleCount; ++sampleIndex) {
            const uint8_t *historyPixel = pixel_slot_ptr(view, pixelIndex, sampleIndex + 1u);
            const uint32_t sum = static_cast<uint32_t>(abs_uint(currentPixel[0] - historyPixel[0]))
              + static_cast<uint32_t>(abs_uint(currentPixel[1] - historyPixel[1]))
              + static_cast<uint32_t>(abs_uint(currentPixel[2] - historyPixel[2]));

            if (sum <= view->matchingThreshold) {
              ++matchCount;
              if (matchCount >= view->matchingNumber)
                break;
            }
          }

          set_foreground_bit(view, pixelIndex, matchCount < view->matchingNumber);
        }

        return(0);
      }

      int32_t libvibeModel_Sequential_ReadFpgaForegroundMask_1b_C1R(
        const vibeFpgaSharedMemoryView_Sequential_t *view,
        uint8_t *segmentation_map
      ) {
        if (view == NULL || view->buffer == NULL || segmentation_map == NULL)
          return(-1);

        const uint32_t pixelCount = view->width * view->height;
        for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex)
          segmentation_map[pixelIndex] = get_foreground_bit(view, pixelIndex)
            ? COLOR_FOREGROUND
            : COLOR_BACKGROUND;

        return(0);
      }
    }
  }
}
