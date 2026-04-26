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
        static uint64_t g_fpga_sim_frame_counter = 0u;

        static inline int abs_int(const int value)
        {
          return (value >= 0) ? value : -value;
        }

        static inline uint32_t read_word32(const uint8_t *base, const size_t offset)
        {
          const uint8_t *ptr = base + offset;
          return static_cast<uint32_t>(ptr[0])
            | (static_cast<uint32_t>(ptr[1]) << 8)
            | (static_cast<uint32_t>(ptr[2]) << 16)
            | (static_cast<uint32_t>(ptr[3]) << 24);
        }

        static inline void write_word32(uint8_t *base, const size_t offset, const uint32_t value)
        {
          uint8_t *ptr = base + offset;
          ptr[0] = static_cast<uint8_t>(value & 0xFFu);
          ptr[1] = static_cast<uint8_t>((value >> 8) & 0xFFu);
          ptr[2] = static_cast<uint8_t>((value >> 16) & 0xFFu);
          ptr[3] = static_cast<uint8_t>((value >> 24) & 0xFFu);
        }

        static inline uint32_t current_pixel_offset(
          const vibeFpgaSharedMemoryView_Sequential_t *view,
          const uint32_t pixelIndex
        ) {
          return static_cast<uint32_t>(view->frameOffset + static_cast<size_t>(pixelIndex) * fpga::kInputPixelBytes);
        }

        static inline uint32_t history_pixel_offset(
          const vibeFpgaSharedMemoryView_Sequential_t *view,
          const uint32_t pixelCount,
          const uint32_t historyIndex,
          const uint32_t pixelIndex
        ) {
          return static_cast<uint32_t>(
            view->historyOffset
            + (static_cast<size_t>(historyIndex) * static_cast<size_t>(pixelCount) + static_cast<size_t>(pixelIndex))
              * fpga::kInputPixelBytes
          );
        }

        static inline uint32_t output_word_offset(
          const vibeFpgaSharedMemoryView_Sequential_t *view,
          const uint32_t pixelIndex
        ) {
          return static_cast<uint32_t>(view->outOffset + static_cast<size_t>(pixelIndex) * fpga::kOutputWordBytes);
        }

        static inline bool compare_bgrx32(
          const uint32_t currentPixel,
          const uint32_t historyPixel,
          const uint32_t matchingThreshold
        ) {
          const uint32_t sad = static_cast<uint32_t>(abs_int(static_cast<int>(currentPixel & 0xFFu) - static_cast<int>(historyPixel & 0xFFu)))
            + static_cast<uint32_t>(abs_int(static_cast<int>((currentPixel >> 8) & 0xFFu) - static_cast<int>((historyPixel >> 8) & 0xFFu)))
            + static_cast<uint32_t>(abs_int(static_cast<int>((currentPixel >> 16) & 0xFFu) - static_cast<int>((historyPixel >> 16) & 0xFFu)));
          return sad <= matchingThreshold;
        }
      }

      size_t libvibeModel_Sequential_FpgaSharedMemoryBytes_8u_C3R(
        uint32_t width,
        uint32_t height,
        uint32_t numberOfSamples
      ) {
        (void)numberOfSamples;
        return fpga::MakeSharedLayout(width, height).total_bytes;
      }

      int32_t libvibeModel_Sequential_SimulateFpgaSharedMemory_8u_C3R(
        const vibeFpgaSharedMemoryView_Sequential_t *view
      ) {
        if (view == NULL || view->buffer == NULL)
          return(-1);

        fpga::ControlBlock *control = reinterpret_cast<fpga::ControlBlock*>(view->buffer + view->controlOffset);
        if (control->magic != fpga::kControlMagic || control->version != fpga::kControlVersion)
          return(-1);

        const uint64_t frameIndex = g_fpga_sim_frame_counter++;
        const uint32_t pixelCount = view->width * view->height;
        const uint32_t historyFrames = (view->numberOfSamples < fpga::kHistoryFrames)
          ? view->numberOfSamples
          : fpga::kHistoryFrames;

        control->start = 0u;
        control->done = 0u;
        control->status = fpga::kStatusBusy;
        control->error_code = 0u;

        std::cerr
          << "[FPGA-SIM] enter frame=" << frameIndex
          << " width=" << view->width
          << " height=" << view->height
          << " history_frames=" << historyFrames
          << " threshold=" << view->matchingThreshold
          << std::endl;

        for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
          const uint32_t currentPixel = read_word32(view->buffer, current_pixel_offset(view, pixelIndex));
          uint32_t comparisonWord = 0u;

          for (uint32_t historyIndex = 0; historyIndex < historyFrames; ++historyIndex) {
            const uint32_t historyPixel = read_word32(
              view->buffer,
              history_pixel_offset(view, pixelCount, historyIndex, pixelIndex)
            );

            if (compare_bgrx32(currentPixel, historyPixel, view->matchingThreshold))
              comparisonWord |= (1u << historyIndex);
          }

          write_word32(view->buffer, output_word_offset(view, pixelIndex), comparisonWord);
        }

        control->done = 1u;
        control->status = fpga::kStatusDone;
        control->error_code = 0u;

        std::cerr
          << "[FPGA-SIM] leave frame=" << frameIndex
          << " pixelCount=" << pixelCount
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
        for (uint32_t pixelIndex = 0; pixelIndex < pixelCount; ++pixelIndex) {
          const uint32_t comparisonWord = read_word32(view->buffer, output_word_offset(view, pixelIndex));
          segmentation_map[pixelIndex] = fpga::IsForeground(comparisonWord, view->matchingNumber)
            ? COLOR_FOREGROUND
            : COLOR_BACKGROUND;
        }

        return(0);
      }
    }
  }
}
