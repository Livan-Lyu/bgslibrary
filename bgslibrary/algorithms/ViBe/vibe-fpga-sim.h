#pragma once

#include <stddef.h>
#include <stdint.h>

#include "vibe-background-sequential.h"

namespace bgslibrary
{
  namespace algorithms
  {
    namespace vibe
    {
      size_t libvibeModel_Sequential_FpgaSharedMemoryBytes_8u_C3R(
        uint32_t width,
        uint32_t height,
        uint32_t numberOfSamples
      );

      int32_t libvibeModel_Sequential_SimulateFpgaSharedMemory_8u_C3R(
        const vibeFpgaSharedMemoryView_Sequential_t *view
      );

      int32_t libvibeModel_Sequential_ReadFpgaForegroundMask_1b_C1R(
        const vibeFpgaSharedMemoryView_Sequential_t *view,
        uint8_t *segmentation_map
      );
    }
  }
}
