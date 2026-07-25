#pragma once

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace bgslibrary
{
  namespace algorithms
  {
    namespace vibe
    {
      const int COLOR_BACKGROUND = 0;
      const int COLOR_FOREGROUND = 255;
      const int NUMBER_OF_HISTORY_IMAGES = 2;

      /** Opaque model structure. */
      typedef struct vibeModel_Sequential vibeModel_Sequential_t;

      // -----------------------------------------------------------------------
      // Lifecycle
      // -----------------------------------------------------------------------
      vibeModel_Sequential_t *libvibeModel_Sequential_New();
      int32_t libvibeModel_Sequential_Free(vibeModel_Sequential_t *model);

      // -----------------------------------------------------------------------
      // Parameters
      // -----------------------------------------------------------------------
      uint32_t libvibeModel_Sequential_PrintParameters(const vibeModel_Sequential_t *model);

      uint32_t libvibeModel_Sequential_GetNumberOfSamples(const vibeModel_Sequential_t *model);
      uint32_t libvibeModel_Sequential_GetMatchingThreshold(const vibeModel_Sequential_t *model);
      uint32_t libvibeModel_Sequential_GetMatchingNumber(const vibeModel_Sequential_t *model);
      uint32_t libvibeModel_Sequential_GetUpdateFactor(const vibeModel_Sequential_t *model);

      int32_t libvibeModel_Sequential_SetMatchingThreshold(vibeModel_Sequential_t *model, const uint32_t matchingThreshold);
      int32_t libvibeModel_Sequential_SetMatchingNumber(vibeModel_Sequential_t *model, const uint32_t matchingNumber);
      int32_t libvibeModel_Sequential_SetUpdateFactor(vibeModel_Sequential_t *model, const uint32_t updateFactor);

      // -----------------------------------------------------------------------
      // C3R (3-channel color) — FPGA-accelerated
      // -----------------------------------------------------------------------
      int32_t libvibeModel_Sequential_AllocInit_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        const uint32_t width,
        const uint32_t height
      );

      int32_t libvibeModel_Sequential_PrepareSlot_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint32_t slotIndex,
        const uint8_t *image_data
      );

      int32_t libvibeModel_Sequential_SegmentSlot_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint32_t slotIndex,
        uint8_t *segmentation_map
      );

      int32_t libvibeModel_Sequential_CommitSlot_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint32_t slotIndex,
        uint8_t *segmentation_map
      );

      int32_t libvibeModel_Sequential_Segmentation_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *segmentation_map
      );

      int32_t libvibeModel_Sequential_Update_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *updating_mask
      );
    }
  }
}
