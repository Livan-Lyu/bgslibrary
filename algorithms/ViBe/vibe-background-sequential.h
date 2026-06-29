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
      const int COLOR_BACKGROUND = 0; // Default label for background pixels
      const int COLOR_FOREGROUND = 255; // Default label for foreground pixels. Note that some authors chose any value different from 0 instead
      const int NUMBER_OF_HISTORY_IMAGES = 2;

      /**
       * \typedef struct vibeModel_Sequential_t
       * \brief Data structure for the background subtraction model.
       *
       * This data structure contains the background model as well as some paramaters value.
       * The code is designed to hide all the implementation details to the user to ease its use.
       */
      typedef struct vibeModel_Sequential vibeModel_Sequential_t;

      /**
       * Allocation of a new data structure where the background model will be stored.
       * Please note that this function only creates the structure to host the data.
       * This data structures will only be filled with a call to \ref libvibeModel_Sequential_AllocInit_8u_C3R.
       *
       * \result A pointer to a newly allocated \ref vibeModel_Sequential_t
       * structure, or <tt>NULL</tt> in the case of an error.
       */
      vibeModel_Sequential_t *libvibeModel_Sequential_New();

      /**
       * ViBe uses several parameters.
       * You can print and change some of them if you want. However, default
       * value should meet your needs for most videos.
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @return
       */
      uint32_t libvibeModel_Sequential_PrintParameters(const vibeModel_Sequential_t *model);

      /**
       * Getter.
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @return
       */
      uint32_t libvibeModel_Sequential_GetNumberOfSamples(const vibeModel_Sequential_t *model);

      /**
       * Setter.
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @param matchingThreshold
       * @return
       */
      int32_t libvibeModel_Sequential_SetMatchingThreshold(
        vibeModel_Sequential_t *model,
        const uint32_t matchingThreshold
      );

      /**
       * Setter.
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @return
       */
      uint32_t libvibeModel_Sequential_GetMatchingThreshold(const vibeModel_Sequential_t *model);

      /**
       * Setter.
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @param matchingNumber
       * @return
       */
      int32_t libvibeModel_Sequential_SetMatchingNumber(
        vibeModel_Sequential_t *model,
        const uint32_t matchingNumber
      );

      /**
       * Setter.
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @param updateFactor New value for the update factor. Please note that the update factor is to be understood as a probability of updating. More specifically, an update factor of 16 means that 1 out of every 16 background pixels is updated. Likewise, an update factor of 1 means that every background pixel is updated.
       * @return
       */
      int32_t libvibeModel_Sequential_SetUpdateFactor(
        vibeModel_Sequential_t *model,
        const uint32_t updateFactor
      );

      /**
       * Getter.
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @return
       */
      uint32_t libvibeModel_Sequential_GetMatchingNumber(const vibeModel_Sequential_t *model);


      /**
       * Getter.
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @return
       */
      uint32_t libvibeModel_Sequential_GetUpdateFactor(const vibeModel_Sequential_t *model);

      /**
       * \brief Frees all the memory used by the <tt>model</tt> and deallocates the structure.
       *
       * This function frees all the memory allocated by \ref libvibeModel_SequentialNew and
       * \ref libvibeModel_Sequential_AllocInit_8u_C3R.
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @return
       */
      int32_t libvibeModel_Sequential_Free(vibeModel_Sequential_t *model);

      // -------------------------  Three channel images (C3R) -------------------------
      /**
       * Allocates and initializes the model with the first frame (3-channel color).
       * Pixel values are arranged in BGR order (OpenCV default).
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @param image_data Pointer to the pixel buffer of the first image.
       * @param width Image width in pixels.
       * @param height Image height in pixels.
       * @return 0 on success.
       */
      int32_t libvibeModel_Sequential_AllocInit_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        const uint32_t width,
        const uint32_t height
      );

      /**
       * Classifies pixels using the background model and stores the result
       * in segmentation_map (0 = background, 255 = foreground).
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @param image_data Pointer to the pixel buffer of the current frame (BGR).
       * @param segmentation_map Output foreground mask (single-channel, 0 or 255).
       * @return 0 on success.
       */
      int32_t libvibeModel_Sequential_Segmentation_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *segmentation_map
      );

      /**
       * Updates the background model according to the segmentation results.
       * Only pixels classified as background are used for the update.
       *
       * @param model The data structure with ViBe's background subtraction model and parameters.
       * @param image_data Pointer to the pixel buffer of the current frame (BGR).
       * @param updating_mask Foreground mask from segmentation (0 = background).
       * @return 0 on success.
       */
      int32_t libvibeModel_Sequential_Update_8u_C3R(
        vibeModel_Sequential_t *model,
        const uint8_t *image_data,
        uint8_t *updating_mask
      );
    }
  }
}
