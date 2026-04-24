#pragma once

#include "IBGS.h"
#include "ViBe/vibe-fpga-shared.h"
#include "ViBe/vibe-background-sequential.h"

namespace bgslibrary
{
  namespace algorithms
  {
    class ViBe : public IBGS
    {
    private:
      static const int DEFAULT_NUM_SAMPLES = static_cast<int>(vibe::fpga::kHistoryFrames);
      static const int DEFAULT_MATCH_THRESH = 90;
      static const int DEFAULT_MATCH_NUM = 2;
      static const int DEFAULT_UPDATE_FACTOR = 16;

    private:
      //int numberOfSamples;
      int matchingThreshold;
      int matchingNumber;
      int updateFactor;
      vibe::vibeModel_Sequential_t* model;

    public:
      ViBe();
      ~ViBe();

      void process(const cv::Mat &img_input, cv::Mat &img_output, cv::Mat &img_bgmodel);

    private:
      void save_config(cv::FileStorage &fs);
      void load_config(cv::FileStorage &fs);
    };

    bgs_register(ViBe);
  }
}
