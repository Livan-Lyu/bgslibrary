#pragma once

#include <opencv2/opencv.hpp>

#include "ViBe/vibe-fpga-shared.h"
#include "ViBe/vibe-background-sequential.h"

namespace bgslibrary
{
  namespace algorithms
  {
    class ViBe
    {
    private:
      static const int DEFAULT_NUM_SAMPLES = static_cast<int>(vibe::fpga::kHistoryFrames);
      static const int DEFAULT_MATCH_THRESH = 20;
      static const int DEFAULT_MATCH_NUM = 2;
      static const int DEFAULT_UPDATE_FACTOR = 16;

    private:
      int matchingThreshold;
      int matchingNumber;
      int updateFactor;
      bool firstTime;
      vibe::vibeModel_Sequential_t* model;

      // Cached output buffers (reused across frames).
      cv::Mat m_outputFg;
      cv::Mat m_outputBg;
      cv::Size m_lastSize;

    public:
      ViBe();
      ~ViBe();

      void process(const cv::Mat &img_input, cv::Mat &img_output, cv::Mat &img_bgmodel);

    private:
      void init(const cv::Mat &img_input, cv::Mat &img_outfg, cv::Mat &img_outbg);
    };
  }
}
