#pragma once

#include <array>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>

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

      enum class SlotState
      {
        Free,
        Preparing,
        Ready,
        Running,
        Complete
      };

      struct WorkItem
      {
        uint64_t frameNumber;
        uint32_t slotIndex;
      };

      std::thread worker;
      std::mutex pipelineMutex;
      std::condition_variable workAvailable;
      std::condition_variable resultAvailable;
      std::deque<WorkItem> workQueue;
      std::array<SlotState, vibe::kFpgaBufferCount> slotStates;
      std::array<uint64_t, vibe::kFpgaBufferCount> slotFrameNumbers;
      uint64_t nextFrameNumber;
      bool initialized;
      bool stopRequested;
      bool workerFailed;
      std::string workerError;

    public:
      ViBe();
      ~ViBe();

      void process(const cv::Mat &img_input, cv::Mat &img_output, cv::Mat &img_bgmodel);
      void finish();

    private:
      void init(const cv::Mat &img_input, cv::Mat &img_outfg, cv::Mat &img_outbg);
      void workerLoop();
      void stopWorker();
      void throwWorkerErrorLocked() const;
      void collectResult(uint64_t frameNumber, cv::Mat &img_output, const cv::Size &size);
    };
  }
}
