#pragma once
#pragma warning(disable : 4482)

#include <memory>

#include <opencv2/opencv.hpp>

#include "algorithms/ViBe.h"
#include "utils/GenericMacros.h"

namespace bgslibrary
{
  class FrameProcessor
  {
  private:
    bool firstTime;
    long frameNumber;
    std::string processname;
    double duration;
    std::string tictoc;

    cv::Mat img_vibe;
    std::shared_ptr<algorithms::ViBe> vibe;

    cv::VideoWriter udpWriter;
    bool udpWriterInitialized = false;
    double outputFps = 30.0;
    const std::string streamHost = "192.168.7.3";
    const int streamPort = 5000;
    bool enableUdpStreaming = true;

  public:
    FrameProcessor();
    ~FrameProcessor();

    void setOutputFps(double fps);
    void process(const cv::Mat &img_input);
    void finish(void);

  private:
    void initUdpWriter(const cv::Size &frameSize);
    cv::Mat buildStreamFrame(const cv::Mat &img_input, const cv::Mat &img_output);
    void tic(std::string value);
    void toc();
  };
}
