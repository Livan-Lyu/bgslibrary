#pragma once

#include <iostream>
#include <fstream>
#include <memory>
#include <opencv2/opencv.hpp>

#include "utils/GenericKeys.h"
#include "utils/GenericMacros.h"
#include "FrameProcessor.h"

namespace bgslibrary
{
  class VideoCapture
  {
  private:
    std::shared_ptr<FrameProcessor> frameProcessor;
    cv::VideoCapture capture;
    cv::Mat frame;
    int key;
    int frameNumber;
    int stopAt;
    bool useCamera;
    int cameraIndex;
    bool useVideo;
    std::string videoFileName;
    int input_resize_percent;
    bool showOutput;
    bool showFPS;
    bool enableFlip;
    double loopDelay = 33.333;
    bool firstTime = true;

  public:
    VideoCapture();
    ~VideoCapture();

    void setFrameProcessor(const std::shared_ptr<FrameProcessor> &_frameProcessor);
    void setCamera(int _index);
    void setVideo(std::string _filename);
    void start();

  private:
    void setUpCamera();
    void setUpVideo();
  };
}
