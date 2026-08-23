#include <iomanip>
#include <sstream>

#include "FrameProcessor.h"

namespace bgslibrary
{
  FrameProcessor::FrameProcessor() :
    firstTime(true), frameNumber(0),
    lastOutputTick(0),
    pipelineFps(0.0),
    fpgaLatencyMs(0.0),
    pipelineStatus("Warm-up 0/2")
  {
    debug_construction(FrameProcessor);
  }

  FrameProcessor::~FrameProcessor() {
    debug_destruction(FrameProcessor);
  }

  void FrameProcessor::setOutputFps(double fps)
  {
    if (fps > 0.0)
      outputFps = fps;
  }

  void FrameProcessor::setShowPipelineStats(bool show)
  {
    showPipelineStats = show;
  }

  void FrameProcessor::initUdpWriter(const cv::Size &frameSize)
  {
    if (udpWriterInitialized || frameSize.width <= 0 || frameSize.height <= 0)
      return;

    std::ostringstream pipeline;
    pipeline
      << "appsrc is-live=true block=true format=time do-timestamp=true ! "
      << "videoconvert ! "
      << "video/x-raw,format=I420 ! "
      << "x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 key-int-max=30 bframes=0 byte-stream=true ! "
      << "rtph264pay config-interval=1 pt=96 ! "
      << "udpsink host=" << streamHost << " port=" << streamPort << " sync=false async=false";

    udpWriter.open(pipeline.str(), cv::CAP_GSTREAMER, 0, outputFps, frameSize, true);

    if (!udpWriter.isOpened()) {
      std::cerr << "Failed to initialize UDP streamer (GStreamer)." << std::endl;
      return;
    }

    std::cout << "UDP stream started: udp://" << streamHost << ":" << streamPort
              << " @ " << outputFps << " FPS, size=" << frameSize.width << "x" << frameSize.height
              << std::endl;
    udpWriterInitialized = true;
  }

  cv::Mat FrameProcessor::buildStreamFrame(const cv::Mat &img_input, const cv::Mat &img_output)
  {
    cv::Mat outBgr;
    if (img_output.empty()) {
      outBgr = cv::Mat::zeros(img_input.size(), CV_8UC3);
    } else if (img_output.channels() == 1) {
      cv::cvtColor(img_output, outBgr, cv::COLOR_GRAY2BGR);
    } else {
      img_output.copyTo(outBgr);
    }

    if (outBgr.size() != img_input.size())
      cv::resize(outBgr, outBgr, img_input.size());

    cv::Mat canvas;
    img_input.copyTo(canvas);

    const int insetWidth = std::max(1, img_input.cols / 3);
    const int insetHeight = std::max(1, img_input.rows / 3);
    cv::Mat inset;
    cv::resize(outBgr, inset, cv::Size(insetWidth, insetHeight));

    const int x = img_input.cols - insetWidth - 10;
    const int y = 10;
    inset.copyTo(canvas(cv::Rect(x, y, insetWidth, insetHeight)));
    cv::rectangle(canvas, cv::Rect(x, y, insetWidth, insetHeight), cv::Scalar(0, 255, 0), 2);

    cv::putText(canvas, "Input", cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    cv::putText(canvas, "FG", cv::Point(x + 8, y + 24), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
    if (showPipelineStats)
      cv::putText(canvas, pipelineStatus, cv::Point(10, 52), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(0, 255, 255), 2);

    return canvas;
  }

  void FrameProcessor::process(const cv::Mat &img_input)
  {
    frameNumber++;

    if (firstTime)
      vibe = std::make_shared<algorithms::ViBe>();

    cv::Mat img_bgmodel;
    vibe->process(img_input, img_vibe, img_bgmodel);
    const int64 processEndTick = cv::getTickCount();
    fpgaLatencyMs = vibe->getFpgaLatencyMs();
    updatePipelineStats(processEndTick);

    if (enableUdpStreaming && !img_input.empty())
    {
      cv::Mat streamFrame = buildStreamFrame(img_input, img_vibe);
      initUdpWriter(streamFrame.size());
      if (udpWriterInitialized)
        udpWriter.write(streamFrame);
    }

    firstTime = false;
  }

  void FrameProcessor::finish(void)
  {
    if (vibe)
      vibe->finish();

    if (udpWriter.isOpened())
      udpWriter.release();
    udpWriterInitialized = false;
  }

  void FrameProcessor::updatePipelineStats(int64 processEndTick)
  {
    if (frameNumber <= static_cast<long>(algorithms::vibe::kFpgaBufferCount)) {
      std::ostringstream status;
      status << "Warm-up " << frameNumber << "/" << algorithms::vibe::kFpgaBufferCount
              << " | FPGA " << std::fixed << std::setprecision(2) << fpgaLatencyMs << " ms";
      pipelineStatus = status.str();
      if (showPipelineStats)
        std::cout << pipelineStatus << std::endl;
      return;
    }

    if (lastOutputTick != 0) {
      const double tickDelta = static_cast<double>(processEndTick - lastOutputTick);
      if (tickDelta > 0.0)
        pipelineFps = cv::getTickFrequency() / tickDelta;
    }
    lastOutputTick = processEndTick;

    std::ostringstream status;
    status << "Pipe FPS: " << std::fixed << std::setprecision(2) << pipelineFps
          << " | FPGA " << std::fixed << std::setprecision(2) << fpgaLatencyMs << " ms";
    pipelineStatus = status.str();
    if (showPipelineStats)
      std::cout << pipelineStatus << std::endl;
  }

}
