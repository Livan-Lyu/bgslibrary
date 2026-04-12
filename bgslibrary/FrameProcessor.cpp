#include <iomanip>
#include <sstream>

#include "FrameProcessor.h"

namespace bgslibrary
{
  FrameProcessor::FrameProcessor() :
    firstTime(true), frameNumber(0), duration(0),
    tictoc(""), frameToStop(0)
  {
    debug_construction(FrameProcessor);
    initLoadSaveConfig(quote(FrameProcessor));
  }

  FrameProcessor::~FrameProcessor() {
    debug_destruction(FrameProcessor);
  }

  void FrameProcessor::init()
  {
    if (enablePreProcessor)
      preProcessor = std::make_unique<PreProcessor>();

    if (enableViBe)
      vibe = std::make_shared<ViBe>();

    if (enableForegroundMaskAnalysis)
      foregroundMaskAnalysis = std::make_shared<tools::ForegroundMaskAnalysis>();
  }

  void FrameProcessor::setOutputFps(double fps)
  {
    if (fps > 0.0)
      outputFps = fps;
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

    return canvas;
  }

  void FrameProcessor::process(const std::string name, const std::shared_ptr<IBGS> &bgs, const cv::Mat &img_input, cv::Mat &img_bgs)
  {
    if (tictoc == name)
      tic(name);

    cv::Mat img_bkgmodel;
    bgs->process(img_input, img_bgs, img_bkgmodel);

    if (tictoc == name)
      toc();
  }

  void FrameProcessor::process(const cv::Mat &img_input)
  {
    frameNumber++;

    if (enablePreProcessor)
      preProcessor->process(img_input, img_preProcessor);
    else
      img_input.copyTo(img_preProcessor);

    if (enableViBe)
      process("ViBe", vibe, img_preProcessor, img_vibe);

    if (enableForegroundMaskAnalysis)
    {
      foregroundMaskAnalysis->stopAt = frameToStop;
      foregroundMaskAnalysis->img_ref_path = imgref;

      foregroundMaskAnalysis->process(frameNumber, "ViBe", img_vibe);
    }

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
    if (udpWriter.isOpened())
      udpWriter.release();
    udpWriterInitialized = false;
  }

  void FrameProcessor::tic(std::string value)
  {
    processname = value;
    duration = static_cast<double>(cv::getTickCount());
  }

  void FrameProcessor::toc()
  {
    duration = (static_cast<double>(cv::getTickCount()) - duration) / cv::getTickFrequency();
    std::cout << processname << "\ttime(sec):" << std::fixed << std::setprecision(6) << duration << std::endl;
  }

  void FrameProcessor::save_config(cv::FileStorage &fs) {
    fs << "tictoc" << tictoc;
    fs << "enablePreProcessor" << enablePreProcessor;
    fs << "enableForegroundMaskAnalysis" << enableForegroundMaskAnalysis;
    fs << "enableViBe" << enableViBe;
  }

  void FrameProcessor::load_config(cv::FileStorage &fs) {
    fs["tictoc"] >> tictoc;
    fs["enablePreProcessor"] >> enablePreProcessor;
    fs["enableForegroundMaskAnalysis"] >> enableForegroundMaskAnalysis;
    fs["enableViBe"] >> enableViBe;
  }
}
