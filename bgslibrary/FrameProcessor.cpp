#include <iomanip>

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

    firstTime = false;
  }

  void FrameProcessor::finish(void){}

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
