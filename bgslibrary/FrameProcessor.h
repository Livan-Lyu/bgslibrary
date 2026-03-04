#pragma once
#pragma warning(disable : 4482)

#include "IFrameProcessor.h"
#include "PreProcessor.h"

#include "algorithms/algorithms.h"
#include "tools/ForegroundMaskAnalysis.h"

namespace bgslibrary
{
  class FrameProcessor : public IFrameProcessor, public ILoadSaveConfig
  {
  private:
    bool firstTime;
    long frameNumber;
    std::string processname;
    double duration;
    std::string tictoc;

    cv::Mat img_preProcessor;
    std::unique_ptr<PreProcessor> preProcessor;
    bool enablePreProcessor = false;

    cv::Mat img_vibe;
    std::shared_ptr<ViBe> vibe;
    bool enableViBe = false;

    std::shared_ptr<tools::ForegroundMaskAnalysis> foregroundMaskAnalysis;
    bool enableForegroundMaskAnalysis = false;

  public:
    FrameProcessor();
    ~FrameProcessor();

    long frameToStop;
    std::string imgref;

    void init();
    void process(const cv::Mat &img_input);
    void finish(void);

  private:
    void process(const std::string name, const std::shared_ptr<IBGS> &bgs, const cv::Mat &img_input, cv::Mat &img_bgs);
    void tic(std::string value);
    void toc();

    void save_config(cv::FileStorage &fs);
    void load_config(cv::FileStorage &fs);
  };
}
