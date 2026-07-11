#include "ViBe.h"
#include "../utils/GenericMacros.h"

using namespace bgslibrary::algorithms;

ViBe::ViBe() :
  matchingThreshold(DEFAULT_MATCH_THRESH),
  matchingNumber(DEFAULT_MATCH_NUM),
  updateFactor(DEFAULT_UPDATE_FACTOR),
  firstTime(true),
  model(nullptr)
{
  debug_construction(ViBe);
  model = vibe::libvibeModel_Sequential_New();
}

ViBe::~ViBe() {
  debug_destruction(ViBe);
  vibe::libvibeModel_Sequential_Free(model);
}

void ViBe::init(const cv::Mat &img_input, cv::Mat &img_outfg, cv::Mat &img_outbg)
{
  assert(img_input.empty() == false);

  // Only (re)allocate when size changes.
  if (m_lastSize != img_input.size()) {
    m_outputFg = cv::Mat::zeros(img_input.size(), CV_8UC1);
    m_outputBg = cv::Mat::zeros(img_input.size(), CV_8UC3);
    m_lastSize = img_input.size();
  }

  img_outfg = m_outputFg;
  img_outbg = m_outputBg;
}

void ViBe::process(const cv::Mat &img_input, cv::Mat &img_output, cv::Mat &img_bgmodel)
{
  init(img_input, img_output, img_bgmodel);

  if (img_input.empty())
    return;

  if (firstTime) {
    vibe::libvibeModel_Sequential_AllocInit_8u_C3R(model, img_input.data, img_input.cols, img_input.rows);
  }

  vibe::libvibeModel_Sequential_Segmentation_8u_C3R(model, img_input.data, img_output.data);
  vibe::libvibeModel_Sequential_Update_8u_C3R(model, img_input.data, img_output.data);

  firstTime = false;
}
