#include "ViBe.h"
#include "../utils/GenericMacros.h"

#include <cstring>
#include <stdexcept>

using namespace bgslibrary::algorithms;

ViBe::ViBe() :
  matchingThreshold(DEFAULT_MATCH_THRESH),
  matchingNumber(DEFAULT_MATCH_NUM),
  updateFactor(DEFAULT_UPDATE_FACTOR),
  firstTime(true),
  model(nullptr),
  nextFrameNumber(0u),
  initialized(false),
  stopRequested(false),
  workerFailed(false)
{
  debug_construction(ViBe);
  model = vibe::libvibeModel_Sequential_New();
  slotStates.fill(SlotState::Free);
}

ViBe::~ViBe() {
  finish();
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
  if (img_input.empty())
    return;

  if (img_input.type() != CV_8UC3)
    throw std::invalid_argument("ViBe expects a continuous CV_8UC3 frame");

  if (initialized && img_input.size() != m_lastSize)
    throw std::invalid_argument("ViBe frame size cannot change after initialization");

  const cv::Mat input = img_input.isContinuous() ? img_input : img_input.clone();
  init(input, img_output, img_bgmodel);

  if (!initialized) {
    if (vibe::libvibeModel_Sequential_AllocInit_8u_C3R(
          model, input.data, input.cols, input.rows) != 0) {
      throw std::runtime_error("failed to initialize FPGA ViBe double buffer");
    }

    initialized = true;
    worker = std::thread(&ViBe::workerLoop, this);
  }

  const uint64_t frameNumber = nextFrameNumber++;
  const uint32_t slotIndex =
    static_cast<uint32_t>(frameNumber % vibe::kFpgaBufferCount);

  // The first two frames have no N-2 result yet. Both slots were initialized
  // from frame 0, so they are valid warm-up models.
  if (frameNumber >= vibe::kFpgaBufferCount)
    collectResult(frameNumber - vibe::kFpgaBufferCount, img_output, input.size());
  else
    img_output = cv::Mat::zeros(input.size(), CV_8UC1);

  {
    std::unique_lock<std::mutex> lock(pipelineMutex);
    resultAvailable.wait(lock, [this, slotIndex] {
      return slotStates[slotIndex] == SlotState::Free ||
        workerFailed || stopRequested;
    });
    throwWorkerErrorLocked();
    if (stopRequested)
      throw std::runtime_error("ViBe pipeline has been stopped");
    slotStates[slotIndex] = SlotState::Preparing;
  }

  // This copy can overlap the worker polling the other slot.
  if (vibe::libvibeModel_Sequential_PrepareSlot_8u_C3R(
        model, slotIndex, input.data) != 0) {
    std::lock_guard<std::mutex> lock(pipelineMutex);
    slotStates[slotIndex] = SlotState::Free;
    resultAvailable.notify_all();
    throw std::runtime_error("failed to prepare FPGA ViBe slot");
  }

  {
    std::lock_guard<std::mutex> lock(pipelineMutex);
    slotStates[slotIndex] = SlotState::Ready;
    workQueue.push_back({frameNumber, slotIndex});
  }
  workAvailable.notify_one();

  firstTime = false;
}

void ViBe::finish()
{
  stopWorker();
}

void ViBe::workerLoop()
{
  for (;;) {
    WorkItem item;
    {
      std::unique_lock<std::mutex> lock(pipelineMutex);
      workAvailable.wait(lock, [this] {
        return stopRequested || !workQueue.empty();
      });

      if (workQueue.empty()) {
        if (stopRequested)
          return;
        continue;
      }

      item = workQueue.front();
      workQueue.pop_front();
      slotStates[item.slotIndex] = SlotState::Running;
    }

    std::vector<uint8_t> output(m_lastSize.area(), vibe::COLOR_BACKGROUND);
    const int32_t status =
      vibe::libvibeModel_Sequential_SegmentSlot_8u_C3R(
        model, item.slotIndex, output.data());

    {
      std::lock_guard<std::mutex> lock(pipelineMutex);
      if (status != 0) {
        workerFailed = true;
        workerError = "FPGA ViBe slot execution failed";
        slotStates[item.slotIndex] = SlotState::Free;
        workQueue.clear();
        for (SlotState &state : slotStates) {
          if (state == SlotState::Ready)
            state = SlotState::Free;
        }
        stopRequested = true;
      } else {
        slotStates[item.slotIndex] = SlotState::Complete;
        completedFrames.emplace(item.frameNumber, std::move(output));
      }
    }
    resultAvailable.notify_all();
    workAvailable.notify_all();

    if (status != 0)
      return;
  }
}

void ViBe::stopWorker()
{
  if (!worker.joinable())
    return;

  {
    std::lock_guard<std::mutex> lock(pipelineMutex);
    stopRequested = true;
  }
  workAvailable.notify_all();
  resultAvailable.notify_all();
  worker.join();
}

void ViBe::throwWorkerErrorLocked() const
{
  if (workerFailed)
    throw std::runtime_error(workerError.empty() ?
      "FPGA ViBe worker failed" : workerError);
}

void ViBe::collectResult(
  const uint64_t frameNumber,
  cv::Mat &img_output,
  const cv::Size &size)
{
  std::unique_lock<std::mutex> lock(pipelineMutex);
  resultAvailable.wait(lock, [this, frameNumber] {
    return completedFrames.find(frameNumber) != completedFrames.end() ||
      workerFailed || stopRequested;
  });
  throwWorkerErrorLocked();
  if (stopRequested && completedFrames.find(frameNumber) == completedFrames.end())
    throw std::runtime_error("ViBe pipeline stopped before result collection");

  const std::vector<uint8_t> &result = completedFrames.at(frameNumber);
  img_output = cv::Mat(size, CV_8UC1);
  std::memcpy(img_output.data, result.data(), result.size());
  completedFrames.erase(frameNumber);
  slotStates[frameNumber % vibe::kFpgaBufferCount] = SlotState::Free;
  resultAvailable.notify_all();
}
