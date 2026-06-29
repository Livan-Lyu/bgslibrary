#include <cstring>
#include <iostream>

#include "vibe-fpga-sim.h"
#include "vibe-background-sequential.h"

namespace bgslibrary
{
  namespace algorithms
  {
    namespace vibe
    {
      namespace
      {
        // -----------------------------------------------------------------------
        // Internal simulation state (hidden, not exposed in the register file)
        // -----------------------------------------------------------------------
        struct SimState
        {
          const uint8_t *ddrBuffer;
          uint32_t       totalPixels;
          uint32_t       sadThreshold;
          uint32_t       matchingNumber;
          uint8_t       *segmentationMap;
          uint32_t       nextPixelIndex;
        };

        static SimState g_state;
        static uint64_t g_frameCounter = 0u;

        static inline int absInt(int v) { return (v >= 0) ? v : -v; }

        // SAD: |R0-R1| + |G0-G1| + |B0-B1| ≤ threshold
        // Entry layout: byte[0]=R, [1]=G, [2]=B, [3]=X(padding)
        static bool compareEntry(const uint8_t *cur, const uint8_t *hist, uint32_t threshold)
        {
          uint32_t sad =
              static_cast<uint32_t>(absInt(static_cast<int>(cur[0]) - static_cast<int>(hist[0])))   // R
            + static_cast<uint32_t>(absInt(static_cast<int>(cur[1]) - static_cast<int>(hist[1])))   // G
            + static_cast<uint32_t>(absInt(static_cast<int>(cur[2]) - static_cast<int>(hist[2])));  // B
          return sad <= threshold;
        }
      }

      // =========================================================================
      // simPixelProcInit
      // =========================================================================
      void simPixelProcInit(
        PixelProcSimRegs *regs,
        const uint8_t    *ddrBuffer,
        uint32_t          totalPixels,
        uint32_t          width,
        uint32_t          height,
        uint8_t          *segmentationMap,
        uint32_t          sadThreshold,
        uint32_t          matchingNumber
      )
      {
        (void)width;
        (void)height;

        std::memset(regs, 0, sizeof(*regs));
        regs->status     = kStatusBusy;
        regs->pixelCount = totalPixels;

        g_state.ddrBuffer       = ddrBuffer;
        g_state.totalPixels     = totalPixels;
        g_state.sadThreshold    = sadThreshold;
        g_state.matchingNumber  = matchingNumber;
        g_state.segmentationMap = segmentationMap;
        g_state.nextPixelIndex  = 0u;

        std::cerr
          << "[FPGA-SIM] init frame=" << g_frameCounter
          << " totalPixels=" << totalPixels
          << " threshold=" << sadThreshold
          << " matchingNum=" << matchingNumber
          << std::endl;
      }

      // =========================================================================
      // simPixelProcTick — process next batch of up to 32 pixels
      // =========================================================================
      void simPixelProcTick(PixelProcSimRegs *regs)
      {
        // Only process if BUSY, not READY, and not DONE.
        if (!(regs->status & kStatusBusy))
          return;
        if (regs->status & kStatusDone)
          return;
        if (regs->status & kStatusReady)
          return;  // previous batch not yet consumed by CPU

        if (g_state.nextPixelIndex >= g_state.totalPixels) {
          regs->status = kStatusDone;
          std::cerr << "[FPGA-SIM] done frame=" << g_frameCounter << std::endl;
          ++g_frameCounter;
          return;
        }

        uint32_t batchResult = 0u;
        uint32_t batchCount  = 0u;
        const uint32_t batchStart = g_state.nextPixelIndex;

        while (g_state.nextPixelIndex < g_state.totalPixels && batchCount < kPixelsPerBatch) {
          const uint32_t pi = g_state.nextPixelIndex;
          const uint8_t *cur = g_state.ddrBuffer + DdrPixelLayout::currentOffset(pi);

          uint32_t matchCount = 0u;
          for (uint32_t s = 0u; s < kHistoryFramesHw; ++s) {
            const uint8_t *hist = g_state.ddrBuffer + DdrPixelLayout::historyOffset(pi, s);
            if (compareEntry(cur, hist, g_state.sadThreshold)) {
              ++matchCount;
              if (matchCount >= g_state.matchingNumber)
                break;
            }
          }

          const bool isForeground = (matchCount < g_state.matchingNumber);
          if (isForeground)
            batchResult |= (1u << batchCount);

          g_state.segmentationMap[pi] = isForeground ? COLOR_FOREGROUND : COLOR_BACKGROUND;

          ++g_state.nextPixelIndex;
          ++batchCount;
        }

        regs->result = batchResult;
        regs->status = kStatusBusy | kStatusReady;

        std::cerr
          << "[FPGA-SIM] tick frame=" << g_frameCounter
          << " batch_start=" << batchStart
          << " batch_count=" << batchCount
          << " result=0x" << std::hex << batchResult << std::dec
          << std::endl;
      }

      // =========================================================================
      // simPixelProcClearReady — clear READY after CPU ACK
      // =========================================================================
      void simPixelProcClearReady(PixelProcSimRegs *regs)
      {
        // Keep BUSY, clear READY.
        if (regs->status & kStatusReady)
          regs->status = kStatusBusy;
      }

      // =========================================================================
      // Legacy full-frame entry point
      // =========================================================================
      int32_t libvibeModel_Sequential_SimulatePixelProc_8u_C3R(
        const uint8_t    *ddrBuffer,
        uint32_t          totalPixels,
        uint32_t          width,
        uint32_t          height,
        uint8_t          *segmentation_map,
        uint32_t          sadThreshold,
        uint32_t          matchingNumber,
        PixelProcSimRegs *simRegs
      )
      {
        if (ddrBuffer == NULL || segmentation_map == NULL || simRegs == NULL)
          return -1;

        simPixelProcInit(simRegs, ddrBuffer, totalPixels, width, height,
                         segmentation_map, sadThreshold, matchingNumber);

        bool done = false;
        while (!done) {
          simPixelProcTick(simRegs);
          done = (simRegs->status & kStatusDone) != 0u;
        }

        return 0;
      }
    }
  }
}
