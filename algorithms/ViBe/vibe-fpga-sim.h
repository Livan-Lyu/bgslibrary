#pragma once

#include <stddef.h>
#include <stdint.h>

namespace bgslibrary
{
  namespace algorithms
  {
    namespace vibe
    {
      // =========================================================================
      // Hardware-fixed parameters (FPGA RTL constants — no software validation)
      // =========================================================================
      constexpr uint32_t kHistoryFramesHw = 23u;       // 24 entries = 1 current + 23 history
      constexpr size_t   kDdrBytesPerPixel = 96u;       // 24 entries × 4 bytes (RGBX)
      constexpr uint32_t kHardwareSadThreshold = 45u;   // SAD ≤ 45 is a match
      constexpr uint32_t kHardwareMatchingNumber = 2u;  // match_count >= 2 → background
      constexpr uint32_t kPixelsPerBatch = 32u;         // FPGA packs 32 results per RESULT read

      // =========================================================================
      // CAPE base address (mmap target for all pixel_proc registers)
      // =========================================================================
      constexpr uint32_t kPixelProcPhysBase = 0x41280000u;  // CAPE APB slot

      // =========================================================================
      // Register offsets (absolute, relative to CAPE base 0x41100000).
      // Word index = offset / 4.  mmap at CAPE_BASE → REG(offset) = base[offset/4].
      // =========================================================================
      enum PixelProcReg : uint32_t
      {
        REG_CONTROL     = 0x80,  // RW: [0]=START, [7]=ACK
        REG_STATUS      = 0x84,  // RO: [0]=BUSY, [1]=READY, [2]=DONE
        REG_SRC_ADDR_LO = 0x88,  // RW: DDR physical address low 32 bits
        REG_SRC_ADDR_HI = 0x8C,  // RW: DDR physical address high 32 bits
        REG_PIXEL_COUNT = 0x90,  // RW: total pixel count N
        REG_RESULT      = 0x94,  // RO: 32 packed foreground bits ([0]=first pixel)
        REG_DEBUG       = 0x98,  // RO: 0xDEADBEEF
      };

      // =========================================================================
      // CONTROL register bits
      // =========================================================================
      constexpr uint32_t kControlStart = 0x01;
      constexpr uint32_t kControlAck   = 0x80;

      // =========================================================================
      // STATUS register bits
      // =========================================================================
      constexpr uint32_t kStatusBusy  = 0x01;
      constexpr uint32_t kStatusReady = 0x02;
      constexpr uint32_t kStatusDone  = 0x04;

      // =========================================================================
      // DDR buffer layout (pixel-interleaved, 96 bytes/pixel, 24 entries)
      // Entry 0 = current frame, entries 1–23 = history samples.
      // Byte order: [R(7:0), G(15:8), B(23:16), X(31:24)]
      // =========================================================================
      struct DdrPixelLayout
      {
        static constexpr uint32_t kEntriesPerPixel = 24;  // 1 current + 23 history
        static constexpr uint32_t kBytesPerEntry = 4;     // RGBX

        static size_t pixelBlockOffset(uint32_t pixelIndex)
        {
          return static_cast<size_t>(pixelIndex) * kDdrBytesPerPixel;
        }

        static size_t currentOffset(uint32_t pixelIndex)
        {
          return pixelBlockOffset(pixelIndex);  // entry 0
        }

        static size_t historyOffset(uint32_t pixelIndex, uint32_t sampleIndex)
        {
          // Entries: 0=current, 1..23=history
          return pixelBlockOffset(pixelIndex) + (1u + sampleIndex) * kBytesPerEntry;
        }

        static size_t totalBytes(uint32_t totalPixels)
        {
          return static_cast<size_t>(totalPixels) * kDdrBytesPerPixel;
        }
      };

      // =========================================================================
      // Software register file for simulation (mirrors pixel_proc registers).
      // The sim path uses this struct; the MMIO path uses mmap'd hardware regs.
      // =========================================================================
      struct PixelProcSimRegs
      {
        uint32_t control;     // [0]=START, [7]=ACK
        uint32_t status;      // [0]=BUSY, [1]=READY, [2]=DONE
        uint32_t pixelCount;  // total pixel count
        uint32_t result;      // 32 packed foreground bits
      };

      // =========================================================================
      // Step-by-step FPGA pixel_proc simulation API
      //
      // Usage (called from the unified batch loop in Segmentation):
      //
      //   simPixelProcInit(&simRegs, ddrBuf, N, w, h, segMap, thresh, matchNum);
      //   simPixelProcTick(&simRegs);  // process first batch → READY
      //
      //   while (未完成) {
      //       ... CPU reads simRegs.result, updates history, writes ACK ...
      //       simPixelProcClearReady(&simRegs);   // clear READY after ACK
      //       simPixelProcTick(&simRegs);         // process next batch
      //   }
      //
      // After loop: simRegs.status == kStatusDone.
      // =========================================================================

      // Initialize simulation state for a new frame. Sets BUSY.
      void simPixelProcInit(
        PixelProcSimRegs *regs,
        const uint8_t    *ddrBuffer,
        uint32_t          totalPixels,
        uint32_t          width,
        uint32_t          height,
        uint8_t          *segmentationMap,
        uint32_t          sadThreshold,
        uint32_t          matchingNumber
      );

      // Process the next batch of up to 32 pixels.
      // Writes result to regs->result, sets regs->status = BUSY|READY.
      // Sets regs->status = DONE when all pixels have been processed.
      // No-op if already DONE or READY hasn't been cleared yet.
      void simPixelProcTick(PixelProcSimRegs *regs);

      // Clear the READY flag after CPU has consumed the result and written ACK.
      // This allows simPixelProcTick to process the next batch.
      void simPixelProcClearReady(PixelProcSimRegs *regs);

      // Legacy entry point — runs the full simulation synchronously (all batches
      // in one call, no step-by-step interleaving).  Still updates simRegs at
      // each batch boundary so the caller can inspect intermediate state.
      // Returns 0 on success.
      int32_t libvibeModel_Sequential_SimulatePixelProc_8u_C3R(
        const uint8_t    *ddrBuffer,
        uint32_t          totalPixels,
        uint32_t          width,
        uint32_t          height,
        uint8_t          *segmentation_map,
        uint32_t          sadThreshold,
        uint32_t          matchingNumber,
        PixelProcSimRegs *simRegs
      );
    }
  }
}
