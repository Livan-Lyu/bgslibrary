#!/bin/bash
export VIBE_FPGA_TRANSPORT=pdma
./build/bgslibrary -uf -fn=dataset/video.avi
