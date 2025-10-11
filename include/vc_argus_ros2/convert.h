//
//
// Copyright 2025 University of Washington

#pragma once

#include <cuda.h>

// Declaration for cuda function found in convert.cu

extern uint8_t* oBuffer;

extern float convertSurfObject(CUsurfObject surface1, CUsurfObject surface2,
                               unsigned int width, unsigned int height,
                               uint8_t* oBuffer);
