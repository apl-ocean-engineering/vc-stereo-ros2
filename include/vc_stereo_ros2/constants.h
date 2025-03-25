//
//
// Copyright 2025 University of Washington

#pragma once

#include <Argus/Argus.h>

namespace vc_stereo_ros2 {

static const Argus::Range<float> GAIN_RANGE(1, 44);
static const Argus::Range<float> ISP_DIGITAL_GAIN_RANGE(1, 1);
static const Argus::Range<uint64_t> EXPOSURE_TIME_RANGE(44000, 1000000);
static const Argus::Size2D<uint32_t> STREAM_SIZE(1440, 1080);

}  // namespace vc_stereo_ros2
