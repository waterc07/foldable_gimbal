#pragma once

#include "gsrl_common.h"

namespace GimbalCalibrationStorage
{
bool loadYawGyroOffset(fp32 &offsetY);
bool saveYawGyroOffset(fp32 offsetY);
} // namespace GimbalCalibrationStorage
