/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/image/image.hpp"
#include "messages/messages.hpp"
#include "engine/gems/geometry/pinhole.hpp"

#include "TaraXL.h"
#include "TaraXLCam.h"
#include "TaraXLPoseTracking.h"
#include "TaraXLEnums.h"


namespace TaraXLSDK{
  NLOHMANN_JSON_SERIALIZE_ENUM(TARAXL_IMU_OUTPUT_FREQUENCY, {
      {IMU_12_5_HZ, "13Hz"},
      {IMU_26_HZ, "26Hz"},
      {IMU_52_HZ, "52Hz"},
      {IMU_104_HZ, "104Hz"},
      {IMU_208_HZ, "208Hz"},
      {IMU_416_HZ, "416Hz"},
      {IMU_833_HZ, "833Hz"},
      {IMU_208_HZ, nullptr},
  });
}


namespace isaac {
// Provides IMU data from a TaraXL camera
class TaraXLIMU : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // IMU data provided from the TaraXL device.
  ISAAC_PROTO_TX(ImuProto, imu_data);

  // Set the frequency in which IMU wants to tick.
  ISAAC_PARAM(TaraXLSDK::TARAXL_IMU_OUTPUT_FREQUENCY, imu_frequency, TaraXLSDK::IMU_208_HZ);

  // The numeral of the system video device of the TaraXL camera. For example for /dev/video0 choose 0.
  ISAAC_PARAM(int, device_id, 0);


 private:

  // TaraXL camera data
  TaraXLSDK::TaraXL taraxlCameras;
  TaraXLSDK::TaraXLCam selectedCam;

  TaraXLSDK::TaraXLCamList taraxlCamList;
  TaraXLSDK::TaraXLPoseTracking *taraxlPoseTracking;
  struct TaraXLSDK::TaraXLIMUData imuData;
};
}  // namespace isaac
ISAAC_ALICE_REGISTER_CODELET(isaac::TaraXLIMU);
