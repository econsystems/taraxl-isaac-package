#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/image/image.hpp"
#include "messages/messages.hpp"
#include "engine/gems/geometry/pinhole.hpp"

#include "TaraXL.h"
#include "TaraXLCam.h"
#include "TaraXLEnums.h"


namespace TaraXLSDK{
  enum TaraXLNativeResolutions{
    TARAXL_640_480 = 1,
    TARAXL_320_240 = 2,
    TARAXL_752_480 = 3,
    TARAXL_1600_1300 = 4
  };
  enum TaraXLFrames{
    TARAXL_LEFT=1,
    TARAXL_RIGHT=0
  };
  enum TaraXLDownscaledResolutions{
    TARAXL_DOWNSCALE_1600_1200 = 0,
    TARAXL_DOWNSCALE_1440_1080 = 1,
    TARAXL_DOWNSCALE_1400_1050 = 2,
    TARAXL_DOWNSCALE_1280_960 = 3,
    TARAXL_DOWNSCALE_1200_900 = 4,
    TARAXL_DOWNSCALE_1152_864 = 5,
    TARAXL_DOWNSCALE_1024_768 = 6,
    TARAXL_DOWNSCALE_960_720 = 7,
    TARAXL_DOWNSCALE_800_600 = 8,
    TARAXL_DOWNSCALE_768_576 = 9,
    TARAXL_DOWNSCALE_640_480 = 10,
    TARAXL_DOWNSCALE_480_360 = 11,
    TARAXL_DOWNSCALE_320_240 = 12,
    TARAXL_DOWNSCALE_192_144 = 13,
    TARAXL_DOWNSCALE_160_120 = 14,
    TARAXL_DOWNSCALE_NAN = 15
  };
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
  NLOHMANN_JSON_SERIALIZE_ENUM(TaraXLNativeResolutions, {
      {TARAXL_1600_1300, "1600x1300"},
      {TARAXL_752_480, "752x480"},
      {TARAXL_640_480, "640x480"},
      {TARAXL_320_240, "320x240"},
      {TARAXL_1600_1300, nullptr},
  });
  NLOHMANN_JSON_SERIALIZE_ENUM(TaraXLDownscaledResolutions, {
      {TARAXL_DOWNSCALE_1600_1200, "1600x1200"},
      {TARAXL_DOWNSCALE_1440_1080, "1440x1080"},
      {TARAXL_DOWNSCALE_1400_1050, "1400x1050"},
      {TARAXL_DOWNSCALE_1280_960, "1280x960"},
      {TARAXL_DOWNSCALE_1200_900, "1200x900"},
      {TARAXL_DOWNSCALE_1152_864, "1152x864"},
      {TARAXL_DOWNSCALE_1024_768, "1024x720"},
      {TARAXL_DOWNSCALE_960_720, "960x720"},
      {TARAXL_DOWNSCALE_800_600, "800x600"},
      {TARAXL_DOWNSCALE_768_576, "768x576"},
      {TARAXL_DOWNSCALE_640_480, "640x480"},
      {TARAXL_DOWNSCALE_480_360, "480x360"},
      {TARAXL_DOWNSCALE_320_240, "320x240"},
      {TARAXL_DOWNSCALE_192_144, "192x144"},
      {TARAXL_DOWNSCALE_160_120, "160x120"},
      {TARAXL_DOWNSCALE_NAN, ""},
      {TARAXL_DOWNSCALE_640_480, nullptr}
  });

}


namespace isaac {

// Provides stereo image pairs and calibration information from a TaraXL camera
class TaraXLCameraDevice : public alice::Codelet {
 public:
  void start() override;
  void tick() override;
  void stop() override;

  // left image and camera intrinsics
  ISAAC_PROTO_TX(ColorCameraProto, leftImage);
  // right image and camera intrinsics
  ISAAC_PROTO_TX(ColorCameraProto, rightImage);
  // camera pair extrinsics (right-to-left)
  ISAAC_PROTO_TX(Pose3dProto, extrinsics);



  ISAAC_PARAM(TaraXLSDK::TaraXLNativeResolutions, native_resolution, TaraXLSDK::TARAXL_752_480);

  ISAAC_PARAM(TaraXLSDK::TaraXLDownscaledResolutions, downscaled_resolution, TaraXLSDK::TARAXL_DOWNSCALE_NAN);

  // The numeral of the system video device of the TaraXL camera. For example for /dev/video0 choose 0.
  ISAAC_PARAM(int, device_id, 0);

  //To allow codelet to publish raw/rectified frames.
  ISAAC_PARAM(bool, publish_raw, false);

 private:

  // Publish the stereo data (images, camera intrinsics and extrinsics)
  void publish(cv::Mat left, cv::Mat right);

  // Retrieve the camera extrinsics
  Pose3d getCameraExtrinsics(cv::Mat rotation,cv::Mat translation);

  void setResolutionCaller(TaraXLSDK::TaraXLNativeResolutions);

  void SetCameraProtoParameters(const TaraXLSDK::CalibrationParams& in, ::ColorCameraProto::Builder& out,TaraXLSDK::TaraXLFrames frames);


  void getDownscaledWidthHeight(TaraXLSDK::TaraXLDownscaledResolutions selected_downscaled_resolution,int &downscaledCols,int &downscaledRows);
  // TaraXL camera data
  TaraXLSDK::TaraXL taraxlCameras;
  TaraXLSDK::TaraXLCam selectedCam;
  TaraXLSDK::ResolutionList supportedResolutions;
  TaraXLSDK::Resolution selectedResolution;
  TaraXLSDK::TaraXLCamList taraxlCamList;
  std::string cameraName;
  cv::Mat leftImage;
  cv::Mat rightImage;
  TaraXLSDK::TaraXLNativeResolutions selected_native_resolution;
  TaraXLSDK::TaraXLDownscaledResolutions selected_downscaled_resolution;
  int downscaledRows, downscaledCols;
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::TaraXLCameraDevice);
