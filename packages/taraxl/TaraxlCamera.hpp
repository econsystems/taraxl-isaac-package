#pragma once

#include <memory>
#include "engine/alice/alice_codelet.hpp"
#include "engine/core/image/image.hpp"
#include "messages/messages.hpp"
#include "engine/gems/geometry/pinhole.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/types.h>

#include "CUDAHelper.h"
#include <Argus/Argus.h>
#include "CameraModuleEGL.h"
#include <EGLStream/EGLStream.h>
#include <Argus/CameraProvider.h>

#include "CudaDOLDenoiseKernel.h"
#include "CameraModuleEGL.h"
#include "ArgusMetaEvent.h"
#include "CudaDOLDenoiseConsumer.h"
#include "ControlSettings.h"

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cuda_runtime_api.h>
#include <cuda.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/cudawarping.hpp"
#include <iostream>
#include <thread>
#include <mutex>
#include <stdint.h>
#include <sys/ioctl.h> 
#include <linux/videodev2.h>


namespace TaraXLSDK{
  enum TaraXLNativeResolutions{
      TARAXL_1600 = 1600,
      TARAXL_1300 =1300,
      TARAXL_768 = 768,
      TARAXL_576 = 576,
      TARAXL_640 = 640,
      TARAXL_480 = 480,
      TARAXL_320 = 320,
      TARAXL_240 = 240
  };
  enum TaraXLFrames{
    TARAXL_LEFT=1,
    TARAXL_RIGHT=0
  };
 
  enum TaraXLDenoise {
    OFF = 0,
    FAST = 1,
    HIGH = 2
  };

  NLOHMANN_JSON_SERIALIZE_ENUM(TaraXLNativeResolutions, {
      {TARAXL_1600, "1600"},
      {TARAXL_1300, "1300"},
      {TARAXL_768, "768"},
      {TARAXL_576, "576"},
      {TARAXL_640, "640"},
      {TARAXL_480, "480"},
      {TARAXL_320, "320"},
      {TARAXL_240, "240"},
      {TARAXL_1600, nullptr},
  });
}

extern uint32_t	g_deviceID;
extern bool g_streamState;
extern bool g_controlState;
extern bool g_captureState;
extern bool g_burstCapture;
extern bool g_controlState;
extern bool g_executeCommand;

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


  ISAAC_PARAM(int, resolution_width, TaraXLSDK::TARAXL_1600);
  ISAAC_PARAM(int, resolution_height, TaraXLSDK::TARAXL_1300);

  // The numeral of the system video device of the TaraXL camera. For example for /dev/video0 choose 0.
  ISAAC_PARAM(int, device_id, 0);

  //To allow codelet to publish raw/rectified frames.
  ISAAC_PARAM(bool, publish_raw, true);

  //To allow codelet default settings
  ISAAC_PARAM(bool, auto_exposure, true);
  ISAAC_PARAM(int, denoise_mode, TaraXLSDK::FAST);
  ISAAC_PARAM(float, denoise_strength, 1.0);
  ISAAC_PARAM(int, edgeenhance_mode, TaraXLSDK::FAST);
  ISAAC_PARAM(float, edgeenhance_strength, -1.0);
  ISAAC_PARAM(int, exposure_min_time, 10000);
  ISAAC_PARAM(int, exposure_max_time, 66666000);
  ISAAC_PARAM(float, sensor_min_gain, 1.12202);
  ISAAC_PARAM(float, sensor_max_gain, 5.0);


  void acquireFrame();

 private:
  typedef struct _tara_calibdata {
	__u16 index;
	__u16 size;
	char data[512];
  }TARA_CALIBDATA;
  struct CalibrationParams
  {
      cv::Mat cameraMatrix;	
      cv::Mat distortionMatrix;
  };

  // Publish the stereo data (images, camera intrinsics and extrinsics)
  void publish(cv::Mat left, cv::Mat right);

  // Retrieve the camera extrinsics
  Pose3d getCameraExtrinsics(cv::Mat rotation,cv::Mat translation);
  void SetCameraProtoParameters(const CalibrationParams& in, ::ColorCameraProto::Builder& out,TaraXLSDK::TaraXLFrames frames);
  int readCalibrationParams(int width,int height);

  void startCamera();
  void shutdownCamera();


  Argus::UniqueObj<Argus::CameraProvider>  g_cameraProvider;

  std::vector<CameraDevice*> g_cameraDevices;
  std::vector<ArgusSamples::CameraModulesEGL*> camerProducerList;
  std::vector<SensorMode*> sensorModes;

  uint32_t streamCount;
  static const uint32_t MAX_CAMERA_NUM = 6;
  const uint32_t DEFAULT_SENSOR_MODE   = 0;
  const uint32_t DEFAULT_MIN_CAMERAS   = 1;

  UniqueObj<ArgusSamples::CameraModulesEGL> cameraProducer[6];
  UniqueObj<Argus::CaptureSession> captureSession[6];
  UniqueObj<ArgusSamples::ControlSettings> controlSettings[6];


  ArgusSamples::CudaDOLDenoiseConsumer *cudaConsumer;
  bool runOnce, queryFrame;

  cv::Mat cpuFrame;
  cv::cuda::GpuMat gpuFrame;
  cv::Mat rotationMatrix, translationMatrix;
  cv::Mat leftCameraMatrix, leftDistortionMatrix, rightCameraMatrix, rightDistortionMatrix,leftRectifiedCameraMatrix,rightRectifiedCameraMatrix;
  cv::cuda::GpuMat m_gpuMap11, m_gpuMap21, m_gpuMap12, m_gpuMap22;
   std::thread context;
   std::mutex mtx;
 
  int oldWidth, oldHeight, oldAutoExposure;
  int oldDenoiseMode, oldEdgeenhance_mode;
  int oldExposureMinTime, oldExposureMaxTime;
  float oldDenoiseStrength, oldEdgeenhance_strength;
  float oldSensorMinGain, oldSensorMaxGain;
};

}  // namespace isaac

ISAAC_ALICE_REGISTER_CODELET(isaac::TaraXLCameraDevice);
