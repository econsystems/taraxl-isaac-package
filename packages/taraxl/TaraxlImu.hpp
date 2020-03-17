#pragma once

#include <memory>
#include <string>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h> 

#include "engine/alice/alice_codelet.hpp"
#include "engine/core/image/image.hpp"
#include "messages/messages.hpp"
#include "engine/gems/geometry/pinhole.hpp"

extern "C" {
#include "libimu_public.h"
}

namespace TaraXLSDK{

enum TARAXL_IMU_OUTPUT_FREQUENCY
  {
   IMU_119_HZ = (0x03),
   IMU_238_HZ = (0x04),
   IMU_476_HZ = (0x05),
   IMU_952_HZ = (0x06),
   IMU_12_5_HZ = (0x01),
   IMU_26_HZ = (0x02),
   IMU_52_HZ = (0x03),
   IMU_104_HZ = (0x04),
   IMU_208_HZ = (0x05),
   IMU_416_HZ = (0x06),
   IMU_833_HZ = (0x07),
   IMU_1666_HZ = (0x08)
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
 	void *handle = NULL;
	int err;
	struct imuconfig imu_cfg;	
	short int tempval;
	struct imudata_ag data;
 	enum _ov2311_custom_ctrl {
	V4L2_CID_FACEDETECT = (V4L2_CID_AUTO_FOCUS_RANGE+1),
	V4L2_CID_FACEMARK,
	V4L2_CID_SMILEDETECT,
	V4L2_GET_FACEINFO,
	V4L2_CID_ROI_WINDOW,
	V4L2_CID_ROI_FOCUS,
	V4L2_CID_ROI_EXPOSURE,
	V4L2_CID_TRIGGER_FOCUS,
	
	/* New Controls */
	V4L2_CID_HDR,
	V4L2_CID_COLORKILL,
	V4L2_CID_FRAME_SYNC,
	V4L2_CID_CUSTOM_EXPOSURE_AUTO,
	V4L2_CID_CUSTOM_FLASH_STROBE,
	V4L2_CID_DENOISE,
	V4L2_CID_GRAYSCALE,
	V4L2_CID_LSCMODE,
	V4L2_CID_TARGET_BRIGHTNESS,
	V4L2_CID_GET_CALIBDATA,
	V4L2_CID_SET_CALIBDATA,
	V4L2_CID_VERIFY_CALIBDATA,
	V4L2_CID_TARAPRESENT,
	V4L2_CID_TARAPASSWD,
	V4L2_CID_TARAUNIQID,
	V4L2_CID_TARAFWV,	
};

	typedef struct _tara_uniq {
		char	position[32];
	}TARA_UNIQ;
	void getuniqueid(int deviceID, std::string &id);
	void setDataRate();
	void registerIMU();

	int oldIMUFreq;
	int oldDeviceID;
	int deviceHandle;


};
}  // namespace isaac
ISAAC_ALICE_REGISTER_CODELET(isaac::TaraXLIMU);

