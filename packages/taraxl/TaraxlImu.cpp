#include "TaraxlImu.hpp"
#include <algorithm>
#include <utility>
#include <vector>
#include <iostream>

#include "engine/core/assert.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/system/cuda_context.hpp"
#include "messages/camera.hpp"
#include "engine/gems/image/processing.hpp"



using namespace TaraXLSDK;

namespace isaac {
void TaraXLIMU::start() {
  // sanity-check parameters

  TARAXL_STATUS_CODE status;

  status = taraxlCameras.enumerateDevices(taraxlCamList);
  if (status != TARAXL_SUCCESS) {

      reportFailure("Camera enumeration failed. Error code : %d\n", status);
      return;
  }

  if (taraxlCamList.size() == 0) {

      reportFailure("No TaraXL cameras connected.");
      return;
  }

  //Updates the device ID from the ISAAC_PARAM option that can be set from sight.
  selectedCam = taraxlCamList.at(get_device_id());

  status = selectedCam.connect();
  if (status != TARAXL_SUCCESS) {

      reportFailure("Camera connect failed. Error code : %d\n", status);
      return;
  }

  taraxlPoseTracking = std::make_unique<TaraXLPoseTracking> (selectedCam);

  taraxlPoseTracking->setIMUOutputFrequency(get_imu_frequency());
  tickPeriodically();
}

void TaraXLIMU::tick() {

    TARAXL_STATUS_CODE status;

    taraxlPoseTracking->getIMUData(imuData);
    const int64_t acqtime = node()->clock()->timestamp();

    auto imu = tx_imu_data().initProto();


    imu.setLinearAccelerationX(imuData.linearAcceleration[0]);
    imu.setLinearAccelerationY(imuData.linearAcceleration[1]);
    imu.setLinearAccelerationZ(imuData.linearAcceleration[2]);

    imu.setAngularVelocityX(imuData.angularVelocity[0]);
    imu.setAngularVelocityY(imuData.angularVelocity[1]);
    imu.setAngularVelocityZ(imuData.angularVelocity[2]);

    show("LinearAcceleration.x", imu.getLinearAccelerationX());
    show("LinearAcceleration.y", imu.getLinearAccelerationY());
    show("LinearAcceleration.z", imu.getLinearAccelerationZ());

    show("AngularVelocity.x", imu.getAngularVelocityX());
    show("AngularVelocity.y", imu.getAngularVelocityY());
    show("AngularVelocity.z", imu.getAngularVelocityZ());

    //Optional angles yaw, pitch and roll not provided.

    tx_imu_data().publish(acqtime);
}

void TaraXLIMU::stop() {
    selectedCam.disconnect();
    if(taraxlPoseTracking != nullptr)
          taraxlPoseTracking.reset(nullptr);
}
}  // namespace isaac
