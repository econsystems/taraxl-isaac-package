/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "TaraxlCamera.hpp"
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


#define TARAXL "See3CAM_StereoA"
using namespace TaraXLSDK;
#define DEFAULT_RESOLUTION 0
namespace isaac {
namespace {

// Factor by which to scale images down when displayed in Sight
constexpr int kSightReduceSize = 1;

// Helper function to copy camera intrinsics to ColorCameraProto
void SetCameraProtoParameters(const CalibrationParams& in, ::ColorCameraProto::Builder& out) {
  // Color space
  out.setColorSpace(ColorCameraProto::ColorSpace::GREYSCALE);

  // Pinhole camera model parameters
  auto pinhole = out.initPinhole();
  ToProto(Vector2d(in.cameraMatrix.at<float>(0,0), in.cameraMatrix.at<float>(1,1)), pinhole.getFocal());
  ToProto(Vector2d(in.cameraMatrix.at<float>(0,2), in.cameraMatrix.at<float>(1,2)), pinhole.getCenter());

  // Distortion parameters
  auto distortion = out.initDistortion();
  distortion.setModel(DistortionProto::DistortionModel::BROWN);
  // We have all zero distortion coefficients because we are retrieving rectified images
  ToProto(Vector5f::Zero(), distortion.getCoefficients());
}

// Gets a view on a TaraXL image
ImageConstView1ub getIssacMat(const cv::Mat& mat) {
  return CreateImageView<uint8_t, 1>(mat.ptr(), mat.rows, mat.cols);
}

}  // namespace

void TaraXLCameraDevice::setResolutionCaller(TaraXLNativeResolutions nativeResolution)
{
    TARAXL_STATUS_CODE status;
    std::string cameraName;
    status = selectedCam.getFriendlyName(cameraName);
    if (status != TARAXL_SUCCESS) {

        reportFailure("Getting camera name failed. Error code : %d\n", status);
        return;
    }
    status = selectedCam.getResolutionList(supportedResolutions);
    if (status != TARAXL_SUCCESS) {

        reportFailure("Getting camera resolutions failed. Error code : %d\n", status);
        return;
    }

    if(cameraName == TARAXL)
    {
        switch (nativeResolution) {
          case TARAXL_752_480 :
              selectedResolution = supportedResolutions.at(DEFAULT_RESOLUTION);
              break;
          case TARAXL_640_480 :
              selectedResolution = supportedResolutions.at(TARAXL_640_480);
              break;
          case TARAXL_320_240 :
              selectedResolution = supportedResolutions.at(TARAXL_320_240);
              break;
          default:
              selectedResolution = supportedResolutions.at(DEFAULT_RESOLUTION);
              nativeResolution = TARAXL_752_480;
              break;
        }
    }
    else
    {
          selectedResolution = supportedResolutions.at(DEFAULT_RESOLUTION);
          nativeResolution = TARAXL_1600_1300;
    }

    status = selectedCam.setResolution(selectedResolution);
    if (status != TARAXL_SUCCESS) {

        reportFailure("Setting camera resolutions failed. Error code : %d\n", status);
        return;
    }

    //Setting resolution in Sight to update native resolution for different cameras.
    set_native_resolution(nativeResolution);
}
void TaraXLCameraDevice::start() {
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

  std::string cameraName;
  status = selectedCam.getFriendlyName(cameraName);
  if (status != TARAXL_SUCCESS) {

      reportFailure("Getting camera name failed. Error code : %d\n", status);
      return;
  }


  selected_native_resolution = get_native_resolution();
  selected_downscaled_resolution = get_downscaled_resolution();

  getDownscaledWidthHeight(selected_downscaled_resolution,downscaledCols,downscaledRows);
  setResolutionCaller(selected_native_resolution);

  tickPeriodically();
}

void TaraXLCameraDevice::tick() {

    cv::Mat left,right;
    TARAXL_STATUS_CODE status;
    std::string str;
    selectedCam.getFriendlyName(str);
    status = selectedCam.grabFrame(left,right);

    if (status != TARAXL_SUCCESS) {

        reportFailure("grabFrame failed. Error code : %d\n", status);
        return;
    }

    if(selected_native_resolution != get_native_resolution())
    {
        selected_native_resolution = get_native_resolution();
        setResolutionCaller(selected_native_resolution);
    }
    publish(left,right);
}

void TaraXLCameraDevice::stop() {
    selectedCam.disconnect();


}
Pose3d TaraXLCameraDevice::getCameraExtrinsics(cv::Mat rotation,cv::Mat translation) {
  Pose3d out;
  double Q[4];
  Matrix3<double> rotationMatrix(reinterpret_cast<double*>(rotation.data));

  out.rotation = SO3<double>::FromQuaternion(Quaterniond(rotationMatrix));
  out.translation = Vector3d(translation.at<double>(0,0), translation.at<double>(1,0), translation.at<double>(2,0));

  show("Extrinsic parameters Quaternion.w", out.rotation.quaternion().w());
  show("Extrinsic parameters Quaternion.x", out.rotation.quaternion().x());
  show("Extrinsic parameters Quaternion.y", out.rotation.quaternion().y());
  show("Extrinsic parameters Quaternion.z", out.rotation.quaternion().z());


  show("Extrinsic parameters Translation.x", out.translation.x());
  show("Extrinsic parameters Translation.y", out.translation.y());
  show("Extrinsic parameters Translation.z", out.translation.z());


  return out;
}
void TaraXLCameraDevice::getDownscaledWidthHeight(TaraXLDownscaledResolutions downscaledResolution,int &downscaledCols,int &downscaledRows)
{

    if(downscaledResolution == TARAXL_DOWNSCALE_NAN)
    {
        downscaledCols = 0;
        downscaledRows = 0;
        return;
    }
    std::string cameraName;
    selectedCam.getFriendlyName(cameraName);
    if(cameraName == TARAXL)
    {

      if(selected_native_resolution != TARAXL_752_480)
      {
        reportFailure("Error in downscaling resolution. Please update to highest native resoluition to downscale.\n");
        return;
      }
      printf("down : %d\n",downscaledResolution);
      switch (downscaledResolution) {
        case TARAXL_DOWNSCALE_1600_1200 :
        case TARAXL_DOWNSCALE_1440_1080 :
        case TARAXL_DOWNSCALE_1400_1050 :
        case TARAXL_DOWNSCALE_1280_960 :
        case TARAXL_DOWNSCALE_1200_900 :
        case TARAXL_DOWNSCALE_1152_864 :
        case TARAXL_DOWNSCALE_1024_768 :
        case TARAXL_DOWNSCALE_960_720 :
        case TARAXL_DOWNSCALE_800_600 :
        case TARAXL_DOWNSCALE_768_576 :
            reportFailure("Downscale resolution not supported for the selected camera. For TaraXL cameras, use resolutions 640x480 or below.\n");
            return;
        case TARAXL_DOWNSCALE_640_480 :
            downscaledCols = 640;
            downscaledRows = 480;
            break;
        case TARAXL_DOWNSCALE_480_360 :
            downscaledCols = 480;
            downscaledRows = 360;
            break;
        case TARAXL_DOWNSCALE_320_240:
            downscaledCols = 320;
            downscaledRows = 240;
            break;
        case TARAXL_DOWNSCALE_192_144:
            downscaledCols = 192;
            downscaledRows = 144;
            break;
        case TARAXL_DOWNSCALE_160_120:
            downscaledCols = 160;
            downscaledRows = 144;
            break;
        case TARAXL_DOWNSCALE_NAN :
            downscaledCols = 0;
            downscaledRows = 0;
            break;
        default :
            downscaledCols = 640;
            downscaledRows = 480;
            break;
      }
    }
    else
    {
      if(selected_native_resolution != TARAXL_1600_1300)
      {
        reportFailure("Error in downscaling resolution. Please update to highest native resoluition to downscale.\n");
        return;
      }
      switch (downscaledResolution) {
        case TARAXL_DOWNSCALE_1600_1200 :
            downscaledCols = 1600;
            downscaledRows = 1200;
            break;
        case TARAXL_DOWNSCALE_1440_1080 :
            downscaledCols = 1440;
            downscaledRows = 1080;
            break;
        case TARAXL_DOWNSCALE_1400_1050 :
            downscaledCols = 1400;
            downscaledRows = 1050;
            break;
        case TARAXL_DOWNSCALE_1280_960 :
            downscaledCols = 1280;
            downscaledRows = 960;
            break;
        case TARAXL_DOWNSCALE_1200_900 :
            downscaledCols = 1200;
            downscaledRows = 900;
            break;
        case TARAXL_DOWNSCALE_1152_864 :
            downscaledCols = 1152;
            downscaledRows = 864;
            break;
        case TARAXL_DOWNSCALE_1024_768 :
            downscaledCols = 1024;
            downscaledRows = 768;
            break;
        case TARAXL_DOWNSCALE_960_720 :
            downscaledCols = 960;
            downscaledRows = 720;
            break;
        case TARAXL_DOWNSCALE_800_600 :
            downscaledCols = 800;
            downscaledRows = 600;
            break;
        case TARAXL_DOWNSCALE_768_576 :
            downscaledCols = 768;
            downscaledRows = 576;
            break;
        case TARAXL_DOWNSCALE_640_480 :
            downscaledCols = 640;
            downscaledRows = 480;
            break;
        case TARAXL_DOWNSCALE_480_360 :
            downscaledCols = 480;
            downscaledRows = 360;
            break;
        case TARAXL_DOWNSCALE_320_240:
            downscaledCols = 320;
            downscaledRows = 240;
            break;
        case TARAXL_DOWNSCALE_192_144:
            downscaledCols = 192;
            downscaledRows = 144;
            break;
        case TARAXL_DOWNSCALE_160_120:
            downscaledCols = 160;
            downscaledRows = 144;
            break;
        case TARAXL_DOWNSCALE_NAN :
            downscaledCols = 0;
            downscaledRows = 0;
            break;
        default :
            downscaledCols = 640;
            downscaledRows = 480;
            break;
      }
    }
}
void TaraXLCameraDevice::publish(cv::Mat left, cv::Mat right) {


    auto l_camera = tx_leftImage().initProto();
    auto r_camera = tx_rightImage().initProto();

    cv::Mat R,T;
    CalibrationParams leftIntrinsics, rightIntrinsics;
    TARAXL_STATUS_CODE status;


    status = selectedCam.getCalibrationParameters(R,T,leftIntrinsics,rightIntrinsics);


    SetCameraProtoParameters(leftIntrinsics, l_camera);
    SetCameraProtoParameters(rightIntrinsics, r_camera);


    if(selected_downscaled_resolution != get_downscaled_resolution())
    {
        selected_downscaled_resolution = get_downscaled_resolution();
        getDownscaledWidthHeight(selected_downscaled_resolution,downscaledCols,downscaledRows);
    }
    //Left image
    Image1ub buffer_left_gray(left.rows, left.cols);
    Copy(getIssacMat (left), buffer_left_gray);
    if(left.rows > downscaledRows && left.cols > downscaledCols && downscaledCols != 0 && downscaledRows != 0 )
    {
        Image1ub buffer_left_gray_scaled;
        Downsample(buffer_left_gray, Vector2i{downscaledRows, downscaledCols}, buffer_left_gray_scaled);
        show("left_image_thumbnail",[&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_left_gray_scaled)); });
        ToProto(std::move(buffer_left_gray_scaled), l_camera.getImage(), tx_leftImage().buffers());
    }
    else
    {
        show("left_image_thumbnail",[&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_left_gray)); });
        ToProto(std::move(buffer_left_gray), l_camera.getImage(), tx_leftImage().buffers());
    }

    //Right image
    Image1ub buffer_right_gray(left.rows, left.cols);
    Copy(getIssacMat (right), buffer_right_gray);
    if(left.rows > downscaledRows && left.cols > downscaledCols && downscaledCols != 0 && downscaledRows != 0)
    {
        Image1ub buffer_right_gray_scaled;
        Downsample(buffer_right_gray, Vector2i{downscaledRows, downscaledCols}, buffer_right_gray_scaled);
        show("right_image_thumbnail",[&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_right_gray_scaled)); });
        ToProto(std::move(buffer_right_gray_scaled), r_camera.getImage(), tx_rightImage().buffers());
    }
    else
    {
        show("right_image_thumbnail",[&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_right_gray)); });
        ToProto(std::move(buffer_right_gray), r_camera.getImage(), tx_rightImage().buffers());
    }

    const int64_t acqtime = node()->clock()->timestamp();
    tx_leftImage().publish(acqtime);
    tx_rightImage().publish(acqtime);

    // Camera extrinsic parameters
    auto ext = tx_extrinsics().initProto();
    ToProto(getCameraExtrinsics(R,T), ext);
    tx_extrinsics().publish(acqtime);


}

}  // namespace isaac
