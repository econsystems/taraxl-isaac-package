#include "TaraxlCamera.hpp"
#include <algorithm>
#include <utility>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <sys/syscall.h>
#include <stdlib.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include "opencv2/cudastereo.hpp"

 #include <sys/time.h>
#include <chrono>

#include "engine/core/assert.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/system/cuda_context.hpp"
#include "messages/camera.hpp"
#include "engine/gems/image/processing.hpp"

using namespace TaraXLSDK;
using namespace std;
using namespace Argus;

#define VIDIOC_CUSTOM_VERIFY_CALIBDATA _IOWR('V', 119, int)

bool g_streamState = true;
bool g_captureState = false;
bool g_burstCapture = false;
bool g_controlState = false;
bool g_executeCommand = false;
uint32_t g_deviceID = 0;

namespace isaac {
namespace {

// Factor by which to scale images down when displayed in Sight
constexpr int kSightReduceSize = 4;


// Gets a view on a TaraXL image
ImageConstView1ub getIssacMat(const cv::Mat& mat) {
    return CreateImageView<uint8_t, 1>(mat.data, mat.rows, mat.cols);
}

}  // namespace
// Helper function to copy camera intrinsics to ColorCameraProto
void TaraXLCameraDevice::SetCameraProtoParameters(const CalibrationParams& in, ::ColorCameraProto::Builder& out,TaraXLFrames frames) {
    // Color space
    out.setColorSpace(ColorCameraProto::ColorSpace::GREYSCALE);

    // Pinhole camera model parameters
    auto pinhole = out.initPinhole();

    //Multiplying by a scale factor to be flesible to all the resolutions.

    double fx = in.cameraMatrix.at<double>(0,0) ;
    double fy = in.cameraMatrix.at<double>(1,1) ;
    double cx = in.cameraMatrix.at<double>(0,2) ;
    double cy = in.cameraMatrix.at<double>(1,2) ;

    ToProto(Vector2d(fx,fy), pinhole.getFocal());
    ToProto(Vector2d(cx,cy), pinhole.getCenter());

    pinhole.setCols(1600); //vishnu - need to update with resolution width.
    pinhole.setRows(1300); //vishnu - need to update with resolution height.

    // Distortion parameters
    auto distortion = out.initDistortion();
    distortion.setModel(DistortionProto::DistortionModel::BROWN);

    Vector5f dist;
    dist[0] = in.distortionMatrix.at<double>(0);
    dist[1] = in.distortionMatrix.at<double>(1);
    dist[2] = in.distortionMatrix.at<double>(2);
    dist[3] = in.distortionMatrix.at<double>(3);
    dist[4] = in.distortionMatrix.at<double>(4);

    ToProto(dist, distortion.getCoefficients());

    if(frames==TARAXL_LEFT)
    {
        show("Intrinsic parameters.Left Camera Parameters.fx", fx);
        show("Intrinsic parameters.Left Camera Parameters.fy", fy);
        show("Intrinsic parameters.Left Camera Parameters.cx", cx);
        show("Intrinsic parameters.Left Camera Parameters.cy", cy);

        show("Intrinsic parameters.Left Distortion Coefficients.v", dist[0]);
        show("Intrinsic parameters.Left Distortion Coefficients.w", dist[1]);
        show("Intrinsic parameters.Left Distortion Coefficients.x", dist[2]);
        show("Intrinsic parameters.Left Distortion Coefficients.y", dist[3]);
        show("Intrinsic parameters.Left Distortion Coefficients.z", dist[4]);
    }
    else
    {
        show("Intrinsic parameters.Right Camera Parameters.fx", fx);
        show("Intrinsic parameters.Right Camera Parameters.fy", fy);
        show("Intrinsic parameters.Right Camera Parameters.cx", cx);
        show("Intrinsic parameters.Right Camera Parameters.cy", cy);


        show("Intrinsic parameters.Right Distortion Coefficients.v", dist[0]);
        show("Intrinsic parameters.Right Distortion Coefficients.w", dist[1]);
        show("Intrinsic parameters.Right Distortion Coefficients.x", dist[2]);
        show("Intrinsic parameters.Right Distortion Coefficients.y", dist[3]);
        show("Intrinsic parameters.Right Distortion Coefficients.z", dist[4]);
    }
}

int TaraXLCameraDevice::readCalibrationParams(int width,int height)
{
    FILE *IntFile = NULL;
    const char* intrinsic_filename;
    int deviceId = get_device_id();
    string deviceNode = "/dev/video";
    deviceNode.append(to_string(deviceId));
    int deviceHandle = -1;
    if ((deviceHandle = open(deviceNode.c_str(), O_RDWR | O_NONBLOCK, 0)) < 0) {
        close(deviceHandle);
    }
    int i = 0,fd;
    fd = deviceHandle;

    TARA_CALIBDATA calibdata;
    unsigned short int yml_size = 0;
    unsigned char *yml_recv = NULL;
    int calibrationStatus = 0;

    memset(&calibdata, 0x0, sizeof(calibdata));
    calibdata.index = 127;
    calibdata.size = 2;

    struct v4l2_ext_controls ecs;
    struct v4l2_ext_control ec;

    memset(&ecs, 0, sizeof(ecs));
    memset(&ec, 0, sizeof(ec));
    ec.id = 10094897;//V4L2_CID_GET_CALIBDATA;
    ec.ptr = &calibdata;
    ec.size = sizeof(calibdata);
    ecs.controls = &ec;
    ecs.count = 1;
    ecs.ctrl_class = 0;

    if(ioctl(fd, VIDIOC_S_EXT_CTRLS, &ecs) < 0)
    {
        return -1;
    }

    yml_size = *((unsigned short int *)calibdata.data);
    yml_recv = (unsigned char *)malloc(sizeof(int) * yml_size);

    if(!yml_recv)
    {
        return -1;
    }

    memset(yml_recv, 0, yml_size);

    for(i = 0; i < (yml_size/512); i++ ) {
        memset(&calibdata, 0x0, sizeof(calibdata));
        calibdata.index = i;
        calibdata.size = 512;

        memset(&ecs, 0, sizeof(ecs));
        memset(&ec, 0, sizeof(ec));
        ec.id = 10094897;//V4L2_CID_GET_CALIBDATA;
        ec.ptr = &calibdata;
        ec.size = sizeof(calibdata);
        ecs.controls = &ec;
        ecs.count = 1;
        ecs.ctrl_class = 0;


        if(ioctl(fd, VIDIOC_S_EXT_CTRLS, &ecs) < 0)
        {
            return -1;
        }

        memcpy(&yml_recv[512 * i], calibdata.data, 512);
    }

    if(yml_size % 512) {
        memset(&calibdata, 0x0, sizeof(calibdata));
        calibdata.index = i;
        calibdata.size = (yml_size % 512);

        memset(&ecs, 0, sizeof(ecs));
        memset(&ec, 0, sizeof(ec));
        ec.id = 10094897;//V4L2_CID_GET_CALIBDATA;
        ec.ptr = &calibdata;
        ec.size = sizeof(calibdata);
        ecs.controls = &ec;
        ecs.count = 1;
        ecs.ctrl_class = 0;
        if(ioctl(fd, VIDIOC_S_EXT_CTRLS, &ecs) < 0)
        {
            return -1;
        }
        memcpy( &yml_recv[512 * i], calibdata.data, (yml_size % 512));
    }


    if(ioctl(fd, VIDIOC_CUSTOM_VERIFY_CALIBDATA, &calibrationStatus) < 0)
    {
        return -1;
    }


    if ( calibrationStatus == -1)
    {
        return -2;
    }

    if ( calibrationStatus == -2)
    {
        return -3;
    }

    string calibPath1;
    calibPath1 = "calibration_";
    calibPath1.append(to_string(get_device_id()));
    calibPath1.append(".yml");

    IntFile = fopen(calibPath1.c_str(),"wb");

    if(IntFile == NULL)
    {
        fclose(IntFile);
        return -4;
    }
    else
    {
        fwrite(yml_recv, 1,yml_size , IntFile);
        intrinsic_filename = calibPath1.c_str();
        fclose(IntFile);
    }


    free(yml_recv);

    //reading intrinsic parameters
    cv::FileStorage fs(intrinsic_filename, CV_STORAGE_READ);
    if(!fs.isOpened())
    {
        return -5;
    }

    fs["M1"] >> leftCameraMatrix;
    fs["D1"] >> leftDistortionMatrix;
    fs["M2"] >> rightCameraMatrix;
    fs["D2"] >> rightDistortionMatrix;
    fs["R"] >> rotationMatrix;
    fs["T"] >> translationMatrix;

    cv::Mat R1, P1, R2, P2, Q;
    cv::Rect roi1, roi2;
    cv::Size img_size;
    cv::Mat map11, map12, map21, map22;

    if(TARAXL_1600 != width &&  TARAXL_1300 != height)
    {
        leftCameraMatrix.at<double>(0,0) = ((double)width/ TARAXL_1600) * leftCameraMatrix.at<double>(0,0);
        leftCameraMatrix.at<double>(0,2) = ((double)width/ TARAXL_1600) *  leftCameraMatrix.at<double>(0,2);
        leftCameraMatrix.at<double>(1,2) = ((double)height/ TARAXL_1300) *leftCameraMatrix.at<double>(1,2);
        leftCameraMatrix.at<double>(1,1) = ((double)height/ TARAXL_1300) * leftCameraMatrix.at<double>(1,1);

	rightCameraMatrix.at<double>(0,0) = ((double)width/ TARAXL_1600) * rightCameraMatrix.at<double>(0,0);        
        rightCameraMatrix.at<double>(0,2) = ((double)width/ TARAXL_1600) * rightCameraMatrix.at<double>(0,2);
        rightCameraMatrix.at<double>(1,2) = ((double)height/TARAXL_1300) * rightCameraMatrix.at<double>(1,2);
        rightCameraMatrix.at<double>(1,1) = ((double)height/TARAXL_1300) * rightCameraMatrix.at<double>(1,1);
    }
    img_size.width = width;
    img_size.height = height;


    stereoRectify( leftCameraMatrix, leftDistortionMatrix, rightCameraMatrix, rightDistortionMatrix, img_size, rotationMatrix, translationMatrix, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, img_size, &roi1, &roi2 );
    initUndistortRectifyMap(leftCameraMatrix, leftDistortionMatrix, R1, P1, img_size, CV_32FC1, map11, map12);
    initUndistortRectifyMap(rightCameraMatrix, rightDistortionMatrix, R2, P2, img_size, CV_32FC1, map21, map22);

    leftRectifiedCameraMatrix = P1.colRange(0,3);
    rightRectifiedCameraMatrix = P2.colRange(0,3);

    cv::cuda::GpuMat m11(map11);
    cv::cuda::GpuMat m21(map21);
    cv::cuda::GpuMat m12(map12);
    cv::cuda::GpuMat m22(map22);
    m_gpuMap11 = m11.clone();
    m_gpuMap21= m21.clone();
    m_gpuMap12= m12.clone();
    m_gpuMap22= m22.clone();

    fs.release();

    close(fd);
}

void TaraXLCameraDevice::start() {
    
    oldWidth = get_resolution_width();
    oldHeight = get_resolution_height();
    oldAutoExposure = get_auto_exposure();
    oldDenoiseMode = get_denoise_mode();
    oldDenoiseStrength = get_denoise_strength();
    oldEdgeenhance_mode = get_edgeenhance_mode();
    oldEdgeenhance_strength = get_edgeenhance_strength();
    oldExposureMinTime = get_exposure_min_time();
    oldExposureMaxTime = get_exposure_max_time();
    oldSensorMinGain = get_sensor_min_gain();
    oldSensorMaxGain = get_sensor_max_gain();
    cudaConsumer = NULL;
    queryFrame = true;
    runOnce = true;
    readCalibrationParams(oldWidth, oldHeight);
    context = std::thread([this] {this->acquireFrame();});
    context.detach();
    tickPeriodically();
}

void TaraXLCameraDevice::shutdownCamera()
{
            cv::Mat m;
            cpuFrame = m.clone();  //Making CPU Frame empty

            //Stop begins
            for (uint32_t i = 0; i < streamCount; i++) {
                ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession[i]);
                if (!iCaptureSession)
                    printf("Failed to create CaptureSession");

                // Wait until all captures have completed.
                iCaptureSession->cancelRequests();
                iCaptureSession->waitForIdle();
            }

            // Shutdown the CUDA consumer
            cudaConsumer->threadShutdown();  //Vishnu - Need to uncomment

            for (uint32_t i = 0; i < streamCount; i++) {
                cameraProducer[i].reset();
                captureSession[i].reset();
            }


            free(cudaConsumer);
            g_cameraProvider.reset();

}

void TaraXLCameraDevice::acquireFrame()
{

  timeval totalStart, totalEnd;
  float fpsDeltatime, fpsTotaltime,fps;
  unsigned int iFpsFrames = 0;

    if(runOnce)
    {
        startCamera();
        runOnce = false;
    }
    while(queryFrame)
    {
        if(!runOnce) {
        mtx.lock();
        bool ret = cudaConsumer->threadExecute(cpuFrame, gpuFrame);
        if(ret && cpuFrame.empty()) {
                for (uint32_t i = 0; i < streamCount; i++) {
                    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession[i]);
                    if (!iCaptureSession)
                        printf("Failed to create CaptureSession");

                    if (iCaptureSession->repeat(cameraProducer[i].get()->getRequest()) != STATUS_OK)
                        printf("Failed to call Repeat API - 2");
                    controlSettings[i].reset(new ArgusSamples::ControlSettings(camerProducerList[i]));

                }
                mtx.unlock();
            }
        }
        else {
     	    shutdownCamera();
	    startCamera();
            runOnce = false;
            mtx.unlock();
        }
    }
}

void TaraXLCameraDevice::startCamera()
{
            g_cameraProvider.reset(Argus::CameraProvider::create());
            ICameraProvider *iCameraProvider =  interface_cast<ICameraProvider>(g_cameraProvider);
            if (!iCameraProvider)
            {
                printf("Failed to create CameraProvider");
            }

            // Get the selected camera device and sensor mode.
            if (iCameraProvider->getCameraDevices(&g_cameraDevices) != STATUS_OK)
                printf("Failed to get Camera Devices List");

            if (g_cameraDevices.size() == 0)
                printf("There are no camera Device available");

            ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(g_cameraDevices[0]);
            if (!iCameraProperties)
                printf("Failed to create iCameraProperties interface");

            // Enumerate all the sensor modes in the camera device
            if (iCameraProperties->getAllSensorModes(&sensorModes) != STATUS_OK)
                printf("Failed to get sensor modes list");

            if (DEFAULT_SENSOR_MODE >= sensorModes.size())
                printf("Sensor Mode not in Range. Select sensor Mode less than %d",(unsigned)sensorModes.size());

            ISensorMode *iSensorMode = interface_cast<ISensorMode>(sensorModes[DEFAULT_SENSOR_MODE]);
            if (!iSensorMode)
                printf("Failed to get ISensorMode interface");

            // Get the proper Stream count for streaming the cameras.
            streamCount = std::min((unsigned int)g_cameraDevices.size(), MAX_CAMERA_NUM);
            if (DEFAULT_MIN_CAMERAS <= streamCount)
                streamCount = DEFAULT_MIN_CAMERAS;

            camerProducerList.clear();
            // Create the CUDA Bayer consumer and connect it to the YUV420 output stream.
            for (uint32_t i = 0; i < streamCount; i++) {
                // Create the capture session using the selected device.
                captureSession[i].reset(iCameraProvider->createCaptureSession(g_cameraDevices[i]));

                // Create Argus Camera Producer and handle all Producer Details here.
                cameraProducer[i].reset(new ArgusSamples::CameraModulesEGL(captureSession[i].get(), NULL));
                if (!cameraProducer[i].get()->initialize(g_cameraDevices[i], sensorModes[DEFAULT_SENSOR_MODE], get_resolution_width()*2, get_resolution_height()))
                    printf("Failed to initialize camera Stream");

                // Add OutputStreams of cameras to StreamList
                camerProducerList.push_back(cameraProducer[i].get());
            }
            cudaConsumer = new ArgusSamples::CudaDOLDenoiseConsumer(camerProducerList, NULL);

}
void TaraXLCameraDevice::tick() {

if(oldWidth != get_resolution_width() || oldHeight != get_resolution_height())
{
  oldWidth = get_resolution_width();
  oldHeight = get_resolution_height();

switch(oldWidth)
{
case TARAXL_1600:
   if(oldHeight==TARAXL_1300) {
   readCalibrationParams(oldWidth,oldHeight);
   runOnce = true;
   } 
   else
	cout<<"This resolution is not supported"<<endl;
   break;

case TARAXL_768:
   if(oldHeight==TARAXL_576) {
   readCalibrationParams(oldWidth,oldHeight);
   runOnce = true;
   }
   else
	cout<<"This resolution is not supported"<<endl;
 
   break;

case TARAXL_640:
   if(oldHeight==TARAXL_480) {
   readCalibrationParams(oldWidth,oldHeight);
   runOnce = true;
   } 
   else
	cout<<"This resolution is not supported"<<endl;

   break;

case TARAXL_320:
   if(oldHeight==TARAXL_240) {
   readCalibrationParams(oldWidth,oldHeight);
   runOnce = true;
   } 
   else
	cout<<"This resolution is not supported"<<endl;

   break;
default:
cout<<"This resolution is not supported"<<endl;
break;
}
return void();
}
    if(oldAutoExposure != get_auto_exposure())
    {
        bool ret =  controlSettings[get_device_id()].get()->GetSetControls(2, (get_auto_exposure() ? "0" :"1"));
        oldAutoExposure = get_auto_exposure();
    }
    if(oldDenoiseMode != get_denoise_mode())
    {
        bool ret = false;
        switch(get_denoise_mode())
        {
        case 0:
            ret =  controlSettings[get_device_id()].get()->GetSetControls(4, "off");
            break;
        case 1:
            ret =  controlSettings[get_device_id()].get()->GetSetControls(4, "fast");
            break;
        case 2:
            ret =  controlSettings[get_device_id()].get()->GetSetControls(4, "high");
            break;
        default:
            std::cout <<"Out of Range" << std::endl;
            break;
        }
        if(ret)
            oldDenoiseMode = get_denoise_mode();
    }
    if(oldDenoiseStrength != get_denoise_strength())
    {
        bool ret = false;
        if(get_denoise_strength() >= -1.0 && get_denoise_strength() <= 1.0)
            ret =  controlSettings[get_device_id()].get()->GetSetControls(5, std::to_string(get_denoise_strength()));
        else
            std::cout <<"Out of Range" << std::endl;
        oldDenoiseStrength = get_denoise_strength();
    }

    if(oldEdgeenhance_mode != get_edgeenhance_mode())
    {
        bool ret = false;
        switch(get_edgeenhance_mode())
        {
        case 0:
            ret =  controlSettings[get_device_id()].get()->GetSetControls(6, "off");
            break;
        case 1:
            ret =  controlSettings[get_device_id()].get()->GetSetControls(6, "fast");
            break;
        case 2:
            ret =  controlSettings[get_device_id()].get()->GetSetControls(6, "high");
            break;
        default:
            std::cout <<"Out of Range" << std::endl;
            break;
        }
        oldEdgeenhance_mode = get_edgeenhance_mode();
    }
    if(oldEdgeenhance_strength != get_edgeenhance_strength())
    {
        bool ret = false;
        if(get_edgeenhance_strength() >= -1.0 && get_edgeenhance_strength() <= 1.0)
            ret =  controlSettings[get_device_id()].get()->GetSetControls(7, std::to_string(get_edgeenhance_strength()));
        else
            std::cout <<"Out of Range" << std::endl;
        oldEdgeenhance_strength = get_edgeenhance_strength();
    }        
    if((oldExposureMinTime != get_exposure_min_time()) || (oldExposureMaxTime  != get_exposure_max_time()))
    {
	bool ret = false;
        if(get_exposure_min_time() >= 10000 && get_exposure_max_time() <= 66666000) {
	    std::string min = std::to_string(get_exposure_min_time());
	    std::string max = std::to_string(get_exposure_max_time());
	    min.append(",");
	    min.append(max);
            ret =  controlSettings[get_device_id()].get()->GetSetControls(10, min);
	}
        else
            std::cout <<"Out of Range" << std::endl;
        oldExposureMinTime = get_exposure_min_time();
	oldExposureMaxTime = get_exposure_max_time();
    }

    if((oldSensorMinGain != get_sensor_min_gain()) || (oldSensorMaxGain  != get_sensor_max_gain()))
    {
	bool ret = false;
        if(get_sensor_min_gain() >= 1.12202 && get_sensor_max_gain() <= 5.0) {
	    std::string min = std::to_string(get_sensor_min_gain());
	    std::string max = std::to_string(get_sensor_max_gain());
	    min.append(",");
	    min.append(max);
            ret =  controlSettings[get_device_id()].get()->GetSetControls(11, min);
	}
        else
            std::cout <<"Out of Range" << std::endl;
        oldSensorMinGain = get_sensor_min_gain();
	oldSensorMaxGain = get_sensor_max_gain();
    }    

    if(cpuFrame.empty())
        return void();
    cv::Mat left,right;

    cv::cuda::GpuMat reLeftFrame, reRightFrame;
    cv::cuda::GpuMat leftFrame = cv::cuda::GpuMat(gpuFrame, cv::Rect( 0, 0, gpuFrame.size().width/2, gpuFrame.size().height ));
    cv::cuda::GpuMat rightFrame = cv::cuda::GpuMat(gpuFrame, cv::Rect(gpuFrame.size().width/2, 0, gpuFrame.size().width/2, gpuFrame.size().height ));

    if(!get_publish_raw()) {
        cv::cuda::remap(leftFrame, reLeftFrame, m_gpuMap11, m_gpuMap12, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
        cv::cuda::remap(rightFrame, reRightFrame, m_gpuMap21, m_gpuMap22, cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);
        reLeftFrame.download(left);
        reRightFrame.download(right);
    } else {
        leftFrame.download(left);
        rightFrame.download(right);
    }

    if(!left.isContinuous())
        left = left.clone();
    if(!right.isContinuous())
        right = right.clone();

    publish(left,right);
    mtx.unlock();
}

void TaraXLCameraDevice::stop() {
   queryFrame=false;
   shutdownCamera();
}

Pose3d TaraXLCameraDevice::getCameraExtrinsics(cv::Mat rotation,cv::Mat translation) {

    Pose3d out;

    Matrix3<double> rotationMatrix(reinterpret_cast<double*>(rotation.data));

    out.rotation = SO3<double>::FromQuaternion(Quaterniond(rotationMatrix));
    out.translation = Vector3d(translation.at<double>(0,0), translation.at<double>(1,0), translation.at<double>(2,0));

    show("Extrinsic parameters.Rotation.w", out.rotation.quaternion().w());
    show("Extrinsic parameters.Rotation.x", out.rotation.quaternion().x());
    show("Extrinsic parameters.Rotation.y", out.rotation.quaternion().y());
    show("Extrinsic parameters.Rotation.z", out.rotation.quaternion().z());


    show("Extrinsic parameters.Translation.x", out.translation.x());
    show("Extrinsic parameters.Translation.y", out.translation.y());
    show("Extrinsic parameters.Translation.z", out.translation.z());


    return out;
}

void TaraXLCameraDevice::publish(cv::Mat left, cv::Mat right) {

    auto l_camera = tx_leftImage().initProto();
    auto r_camera = tx_rightImage().initProto();

    CalibrationParams leftIntrinsics, rightIntrinsics, leftRectifiedIntrinsics, rightRectifiedIntrinsics;

    if(get_publish_raw())
    {
        leftIntrinsics.cameraMatrix = leftCameraMatrix;
        leftIntrinsics.distortionMatrix = leftDistortionMatrix;
        rightIntrinsics.cameraMatrix = rightCameraMatrix;
        rightIntrinsics.distortionMatrix = rightDistortionMatrix;
        SetCameraProtoParameters(leftIntrinsics, l_camera,TARAXL_LEFT);
        SetCameraProtoParameters(rightIntrinsics, r_camera,TARAXL_RIGHT);
    }
    else
    {
        leftRectifiedIntrinsics.cameraMatrix = leftRectifiedCameraMatrix;
        leftRectifiedIntrinsics.distortionMatrix = cv::Mat::zeros(1, 14, CV_64F);
        rightRectifiedIntrinsics.cameraMatrix = rightRectifiedCameraMatrix;
        rightRectifiedIntrinsics.distortionMatrix = cv::Mat::zeros(1, 14, CV_64F);;
        SetCameraProtoParameters(leftRectifiedIntrinsics, l_camera,TARAXL_LEFT);
        SetCameraProtoParameters(rightRectifiedIntrinsics, r_camera,TARAXL_RIGHT);
    }

    //Left image
    Image1ub buffer_left_gray(left.rows, left.cols);
    Copy(getIssacMat (left), buffer_left_gray);


    show("left_image_thumbnail",[&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_left_gray)); });
    ToProto(std::move(buffer_left_gray), l_camera.getImage(), tx_leftImage().buffers());

    //Right image
    Image1ub buffer_right_gray(left.rows, left.cols);
    Copy(getIssacMat (right), buffer_right_gray);

    show("right_image_thumbnail",[&](sight::Sop& sop) { sop.add(Reduce<kSightReduceSize>(buffer_right_gray)); });
    ToProto(std::move(buffer_right_gray), r_camera.getImage(), tx_rightImage().buffers());

    const int64_t acqtime = node()->clock()->timestamp();
    tx_leftImage().publish(acqtime);
    tx_rightImage().publish(acqtime);

    // Camera extrinsic parameters
    auto ext = tx_extrinsics().initProto();
    ToProto(getCameraExtrinsics(rotationMatrix,translationMatrix), ext);
    tx_extrinsics().publish(acqtime);

}

}  // namespace isaac

