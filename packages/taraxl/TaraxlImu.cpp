#include "TaraxlImu.hpp"
#include <algorithm>
#include <utility>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h> 

#include "engine/core/assert.hpp"
#include "engine/gems/image/conversions.hpp"
#include "engine/gems/image/utils.hpp"
#include "engine/gems/system/cuda_context.hpp"
#include "messages/camera.hpp"
#include "engine/gems/image/processing.hpp"

using namespace std;

namespace isaac {
void TaraXLIMU::registerIMU()
{
	std::ifstream deviceIdFileName;
	deviceIdFileName.open("/sys/module/tegra_fuse/parameters/tegra_chip_id");
	string chipId;
	int busNumber = 0;;

	if (deviceIdFileName)
	{
		ostringstream ss;
		ss << deviceIdFileName.rdbuf(); // reading data
		chipId = ss.str();
	}
	if(memcmp(chipId.c_str(),"33", 2) == 0) //For Nano board
		busNumber = 6;
	else {//For TX2 & Xavier		
		std::string camPort;
		int deviceID = get_device_id();
		getuniqueid(deviceID, camPort);//Get device id
		if(camPort == "CAMA")
			busNumber = 30;
		else if(camPort == "CAMB")
			busNumber = 31;
		else if(camPort == "CAMC")
			busNumber = 32;
		else if(camPort == "CAMD")
			busNumber = 33;
		else if(camPort == "CAME")
			busNumber = 34;
		else
			busNumber = 35;
	}
	err = register_imu_dev(&handle, busNumber, 0x6b);
	if(err)
	{
	  printf("Device registration failed...\n");
	  exit(0);
	}

}

void TaraXLIMU::setDataRate()
{
	int imu_freq = 208;

	switch(get_imu_frequency())
	{	
	case 1:
		imu_freq = 13;
		break;
	case 2:
		imu_freq = 26;
		break;
	case 3:
		imu_freq = 52;
		break;
	case 4:
		imu_freq = 104;
		break;
	case 5:
		imu_freq = 208;
		break;
	case 6:
		imu_freq = 416;
		break;
	case 7:
		imu_freq = 833;
		break;
	default: 
		imu_freq = 208;
		break;
	}
	err = imu_setdatarate(handle, imu_freq);
	if(err)
	{
	  unregister_imu_dev(&handle);
	  printf("Device registration failed");
	  exit(0);
	}
}

void TaraXLIMU::getuniqueid(int deviceID, std::string &uniqueId)
{
 	TARA_UNIQ uniq;
	string deviceNode = "/dev/video";
	deviceNode.append(to_string(deviceID));
	int deviceHandle = -1;
    	if ((deviceHandle = open(deviceNode.c_str(), O_RDWR | O_NONBLOCK, 0)) < 0) {
    	  close(deviceHandle);
        }
	struct v4l2_ext_controls ecs;
   	struct v4l2_ext_control ec;

        memset(&ecs, 0, sizeof(ecs));
        memset(&ec, 0, sizeof(ec));
        ec.id = V4L2_CID_TARAUNIQID;
        ec.ptr = &uniq;
        ec.size = sizeof(uniq);
        ecs.controls = &ec;
        ecs.count = 1;
        ecs.ctrl_class = 0;

        if(ioctl(deviceHandle, VIDIOC_S_EXT_CTRLS, &ecs) < 0)
        {
            uniqueId = "fail";
	    return void();
        }
        uniqueId = uniq.position;
	close(deviceHandle);
}

void TaraXLIMU::start() {
	registerIMU();
	setDataRate();

	oldIMUFreq = get_imu_frequency();
	oldDeviceID = get_device_id();

  	tickPeriodically();
}

void TaraXLIMU::tick() {

    if(oldIMUFreq != get_imu_frequency()){
	setDataRate();
	oldIMUFreq = get_imu_frequency();
    }

    if(oldDeviceID != get_device_id()) {	
	unregister_imu_dev(&handle);
	registerIMU();
	setDataRate();

	oldIMUFreq = get_imu_frequency();
	oldDeviceID = get_device_id();
    }
    const int64_t acqtime = node()->clock()->timestamp();

    auto imu = tx_imu_data().initProto();
    err = imu_get_ag_data(handle, &data);

    //Output in m/s2
    imu.setLinearAccelerationX(((short int)data.acc_xyz[0])*0.061);
    imu.setLinearAccelerationY(((short int)data.acc_xyz[1])*0.061);
    imu.setLinearAccelerationZ(((short int)data.acc_xyz[2])*0.061);

    //Output in degree
    imu.setAngularVelocityX(((short int)data.gyr_xyz[0])*0.00875);
    imu.setAngularVelocityY(((short int)data.gyr_xyz[1])*0.00875);
    imu.setAngularVelocityZ(((short int)data.gyr_xyz[2])*0.00875);

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
	std::cout <<"In stop function"<< std::endl;
   err = unregister_imu_dev(&handle);
}
}  // namespace isaac

