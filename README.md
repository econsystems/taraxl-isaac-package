# TaraXL SDK Package for Isaac SDK

TaraXL is a CUDA accelarated SDK for TaraXL set of cameras.
## Installation
1. Download the BSP package from the following link.

   https://www.dropbox.com/sh/8fdb9wshksnuw0f/AACYV0MdNdpd3tahOfykpbTka?dl=0

2. Copy the release package into the home directory of the flashed JetsonTM
development kit.

3. Run the following commands to extract the release package in the JetsonTM
development kit to obtain the binaries.

   tar -xampf e-CAM20_Stereo_CUMI2311_TX2_JETSON_<L4T_Version>_<release_date>_<release_version>.tar.gz
   cd e-CAM20_Stereo_CUMI2311_TX2_JETSON_<L4T_Version>_<release_date>_<release_version>

4. Run the following commands to install the binaries.

   sudo chmod a+x ./install_binaries_<version>.sh
   sudo ./install_binaries_<version>.sh
   The above script will reboot the Jetson development kit automatically afterinstalling the binaries successfully.

5. Download ISAAC SDK Version 2019.3 from the following website:

   https://developer.nvidia.com/isaac/downloads

6. Extract the release package in a local folder.

7. git clone https://github.com/econsystems/taraxl-isaac-package.git

8. Run the install.sh script to deploy taraxl related packages


        cd taraxl-isaac-package
        ./install.sh
        Enter the absolute path of Isaac SDK( Eg : /home/nvidia/isaac): <ISAAC Directory>

9. Go to your isaac directory and run the below commands to  run the samples:


        cd <ISAAC Directory> 
   For STEEReoCAM Camera(MIPI):
   
   
           ./engine/build/deploy.sh --remote_user nvidia -p //apps/samples/taraxl_camera:taraxl_camera-pkg -d jetpack43 -h <ROBOT IP>
           ./engine/build/deploy.sh --remote_user nvidia -p //apps/samples/taraxl_imu:taraxl_imu-pkg -d jetpack43 -h <ROBOT IP>

Now you can launch the application in the device/robot.
