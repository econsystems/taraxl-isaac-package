# TaraXL SDK Package for Isaac SDK

TaraXL is a CUDA accelarated SDK for TaraXL set of cameras.
## Installation
1. Download the BSP package and follow the instructions provided with the BSP package.

2. Download ISAAC SDK from the following website:
   
   https://developer.nvidia.com/isaac/downloads

3. Extract the release package in a local folder.

4. git clone https://github.com/econsystems/taraxl-isaac-package.git

5. Run the install.sh script to deploy taraxl related packages
   # cd taraxl-isaac-package
   #./install.sh
   # Enter the absolute path of Isaac SDK( Eg : /home/nvidia/isaac): <ISAAC Directory>

6. Go to your isaac directory and run the below commands to  run the samples:
   # cd <ISAAC Directory> 
   For STEEReoCAM Camera(MIPI):
   #./engine/build/deploy.sh --remote_user nvidia -p //apps/samples/taraxl_camera:taraxl_camera-pkg -d jetpack42 -h <ROBOT IP>
   #./engine/build/deploy.sh --remote_user nvidia -p //apps/samples/taraxl_imu:taraxl_imu-pkg -d jetpack42 -h <ROBOT IP>
   
   For TaraXL Camera(USB)
   #sudo ./engine/build/deploy.sh --remote_user nvidia -p //apps/samples/taraxl_camera:taraxl_camera-pkg -d jetpack42 -h <ROBOT IP>
   #sudo ./engine/build/deploy.sh --remote_user nvidia -p //apps/samples/taraxl_imu:taraxl_imu-pkg -d jetpack42 -h <ROBOT IP>

Now you can launch the application in the device/robot.

