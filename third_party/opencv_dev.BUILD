# This package contains the aarch64 version of the libopencv library copied from tx2/xavier/nano
# libopencv
cc_library(
    name = "opencv_dev_aarch64",
    srcs = glob(["opencv_dev/lib/*.so*"]),

    hdrs = glob(["opencv_dev/include/opencv2/*.h*"])+
           glob(["opencv_dev/include/opencv2/core/**/*.h*"])+
           glob(["opencv_dev/include/opencv2/highgui/*.h*"])+
           glob(["opencv_dev/include/opencv2/imgcodecs/*.h*"])+
           glob(["opencv_dev/include/opencv2/videoio/*.h*"])+
	   glob(["opencv_dev/include/opencv2/flann/*.h*"])+
           glob(["opencv_dev/include/opencv2/calib3d/*.h*"])+
           glob(["opencv_dev/include/opencv2/imgproc/**/*.h*"]),
    includes = ["opencv_dev/include"],


    visibility = ["//visibility:public"],
   
)
