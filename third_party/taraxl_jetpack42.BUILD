
# TaraXL headers and libs based on v3.1.1

exports_files(["LICENSE.TXT"])


cc_library(
    name = "taraxl_aarch64_jetpack42",
    srcs = glob(["taraxl_package/lib/*.so*"]),

    hdrs = glob(["taraxl_package/include/*.h*"])+
           glob(["taraxl_package/include/opencv2/*.h*"])+
           glob(["taraxl_package/include/opencv2/core/**/*.h*"])+
           glob(["taraxl_package/include/opencv2/highgui/*.h*"])+
           glob(["taraxl_package/include/opencv2/imgcodecs/*.h*"])+
           glob(["taraxl_package/include/opencv2/videoio/*.h*"])+
           glob(["taraxl_package/include/opencv2/imgproc/**/*.h*"]),
    includes = ["taraxl_package/include"],


    visibility = ["//visibility:public"],
    deps = [
        "@com_nvidia_isaac//third_party:cuda",
        "@openni_dev_aarch64"
    ],
)
