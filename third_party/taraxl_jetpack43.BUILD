# TaraXL headers and libs for ISAAC codelet

exports_files(["LICENSE.TXT"])


cc_library(
    name = "taraxl_aarch64_jetpack43",
    srcs = glob(["lib/libimu_i2c.so"]),
    hdrs = glob(["include/libimu_public.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        "@opencv_dev_aarch64",
	"@argus_aarch64_jetpack43",
  ],
)
