load("//engine/build:isaac.bzl", "isaac_cc_module")


isaac_cc_module(
    name = "taraxl",
    srcs = ["TaraxlCamera.cpp","TaraxlImu.cpp"],
    hdrs = ["TaraxlCamera.hpp","TaraxlImu.hpp"],
    visibility = ["//visibility:public"],
    linkopts = ["-std=c++14"],
    copts = ["-std=c++14"],
    deps = [
          "//engine/gems/system:cuda_context",
	  "//third_party:taraxl",         
    ],
)
