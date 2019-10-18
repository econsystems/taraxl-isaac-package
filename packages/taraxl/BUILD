"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_cc_module")


isaac_cc_module(
    name = "taraxl",
    srcs = ["TaraxlCamera.cpp","TaraxlImu.cpp"],
    hdrs = ["TaraxlCamera.hpp","TaraxlImu.hpp"],
    visibility = ["//visibility:public"],
    deps = [
          "//engine/gems/system:cuda_context",
          "//third_party:cuda",
          "//third_party:taraxl",
    ],
)
