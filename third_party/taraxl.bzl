"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

load("//engine/build:isaac.bzl", "isaac_new_http_archive")
load("//engine/build:isaac.bzl", "isaac_new_local_repository")

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies for taraxl
def isaac_taraxl_workspace():


    isaac_new_http_archive(
        name = "taraxl_aarch64_jetpack42",
        build_file = clean_dep("//third_party:taraxl_jetpack42.BUILD"),
        sha256 = "de32a3ff38104c337c3df4a09626353e7c6ab92f7e7917c9e0e792a96ca75afa",
        url = "https://www.dropbox.com/s/x7ygd1rdi3iaotz/taraxl_package.tar.xz?dl=1",
        type = "tar.xz",
        licenses = [],
    )
    isaac_new_http_archive(
        name = "openni_dev_aarch64",
        build_file = clean_dep("//third_party:openni_dev.BUILD"),
        sha256 = "abc489a1caed5bb413c02bd93f5b92de4980d38df96445f6f32ab56e9380c18e",
        url = "https://www.dropbox.com/s/iij8050oblesqfv/openni_dev.tar.xz?dl=1",
        type = "tar.xz",
        licenses = [],
    )
