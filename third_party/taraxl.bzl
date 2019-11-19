
load("//engine/build:isaac.bzl", "isaac_new_http_archive")
load("//engine/build:isaac.bzl", "isaac_new_local_repository")

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies for taraxl
def isaac_taraxl_workspace():


    isaac_new_http_archive(
        name = "taraxl_aarch64_jetpack42",
        build_file = clean_dep("//third_party:taraxl_jetpack42.BUILD"),
        sha256 = "c04938b86019a7e690c62ab80636d2dc9b21dfdf8f5d0ee7831d0f374250d996",
        url = "https://www.dropbox.com/s/id734hmyvz2eqy0/taraxl_package.tar.xz?dl=0",
        type = "tar.xz",
        licenses = [],
    )
    isaac_new_http_archive(
        name = "opencv_dev_aarch64",
        build_file = clean_dep("//third_party:opencv_dev.BUILD"),
        sha256 = "a315d53f2855f0458f498534c3901bd7a6550d81390061594785f579e425c65e",
        url = "https://www.dropbox.com/s/5dkb2k20cjedf85/opencv_dev.tar.xz?dl=0",
        type = "tar.xz",
        licenses = [],
    )
    isaac_new_http_archive(
        name = "pcl_dev_aarch64",
        build_file = clean_dep("//third_party:pcl_dev.BUILD"),
        sha256 = "471fe7f8f6bc06424f33bd2abda6d632d9fb6ea4af23ea653ec99a48d9e80d7f",
        url = "https://www.dropbox.com/s/ct9lh49fof6lpzq/pcl_dev.tar.xz?dl=0",
        type = "tar.xz",
        licenses = [],
    )
    isaac_new_http_archive(
        name = "openni_dev_aarch64",
        build_file = clean_dep("//third_party:openni_dev.BUILD"),
        sha256 = "e1ae942f98366cebbb1f80d131adf1e977f127fc9c7cdd61f01ec39fa1072ae2",
        url = "https://www.dropbox.com/s/wbmcqf9l03tay78/openni_dev.tar.xz?dl=1",
        type = "tar.xz",
        licenses = [],

    )
    
