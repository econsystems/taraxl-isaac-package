
load("//engine/build:isaac.bzl", "isaac_new_http_archive")
load("//engine/build:isaac.bzl", "isaac_new_local_repository")

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies for taraxl
def isaac_taraxl_workspace():
    isaac_new_http_archive(
        name = "argus_aarch64_jetpack43",
        build_file = clean_dep("//third_party:argus_jetpack43.BUILD"),
        sha256 = "a75906b8a8747d75938ac9cb4b02333dff4660f27041fb3c3b49f01df4c7f28d",
        url = "https://www.dropbox.com/s/tfld2elm7h5w8bi/argus.tar.xz?dl=1",
        type = "tar.xz",
        licenses = [],
    )
    isaac_new_http_archive(
        name = "taraxl_aarch64_jetpack43",
        build_file = clean_dep("//third_party:taraxl_jetpack43.BUILD"),
        sha256 = "bdc6947853a97b8a200fdac0a69c17ff11aa3d6cbeb8a266d5aa1cf8206de0df",
        url = "https://www.dropbox.com/s/82ko5v990mj7wmo/taraxl_imuPackage.tar.xz?dl=1",
        type = "tar.xz",
        licenses = [],
    )
    isaac_new_http_archive(
        name = "opencv_dev_aarch64",
        build_file = clean_dep("//third_party:opencv_dev.BUILD"),
        sha256 = "a5d7ec6669c5906020cd5eca0084a9dc2c910b83b97f8959d01617c3b5cb006f",
        url = "https://www.dropbox.com/s/5vtijglghh5e8f6/opencv_dev.tar.xz?dl=1",
        type = "tar.xz",
        licenses = [],
    )
    
    
    
