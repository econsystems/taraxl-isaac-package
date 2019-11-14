
load("//engine/build:isaac.bzl", "isaac_new_http_archive")
load("//engine/build:isaac.bzl", "isaac_new_local_repository")

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies for taraxl
def isaac_taraxl_workspace():


    isaac_new_http_archive(
        name = "taraxl_aarch64_jetpack42",
        build_file = clean_dep("//third_party:taraxl_jetpack42.BUILD"),
        sha256 = "465ad1bbe6a7629961de07f33939b008102d8db599dc13e0632b3f894472db08",
        url = "https://www.dropbox.com/s/olerqmaculsm313/taraxl_package.tar.xz?dl=1",
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
