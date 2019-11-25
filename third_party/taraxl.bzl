
load("//engine/build:isaac.bzl", "isaac_new_http_archive")
load("//engine/build:isaac.bzl", "isaac_new_local_repository")

def clean_dep(dep):
    return str(Label(dep))

# loads dependencies for taraxl
def isaac_taraxl_workspace():


    isaac_new_http_archive(
        name = "taraxl_aarch64_jetpack42",
        build_file = clean_dep("//third_party:taraxl_jetpack42.BUILD"),
        sha256 = "660e6c0a745f2dec754d1f72a8247367d058bbe53ca8d4d6fe1b41a44acf021e",
        url = "https://www.dropbox.com/s/bo5vnm0okpvqxx0/taraxl_package.tar.xz?dl=0",
        type = "tar.xz",
        licenses = [],
    )
    isaac_new_http_archive(
        name = "opencv_dev_aarch64",
        build_file = clean_dep("//third_party:opencv_dev.BUILD"),
        sha256 = "a5d7ec6669c5906020cd5eca0084a9dc2c910b83b97f8959d01617c3b5cb006f",
        url = "https://www.dropbox.com/s/5vtijglghh5e8f6/opencv_dev.tar.xz?dl=0",
        type = "tar.xz",
        licenses = [],
    )
    isaac_new_http_archive(
        name = "pcl_dev_aarch64",
        build_file = clean_dep("//third_party:pcl_dev.BUILD"),
        sha256 = "3516f7f467113cd7e574e329ef3f777b108390b714deb44713f1fa8b286775c2",
        url = "https://www.dropbox.com/s/oyqtc3dl8fbvfzs/pcl_dev.tar.xz?dl=0",
        type = "tar.xz",
        licenses = [],
    )
    isaac_new_http_archive(
        name = "tbb_dev_aarch64",
        build_file = clean_dep("//third_party:tbb_dev.BUILD"),
        sha256 = "ec1422f6653723313cc585de254440259f2e92c26bdc9465432726325a483ce6",
        url = "https://www.dropbox.com/s/uuqeril2i0vixte/tbb_dev.tar.xz?dl=0",
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
    
