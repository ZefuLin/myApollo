load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")
package(default_visibility = ["//visibility:public"])

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library
cc_library(
    name = "vlsam_mapping_lib",
    srcs = ["mapping.cc"],
    hdrs = ["mapping.h"],
    deps = [
        "//cyber",
        "//modules/localization/vslam/proto:vslam_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "@local_config_pcl//:pcl",
        "@eigen",
        # "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library
cc_library(
    name = "vlsam_relocation_lib",
    srcs = ["relocation.cc"],
    hdrs = ["relocation.h"],
    deps = [
        "//cyber",
        "//modules/localization/vslam/proto:vslam_cc_proto",
        "//modules/localization/proto:gps_cc_proto",
        ":vlsam_mapping_lib",
        "@local_config_pcl//:pcl",
        "@eigen",
        "@local_config_vtk//:vtk",
        # "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library
cc_library(
    name = "vslam_fileProcess_lib",
    srcs = ["fileProcess.cc"],
    hdrs = ["fileProcess.h"],
    deps = [
        "//cyber",
        "@local_config_pcl//:pcl",
        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_binary
cc_binary(
    name = "channel_pose_writer",
    srcs = ["channel_pose_writer.cc"],
    deps = [
        "//cyber",
        ":vslam_fileProcess_lib",
        "//modules/localization/vslam/proto:vslam_cc_proto",
        "//modules/localization/proto:gps_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        ],
)

cc_binary(
    name = "channel_pointCloud_writer",
    srcs = ["channel_pointCloud_writer.cc"],
    deps = [
        "//cyber",
        ":vslam_fileProcess_lib",
        "//modules/localization/vslam/proto:vslam_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        ],
)

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library
cc_library(
    name = "vslam_component_lib",
    srcs = ["vslam_component.cc"],
    hdrs = ["vslam_component.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/localization/vslam/proto:vslam_cc_proto",
        "//modules/localization/proto:gps_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        ":vlsam_mapping_lib",
        ":vlsam_relocation_lib",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
        "@eigen",
    ],
)


# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_binary
cc_binary(
    name = "libvslam_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":vslam_component_lib"],
)

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library
cc_library(
    name = "readBEVCloud_lib",
    srcs = ["readBEVCloud_cpt.cc"],
    hdrs = ["readBEVCloud_cpt.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/localization/vslam/proto:ground_sign_segmentation_cc_proto",
        "//modules/drivers/proto:pointcloud_cc_proto",
        ":vlsam_mapping_lib",
        ":vlsam_relocation_lib",
        ":vslam_fileProcess_lib",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@local_config_pcl//:pcl",
        "@eigen",
    ],
)


# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_binary
cc_binary(
    name = "libreadBEVCloud_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":readBEVCloud_lib"],
)

filegroup(
    name = "conf",
    srcs = [
        ":vslam.dag",
        ":vslam.launch",
    ],
)

install(
    name = "install",
    data = [
        ":conf",
    ],
    # We want to install channel_*_writer into runtime_dest
    # rather than bin/
    runtime_dest = "modules/localization/vslam",
    targets = [
        ":channel_pointCloud_writer",
        ":channel_pose_writer",
        ":libvslam_component.so",
    ],
)


cpplint()