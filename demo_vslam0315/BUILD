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
        "//cyber/demo_vslam/proto:vslam_cc_proto",
        "@local_config_pcl//:pcl",
        "@eigen",
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
        "//cyber/demo_vslam/proto:vslam_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        ],
)

cc_binary(
    name = "channel_pointCloud_writer",
    srcs = ["channel_pointCloud_writer.cc"],
    deps = [
        "//cyber",
        ":vslam_fileProcess_lib",
        "//cyber/demo_vslam/proto:vslam_cc_proto",
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
        "//cyber/demo_vslam/proto:vslam_cc_proto",
        ":vlsam_mapping_lib",
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
    runtime_dest = "cyber/demo_vslam",
    targets = [
        ":channel_pointCloud_writer",
        ":channel_pose_writer",
        ":libvslam_component.so",
    ],
)


cpplint()