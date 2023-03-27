load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")


package(default_visibility = ["//visibility:public"])

proto_library(
    name = "vslam_proto",
    srcs = ["vslam.proto"],
)

cc_proto_library(
    name = "vslam_cc_proto",
    deps = [":vslam_proto"]
)

proto_library(
    name = "ground_sign_segmentation_proto",
    srcs = ["ground_sign_segmentation.proto"],
    deps = [
        "//modules/common/proto:header_proto",
    ],
)

cc_proto_library(
    name = "ground_sign_segmentation_cc_proto",
    deps = [
        ":ground_sign_segmentation_proto",
        ]
)


