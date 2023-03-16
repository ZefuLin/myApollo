load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:cc_so_proto_rules.bzl", "cc_so_proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "vslam_proto",
    srcs = ["vslam.proto"],
)

cc_proto_library(
    name = "vslam_cc_proto",
    deps = [":vslam_proto"]
)


