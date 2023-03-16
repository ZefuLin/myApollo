#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <ctime> 
#include <cmath>
#include <iomanip>
#include <algorithm>

#include "cyber/cyber.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/random_sample.h"
// #include "yaml-cpp/yaml.h"

#include "cyber/demo_vslam/proto/vslam.pb.h"


void mapping(double pose[7], double pointCloud[3000][6],
             int &seq_glo,int &seq_src,
             double &lastPoseX,
             double &lastPoseY,
             double &lastPoseZ,
             Eigen::Affine3d& inverseMatrix,
             Eigen::Affine3d imu2lidar,
             int &keyframe,
             std::string global_cloud_path,
             std::string source_cloud_path
             );
pcl::PointCloud<pcl::PointXYZRGB>::Ptr myTransformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, Eigen::Affine3d& transCur);