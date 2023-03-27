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
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "modules/localization/proto/gps.pb.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/random_sample.h"
#include "pcl/visualization/pcl_visualizer.h"
// #include "yaml-cpp/yaml.h"

#include "modules/localization/vslam/proto/vslam.pb.h"
#include "modules/localization/vslam/mapping.h"

void relocation(double pose[7], double pointCloud[3000][6],
             int &seq_src,
             Eigen::Affine3d& inverseMatrix,
             Eigen::Affine3d imu2lidar,
             std::string mapping_result_path
             );

void broadcastCurrentPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn);