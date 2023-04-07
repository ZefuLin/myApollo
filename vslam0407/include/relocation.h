#ifndef RELOCATION_H_
#define RELOCATION_H_

#include <fstream>
#include <ctime> 
#include <algorithm>
#include <iostream>
#include <ctime>
#include <cmath>
#include <string>
#include <iomanip>
#include <vector>
#include <iterator>
#include <unistd.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/random_sample.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <yaml-cpp/yaml.h>
#include "fileProcess.h"
#include "mapping.h"

void relocation(double pose[][7]);
void broadcastCurrentPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn);

#endif