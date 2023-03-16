#include <fstream>
#include <ctime> 
#include <algorithm>
#include <iostream>
#include <ctime>
#include <cmath>
#include <string>
#include <iomanip>

#include "cyber/cyber.h"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"

#include "yaml-cpp/yaml.h"

using namespace std;

void readDataFromIMUOdometry(string path, int fileNum, double pose[][7]);
void readDataFeomBEVCloud(string path, double pointCloud[][6]);
