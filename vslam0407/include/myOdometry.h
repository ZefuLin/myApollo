#ifndef MYODOMETRY_H_
#define MYODOMETRY_H_

#include <cmath>
#include "Eigen/Dense"

void calWheelOdom(double velocity[][4], double pose[][6], int fileNum);

#endif