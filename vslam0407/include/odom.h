#ifndef ODOM_H
#define ODOM_H

#include <cmath>
#include "fileProcess.h"
#include "ekf.h"
#include <yaml-cpp/yaml.h>

struct velocities
{
    float linear_x;
    float linear_y;
    float angular_z;
};

const double wheel_radius = 0.32985;
const double wheel_distance_x = 2.785;
const double wheel_distance_y = 1.578;
const int total_wheels = 2;

const double imu2lidar_yaw = -1.567455654690774;
const double imu2lidar_tx = -1.19292;
const double imu2lidar_ty = -0.0230692;
const double imu2lidar_tz = -1.28574;
const double lidar2wheel_x = -1.534709;

// Eigen::Affine3d imu2lidar;
// 	imu2lidar(0,0) = imu2lidar_matrix[0];
// 	imu2lidar(0,1) = imu2lidar_matrix[1];
// 	imu2lidar(0,2) = imu2lidar_matrix[2];
// 	imu2lidar(1,0) = imu2lidar_matrix[3];
// 	imu2lidar(1,1) = imu2lidar_matrix[4];
// 	imu2lidar(1,2) = imu2lidar_matrix[5];
// 	imu2lidar(2,0) = imu2lidar_matrix[6];
// 	imu2lidar(2,1) = imu2lidar_matrix[7];
// 	imu2lidar(2,2) = imu2lidar_matrix[8]; 
// 	imu2lidar(0,3) = imu2lidar_matrix[9];
// 	imu2lidar(1,3) = imu2lidar_matrix[10];
// 	imu2lidar(2,3) = imu2lidar_matrix[11];

void imuOdometry(int fileNum, double imu[][9], double imuOdomPose[][7]);
void wheelOdometry(int fileNum, double velocity[][4], double wheelOdomPose[][7]);
void imuAndWheelOdometry(int fileNum,
                         double imu[][9], double velocity[][4], double gnss[][7], double pose[][7]);

void ekf_wheel(double wheel[5]);
void ekf_imu(double imu[9]);
void ekf_gnss(double gnss[7],double gnss_init[7]);
// 输入的都是double的vector，c++不给我多态，这三个函数先用三个不同的名字
// 在同一时刻，哪个传感器有数据进来就用哪个
void ekf_gnss_wheel(double wheel[4], double gnss[7],double gnss_init[7]); 

velocities GetVelocities(float vel_front_left, float vel_front_right, float vel_rear_left, float vel_rear_right);
std::vector<double> EulerToQuaternion(double roll, double pitch, double yaw);
std::vector<double> QuaternionToEuler(double qw, double qx, double qy, double qz);
#endif