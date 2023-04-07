#include "odom.h"
#include "fileProcess.h"

using namespace std;

YAML::Node config_odom = YAML::LoadFile("../config/params.yaml");
double imu2wheel_ratio = config_odom["imu2wheel_ratio"].as<double>();

/// @brief input : accelerated velocity ax,ay,az,and angular velocity of RPY , RPY
/// @brief output: a file with wheel odometry xyz qxqyqzqw
/// @param fileNum
/// @param imu
/// @param imuOdomPose
void imuOdometry(int fileNum, double imu[][9], double imuOdomPose[][7])
{
    double dt = 0.1;
    double imu_x_ = 0.0;
    double imu_y_ = 0.0;
    double imu_vx_ = 0.0;
    double imu_vy_ = 0.0;
    double imu_th_ = 0.0;

    for (int i = 0; i < fileNum; i++)
    {
        double ax = imu[i][0];
        double ay = imu[i][1];
        double vth = imu[i][5];
        double delta_th = vth * dt;
        double vx = imu_vx_ + ax * dt;
        double vy = imu_vy_ + ay * dt;

        double delta_x = ((imu_vx_ + vx) / 2 * cos(imu_th_) - (imu_vy_ + vy) / 2 * sin(imu_th_)) * dt; // m
        double delta_y = ((imu_vx_ + vx) / 2 * sin(imu_th_) + (imu_vy_ + vy) / 2 * cos(imu_th_)) * dt; // m

        imu_x_ += delta_x;
        imu_y_ += delta_y;
        imu_vx_ = vx;
        imu_vy_ = vy;
        imu_th_ += delta_th;

        double tmp_x = imu_x_ * cos(imu2lidar_yaw) - imu_y_ * sin(imu2lidar_yaw);
        double tmp_y = imu_x_ * sin(imu2lidar_yaw) + imu_y_ * cos(imu2lidar_yaw);
        double tmp_vx = imu_vx_ * cos(imu2lidar_yaw) - imu_vy_ * sin(imu2lidar_yaw);
        double tmp_vy = imu_vx_ * sin(imu2lidar_yaw) + imu_vy_ * cos(imu2lidar_yaw);
        double tmp_ax = ax * cos(imu2lidar_yaw) - ay * sin(imu2lidar_yaw);
        double tmp_ay = ax * sin(imu2lidar_yaw) + ay * cos(imu2lidar_yaw);

        std::vector<double> odom_quat = EulerToQuaternion(0, 0, imu_th_);
        imuOdomPose[i][0] = imu_x_;
        imuOdomPose[i][1] = imu_y_;
        imuOdomPose[i][2] = 0.0;
        imuOdomPose[i][3] = odom_quat[0];
        imuOdomPose[i][4] = odom_quat[1];
        imuOdomPose[i][5] = odom_quat[2];
        imuOdomPose[i][6] = odom_quat[3];
    }
}

/// @brief input : velocity of 4 wheels
/// @brief output: a file with wheel odometry xyz qxqyqzqw
/// @param fileNum
/// @param velocity
/// @param wheelOdomPose
void wheelOdometry(int fileNum, double velocity[][4], double wheelOdomPose[][7])
{

    double dt = 0.1; // s
    double x_ = 0.0;
    double y_ = 0.0;
    double th_ = 0.0;

    for (int i = 0; i < fileNum; i++)
    {
        velocities vel = GetVelocities(velocity[i][0],
                                       velocity[i][1],
                                       velocity[i][2],
                                       velocity[i][3]);
        double vx = vel.linear_x;
        double vy = vel.linear_y;
        double vth = vel.angular_z;

        double delta_th = vth * dt;
        double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt; // m
        double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt; // m

        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;

        std::vector<double> odom_quat = EulerToQuaternion(0, 0, th_);
        wheelOdomPose[i][0] = x_;
        wheelOdomPose[i][1] = y_;
        wheelOdomPose[i][2] = 0.0;
        wheelOdomPose[i][3] = odom_quat[0];
        wheelOdomPose[i][4] = odom_quat[1];
        wheelOdomPose[i][5] = odom_quat[2];
        wheelOdomPose[i][6] = odom_quat[3];
    }
}

void imuAndWheelOdometry(int fileNum,
                         double imu[][9],
                         double velocity[][4],
                         double gnss[][7],
                         double pose[][7])
{

    YAML::Node config = YAML::LoadFile("../config/params.yaml");
    std::vector<double> imu2lidar_matrix = config["imu2lidar_matrix"].as<vector<double>>();

    Eigen::Affine3d imu2lidar;
    imu2lidar(0, 0) = imu2lidar_matrix[0];
    imu2lidar(0, 1) = imu2lidar_matrix[1];
    imu2lidar(0, 2) = imu2lidar_matrix[2];
    imu2lidar(1, 0) = imu2lidar_matrix[3];
    imu2lidar(1, 1) = imu2lidar_matrix[4];
    imu2lidar(1, 2) = imu2lidar_matrix[5];
    imu2lidar(2, 0) = imu2lidar_matrix[6];
    imu2lidar(2, 1) = imu2lidar_matrix[7];
    imu2lidar(2, 2) = imu2lidar_matrix[8];
    imu2lidar(0, 3) = imu2lidar_matrix[9];
    imu2lidar(1, 3) = imu2lidar_matrix[10];
    imu2lidar(2, 3) = imu2lidar_matrix[11];

    ExtendedKalmanFilter ekf;
    double imu_measure[fileNum][9];
    double wheel_measure[fileNum][5];
    double gnss_measure[fileNum][3];

    double dt = 0.10; // s

    // get wheel measurment data
    double wheel_x_ = 0.0;
    double wheel_y_ = 0.0;
    double wheel_th_ = 0.0;

    for (int i = 0; i < fileNum; i++)
    {
        velocities vel = GetVelocities(velocity[i][0],
                                       velocity[i][1],
                                       velocity[i][2],
                                       velocity[i][3]);

        double vx = vel.linear_x;
        double vy = vel.linear_y;
        double vth = vel.angular_z;

        double delta_th = vth * dt;
        double delta_x = (vx * cos(wheel_th_) - vy * sin(wheel_th_)) * dt; // m
        double delta_y = (vx * sin(wheel_th_) + vy * cos(wheel_th_)) * dt; // m

        wheel_x_ += delta_x;
        wheel_y_ += delta_y;
        wheel_th_ += delta_th;

        wheel_measure[i][0] = -wheel_x_;
        wheel_measure[i][1] = -wheel_y_;
        wheel_measure[i][2] = vx;
        wheel_measure[i][3] = vy;
        wheel_measure[i][4] = wheel_th_;
    }

    // get imu measurment data
    double imu_x_ = 0.0;
    double imu_y_ = 0.0;
    double imu_vx_ = 0.0;
    double imu_vy_ = 0.0;
    double imu_th_ = 0.0;

    for (int i = 0; i < fileNum; i++)
    {
        double ax = imu[i][0];
        double ay = imu[i][1];
        double vth = imu[i][5];
        double delta_th = vth * dt;
        double delta_vx = ax * dt;
        double delta_vy = ay * dt;
        // 速度分解到x y方向求delta_x,delta_y
        double delta_x = (delta_vx * cos(imu_th_) - delta_vy * sin(imu_th_)) * dt; // m
        double delta_y = (delta_vx * sin(imu_th_) + delta_vy * cos(imu_th_)) * dt; // m

        imu_x_ += delta_x;
        imu_y_ += delta_y;
        imu_vx_ += delta_vx;
        imu_vy_ += delta_vy;
        imu_th_ += delta_th;

        // 从imu变换到lidar坐标
        double tmp_x = imu_x_ * cos(imu2lidar_yaw) - imu_y_ * sin(imu2lidar_yaw);
        double tmp_y = imu_x_ * sin(imu2lidar_yaw) + imu_y_ * cos(imu2lidar_yaw);
        double tmp_vx = imu_vx_ * cos(imu2lidar_yaw) - imu_vy_ * sin(imu2lidar_yaw);
        double tmp_vy = imu_vx_ * sin(imu2lidar_yaw) + imu_vy_ * cos(imu2lidar_yaw);
        double tmp_ax = ax * cos(imu2lidar_yaw) - ay * sin(imu2lidar_yaw);
        double tmp_ay = ax * sin(imu2lidar_yaw) + ay * cos(imu2lidar_yaw);

        // 从lidar变化到轮速计坐标
        imu_measure[i][0] = tmp_x + imu2lidar_tx + lidar2wheel_x;
        imu_measure[i][1] = tmp_y + imu2lidar_ty;
        imu_measure[i][2] = tmp_vx;
        imu_measure[i][3] = tmp_vy;
        imu_measure[i][4] = tmp_ax;
        imu_measure[i][5] = tmp_ay;
        imu_measure[i][6] = 0;
        imu_measure[i][7] = 0;
        imu_measure[i][8] = imu_th_;
    }

    // get gnss measurment data
    Eigen::Quaterniond q_init;
    q_init.x() = gnss[0][3];
    q_init.y() = gnss[0][4];
    q_init.z() = gnss[0][5];
    q_init.w() = gnss[0][6];
    double t_x_init = gnss[0][0];
    double t_y_init = gnss[0][1];
    double t_z_init = gnss[0][2];
    Eigen::Translation3d t_init(t_x_init, t_y_init, t_z_init);
    Eigen::Affine3d scr2global_init = t_init * q_init.toRotationMatrix() * imu2lidar;
    Eigen::Affine3d inverseMatrix = scr2global_init.inverse();

    for (int i = 0; i < fileNum; i++)
    {

        Eigen::Translation3d t;
        t.x() = gnss[i][0];
        t.y() = gnss[i][1];
        t.z() = gnss[i][2];
        Eigen::Quaterniond q;
        q.x() = gnss[i][3];
        q.y() = gnss[i][4];
        q.z() = gnss[i][5];
        q.w() = gnss[i][6];
        Eigen::Affine3d gnss_pose = t * q.toRotationMatrix() * imu2lidar;
        gnss_pose = inverseMatrix * gnss_pose;

        Eigen::Translation3d t_rel(gnss_pose.translation());
        Eigen::Quaterniond q_rel(gnss_pose.rotation());
        double x = t_rel.x();
        double y = t_rel.y();
        std::vector<double> euler = QuaternionToEuler(q_rel.w(),
                                                      q_rel.x(),
                                                      q_rel.y(),
                                                      q_rel.z());
        gnss_measure[i][0] = x - 1.534709;
        // gnss_measure[i][0] = x - lidar2wheel_x;
        gnss_measure[i][1] = y;
        gnss_measure[i][2] = euler[2];
    }

    double ekfPredict[fileNum][9];
    ekf.InitParams();

    for (int i = 0; i < fileNum; i++)
    {
        // use imu
        // Eigen::Vector<double, 9> z1;
        // z1 <<
        //     imu_measure[i][0],
        //     imu_measure[i][1],
        //     imu_measure[i][2],
        //     imu_measure[i][3],
        //     imu_measure[i][4],
        //     imu_measure[i][5],
        //     imu_measure[i][6],
        //     imu_measure[i][7],
        //     imu_measure[i][8];

        // use wheel
        Eigen::Vector<double, 5> z2;
        z2 << wheel_measure[i][0],
            wheel_measure[i][1],
            wheel_measure[i][2],
            wheel_measure[i][3],
            wheel_measure[i][4];

        // use gnss
        Eigen::Vector<double, 3> z3;
        z3 << gnss_measure[i][0],
            gnss_measure[i][1],
            gnss_measure[i][2];

        ekf.SetZ2(z2);
        ekf.SetZ3(z3);
        ekf.Prediction();
        ekf.Update();
        Eigen::Vector<double, 9> x_pred = ekf.GetX_pred();
        // for (int j=0; j<9; j++){
        //     ekfPredict[i][j] = x_pred[j];
        // }
        // 有个问题是 点云在叠加的时候，只会竖着叠不会往前走 why？
        ekfPredict[i][0] = -x_pred[0];
        ekfPredict[i][1] = -x_pred[1];
        ekfPredict[i][2] = x_pred[2];
        ekfPredict[i][3] = x_pred[3];
        ekfPredict[i][4] = x_pred[4];
        ekfPredict[i][5] = x_pred[5];
        ekfPredict[i][6] = x_pred[6];
        ekfPredict[i][7] = x_pred[7];
        // ekfPredict[i][8] = x_pred[8];
        ekfPredict[i][8] = x_pred[8] - 3.1415*0.5;
    }

    // now compute x y z qx qy qz qw from ekfPredict[][],which contains x y vx vy ax ay RPY
    for (int i = 0; i < fileNum; i++)
    {
        std::vector<double> odom_quat = EulerToQuaternion(0, 0, ekfPredict[i][8]);
        pose[i][0] = ekfPredict[i][0];
        pose[i][1] = ekfPredict[i][1];
        pose[i][2] = 0;
        pose[i][3] = odom_quat[0];
        pose[i][4] = odom_quat[1];
        pose[i][5] = odom_quat[2];
        pose[i][6] = odom_quat[3];

        if (i % 5000 == 0)
        {
            cout << "------------------" << i << "------------------" << endl;
            cout << "x" << endl;
            cout << "ekfPredict x = " << ekfPredict[i][0] << endl;
            cout << "gnss_measure x = " << gnss_measure[i][0] << endl;
            cout << "wheel_measure x = " << wheel_measure[i][0] << endl;
            cout << " " << endl;

            cout << "y" << endl;
            std::cout << "ekfPredict y = " << ekfPredict[i][1] << std::endl;
            std::cout << "gnss_measure y = " << gnss_measure[i][1] << std::endl;
            std::cout << "wheel_measure y = " << wheel_measure[i][1] << std::endl;
            cout << " " << endl;

            cout << "yaw" << endl;
            std::cout << "gnss_measure yaw = " << gnss_measure[i][2] << std::endl;
            std::cout << "wheel_measure yaw = " << wheel_measure[i][4] << std::endl;
            std::cout << "ekfPredict yaw = " << ekfPredict[i][8] << std::endl;

            cout << "pose ： " << endl;
            std::cout << "pose x = " << pose[i][0] << std::endl;
            std::cout << "pose y = " << pose[i][1] << std::endl;
            std::vector<double> pose_euler =  QuaternionToEuler(pose[i][6],pose[i][3], pose[i][4], pose[i][5]);
            // cout << pose_euler[0] << endl;
            // cout << pose_euler[1] << endl;
            cout << pose_euler[2] << endl;
        }
    }
}

/// @brief x y vx vy ax av roll pitch yaw
/// @param imu
void ekf_imu(double imu[9])
{
    // -----------------------//
    // 这中间的部分放到init里初始化
    // ekf类里有记录 状态向量，状态协方差矩阵
    // imu是九个imu的数据 加速度 角速度 角度
    Eigen::Affine3d imu2lidar;
    imu2lidar(0, 0) = 0.00334014;
    imu2lidar(0, 1) = 0.999924;
    imu2lidar(0, 2) = 0.0118571;
    imu2lidar(1, 0) = -0.999837;
    imu2lidar(1, 1) = 0.00312901;
    imu2lidar(1, 2) = 0.0177801;
    imu2lidar(2, 0) = 0.0177417;
    imu2lidar(2, 1) = -0.0119146;
    imu2lidar(2, 2) = 0.999772;
    imu2lidar(0, 3) = -1.19292;
    imu2lidar(1, 3) = -0.0230692;
    imu2lidar(2, 3) = -1.28574;

    ExtendedKalmanFilter ekf;
    ekf.InitParams();
    double dt = 0.1; // s
    // -----------------------//

    Eigen::VectorXd state_vector = ekf.GetX();
    double ax = imu[0];
    double ay = imu[1];
    double vth = imu[5];
    double delta_th = vth * dt;
    double vx_ = state_vector[2];
    double vy_ = state_vector[3];
    double vx = vx_ + ax * dt;
    double vy = vy_ + ay * dt;
    double imu_th_ = state_vector[8];

    double delta_x = ((vx_ + vx) / 2 * cos(imu_th_) - (vy_ + vy) / 2 * sin(imu_th_)) * dt; // m
    double delta_y = ((vx_ + vx) / 2 * sin(imu_th_) + (vy_ + vy) / 2 * cos(imu_th_)) * dt; // m

    double x = state_vector[0] + delta_x;
    double y = state_vector[1] + delta_y;
    imu_th_ += delta_th;

    double tmp_x = x * cos(imu2lidar_yaw) - y * sin(imu2lidar_yaw) - imu2lidar_tx - lidar2wheel_x;
    double tmp_y = x * sin(imu2lidar_yaw) + y * cos(imu2lidar_yaw) - imu2lidar_ty;
    double tmp_vx = vx * cos(imu2lidar_yaw) - vy * sin(imu2lidar_yaw);
    double tmp_vy = vx * sin(imu2lidar_yaw) + vy * cos(imu2lidar_yaw);
    double tmp_ax = ax * cos(imu2lidar_yaw) - ay * sin(imu2lidar_yaw);
    double tmp_ay = ax * sin(imu2lidar_yaw) + ay * cos(imu2lidar_yaw);

    Eigen::Vector<double, 9> imu_measure;
    imu_measure[0] = tmp_x;
    imu_measure[1] = tmp_y;
    imu_measure[2] = tmp_vx;
    imu_measure[3] = tmp_vy;
    imu_measure[4] = tmp_ax;
    imu_measure[5] = tmp_ay;
    imu_measure[6] = 0.0;
    imu_measure[7] = 0.0;
    imu_measure[8] = imu_th_;

    ekf.SetZ1(imu_measure);
    ekf.Prediction();
    ekf.Update();
}

/// @brief x y vx vy yaw
/// @param wheel
void ekf_gnss(double wheel[5])
{
    // -----------------------//
    // 这中间的部分放到init里初始化
    // ekf类里有记录 状态向量，状态协方差矩阵
    // wheel是轮速计的四个速度
    Eigen::Affine3d imu2lidar;
    imu2lidar(0, 0) = 0.00334014;
    imu2lidar(0, 1) = 0.999924;
    imu2lidar(0, 2) = 0.0118571;
    imu2lidar(1, 0) = -0.999837;
    imu2lidar(1, 1) = 0.00312901;
    imu2lidar(1, 2) = 0.0177801;
    imu2lidar(2, 0) = 0.0177417;
    imu2lidar(2, 1) = -0.0119146;
    imu2lidar(2, 2) = 0.999772;
    imu2lidar(0, 3) = -1.19292;
    imu2lidar(1, 3) = -0.0230692;
    imu2lidar(2, 3) = -1.28574;

    ExtendedKalmanFilter ekf;
    double dt = 0.1; // s
    // -----------------------//
    Eigen::VectorXd state_vector = ekf.GetX();
    velocities vel = GetVelocities(wheel[0], wheel[1], wheel[2], wheel[3]);
    double vx = vel.linear_x;
    double vy = vel.linear_y;
    double vth = vel.angular_z;
    double delta_th = vth * dt;
    double wheel_th_ = state_vector[8];
    double delta_x = (vx * cos(wheel_th_) - vy * sin(wheel_th_)) * dt; // m
    double delta_y = (vx * sin(wheel_th_) + vy * cos(wheel_th_)) * dt; // m

    Eigen::Vector<double, 5> wheel_measure;
    wheel_measure[0] = (state_vector[0] + delta_x);
    wheel_measure[1] = (state_vector[1] + delta_y);
    wheel_measure[2] = (vx);
    wheel_measure[3] = (vy);
    wheel_measure[4] = (state_vector[8] + delta_th);
    ekf.SetZ2(wheel_measure);
    ekf.Prediction();
    ekf.Update();
}

/// @brief wheel x y vx vy yaw
/// @brief gnss  x y yaw
/// @param gnss
void ekf_gnss(double gnss[7], double gnss_init[7])
{
    // -----------------------//
    // 这中间的部分放到init里初始化
    // ekf类里有记录 状态向量，状态协方差矩阵

    Eigen::Affine3d imu2lidar;
    imu2lidar(0, 0) = 0.00334014;
    imu2lidar(0, 1) = 0.999924;
    imu2lidar(0, 2) = 0.0118571;
    imu2lidar(1, 0) = -0.999837;
    imu2lidar(1, 1) = 0.00312901;
    imu2lidar(1, 2) = 0.0177801;
    imu2lidar(2, 0) = 0.0177417;
    imu2lidar(2, 1) = -0.0119146;
    imu2lidar(2, 2) = 0.999772;
    imu2lidar(0, 3) = -1.19292;
    imu2lidar(1, 3) = -0.0230692;
    imu2lidar(2, 3) = -1.28574;

    ExtendedKalmanFilter ekf;
    double dt = 0.1; // s

    Eigen::Quaterniond q_init;
    q_init.x() = gnss_init[3];
    q_init.y() = gnss_init[4];
    q_init.z() = gnss_init[5];
    q_init.w() = gnss_init[6];
    double t_x_init = gnss_init[0];
    double t_y_init = gnss_init[1];
    double t_z_init = gnss_init[2];
    Eigen::Translation3d t_init(t_x_init, t_y_init, t_z_init);
    Eigen::Affine3d scr2global_init = t_init * q_init.toRotationMatrix() * imu2lidar;
    Eigen::Affine3d inverseMatrix = scr2global_init.inverse();

    // -----------------------//
    Eigen::Affine3d gnss_pose;
    Eigen::Translation3d t;
    t.x() = gnss[0];
    t.y() = gnss[1];
    t.z() = gnss[2];
    Eigen::Quaterniond q;
    q.x() = gnss[3];
    q.y() = gnss[4];
    q.z() = gnss[5];
    q.w() = gnss[6];
    gnss_pose = t * q.toRotationMatrix() * imu2lidar;
    gnss_pose = inverseMatrix * gnss_pose;

    Eigen::Translation3d t_rel(gnss_pose.translation());
    Eigen::Quaterniond q_rel(gnss_pose.rotation());
    double x = t_rel.x();
    double y = t_rel.y();
    std::vector<double> euler = QuaternionToEuler(q_rel.w(),
                                                  q_rel.x(),
                                                  q_rel.y(),
                                                  q_rel.z());
    Eigen::Vector<double, 3> gnss_measure;
    gnss_measure[0] = (x - lidar2wheel_x);
    gnss_measure[1] = (y);
    gnss_measure[2] = (euler[2]);

    ekf.SetZ3(gnss_measure);
    ekf.Prediction();
    ekf.Update();
}

void ekf_gnss_wheel(double wheel[], double gnss[7], double gnss_init[7])
{
    // -----------------------//
    // 这中间的部分放到init里初始化
    // ekf类里有记录 状态向量，状态协方差矩阵
    // wheel是轮速计的四个速度
    Eigen::Affine3d imu2lidar;
    imu2lidar(0, 0) = 0.00334014;
    imu2lidar(0, 1) = 0.999924;
    imu2lidar(0, 2) = 0.0118571;
    imu2lidar(1, 0) = -0.999837;
    imu2lidar(1, 1) = 0.00312901;
    imu2lidar(1, 2) = 0.0177801;
    imu2lidar(2, 0) = 0.0177417;
    imu2lidar(2, 1) = -0.0119146;
    imu2lidar(2, 2) = 0.999772;
    imu2lidar(0, 3) = -1.19292;
    imu2lidar(1, 3) = -0.0230692;
    imu2lidar(2, 3) = -1.28574;

    Eigen::Quaterniond q_init;
    q_init.x() = gnss_init[3];
    q_init.y() = gnss_init[4];
    q_init.z() = gnss_init[5];
    q_init.w() = gnss_init[6];
    double t_x_init = gnss_init[0];
    double t_y_init = gnss_init[1];
    double t_z_init = gnss_init[2];
    Eigen::Translation3d t_init(t_x_init, t_y_init, t_z_init);
    Eigen::Affine3d scr2global_init = t_init * q_init.toRotationMatrix() * imu2lidar;
    Eigen::Affine3d inverseMatrix = scr2global_init.inverse();

    ExtendedKalmanFilter ekf;
    double dt = 0.1; // s
    // -----------------------//
    Eigen::VectorXd state_vector = ekf.GetX();

    velocities vel = GetVelocities(wheel[0], wheel[1], wheel[2], wheel[3]);
    double wheel_vx = vel.linear_x;
    double wheel_vy = vel.linear_y;
    double wheel_vth = vel.angular_z;
    double wheel_delta_th = wheel_vth * dt;
    double wheel_th_ = state_vector[8];
    double wheel_delta_x = (wheel_vx * cos(wheel_th_) - wheel_vy * sin(wheel_th_)) * dt; // m
    double wheel_delta_y = (wheel_vx * sin(wheel_th_) + wheel_vy * cos(wheel_th_)) * dt; // m

    Eigen::Vector<double, 5> wheel_measure;
    wheel_measure[0] = -(state_vector[0] + wheel_delta_x);
    wheel_measure[1] = -(state_vector[1] + wheel_delta_y);
    wheel_measure[2] = (wheel_vx);
    wheel_measure[3] = (wheel_vy);
    wheel_measure[4] = (state_vector[8] + wheel_delta_th);

    Eigen::Affine3d gnss_pose;
    Eigen::Translation3d t;
    t.x() = gnss[0];
    t.y() = gnss[1];
    t.z() = gnss[2];
    Eigen::Quaterniond q;
    q.x() = gnss[3];
    q.y() = gnss[4];
    q.z() = gnss[5];
    q.w() = gnss[6];
    gnss_pose = t * q.toRotationMatrix() * imu2lidar;
    gnss_pose = inverseMatrix * gnss_pose;

    Eigen::Translation3d t_rel(gnss_pose.translation());
    Eigen::Quaterniond q_rel(gnss_pose.rotation());
    double gnss_x = t_rel.x();
    double gnss_y = t_rel.y();
    std::vector<double> euler = QuaternionToEuler(q_rel.w(),
                                                  q_rel.x(),
                                                  q_rel.y(),
                                                  q_rel.z());
    Eigen::Vector<double, 3> gnss_measure;
    gnss_measure[0] = (gnss_x - lidar2wheel_x);
    gnss_measure[1] = (gnss_y);
    gnss_measure[2] = (euler[2]);

    ekf.SetZ2(wheel_measure);
    ekf.SetZ3(gnss_measure);
    ekf.Prediction();
    ekf.Update();
}

velocities GetVelocities(float vel_front_left, float vel_front_right, float vel_rear_left, float vel_rear_right)
{
    velocities vel;
    float average_rps_x;
    // float average_rps_y;
    float average_rps_a;

    // convert km/h to m/s
    average_rps_x = ((float)(vel_front_left + vel_front_right + vel_rear_left + vel_rear_right) / 4) * 1000.0 / 3600.0; // m/s
    vel.linear_x = average_rps_x;                                                                                       // * wheel_circumference_; // m/s

    // convert km/h to m/s
    //  average_rps_y = ((float)(-vel_front_left + vel_front_right + vel_rear_left - vel_rear_right) / total_wheels_) / 60; // RPM

    vel.linear_y = 0;

    // convert km/h to m/s
    average_rps_a = ((float)(-vel_front_left + vel_front_right - vel_rear_left + vel_rear_right) / 2) * 1000.0 / 3600.0;
    vel.angular_z = average_rps_a / wheel_distance_y; //  rad/s

    return vel;
}

std::vector<double> EulerToQuaternion(double roll, double pitch, double yaw)
{
    double coeff = static_cast<double>(0.5);
    double r = roll * coeff;
    double p = pitch * coeff;
    double y = yaw * coeff;

    double sr = std::sin(r);
    double sp = std::sin(p);
    double sy = std::sin(y);

    double cr = std::cos(r);
    double cp = std::cos(p);
    double cy = std::cos(y);

    double qw = cr * cp * cy + sr * sp * sy;
    double qx = sr * cp * cy - cr * sp * sy;
    double qy = cr * sp * cy + sr * cp * sy;
    double qz = cr * cp * sy - sr * sp * cy;

    return {qx, qy, qz, qw};
}

std::vector<double> QuaternionToEuler(double qw, double qx, double qy, double qz)
{
    double euler_yaw;
    double euler_pitch;
    double euler_roll;

    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    euler_roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        euler_pitch = std::copysign(M_PI / 2, sinp);
    else
        euler_pitch = std::asin(sinp);

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    euler_yaw = std::atan2(siny_cosp, cosy_cosp);

    return {euler_roll, euler_pitch, euler_yaw};
}