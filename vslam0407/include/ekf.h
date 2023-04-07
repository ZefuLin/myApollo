#ifndef EKF_H
#define EKF_H

#include <Eigen/Dense>
#include <iostream>
#include <cstdlib>

class ExtendedKalmanFilter
{
private:
    const static int STATE_DIM = 9; // 状态向量的维度x y，vx vy，ax，ay，RPY（x轴对应R，y轴对应P），Y不能超过90度
    const static int MEAS_DIM1 = 9; // 传感器1 IMU 的测量向量维度 ax，ay积分一次得到vx，vy，再积分一次得到x，y，还有RPY
    const static int MEAS_DIM2 = 5; // 传感器2 轮速计 的测量向量维度 x y，vx vy，yaw
    const static int MEAS_DIM3 = 3; // 传感器3 gnss 的测量向量维度 x y yaw

    bool is_initialized = false;
    double delta_t = 0.1;           // 时间间隔
    double noise_var1 = 0.01;  // imu 噪声方差
    double noise_var2 = 0.01; // 轮速计 噪声方差
    double noise_var3 = 0.01; // gnss 噪声方差

    Eigen::Vector<double, STATE_DIM> x;                 // 状态向量
    Eigen::Vector<double, STATE_DIM> x_pred;            // 状态向量预测值
    Eigen::Vector<double, MEAS_DIM1> z1;                // 传感器1 imu 测量向量
    Eigen::Vector<double, MEAS_DIM2> z2;                // 传感器2 轮速计 测量向量
    Eigen::Vector<double, MEAS_DIM3> z3;                // 传感器3 gnss 测量向量

    Eigen::Matrix<double, STATE_DIM, STATE_DIM> F;      // 状态转移矩阵
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P;      // 状态协方差矩阵
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> P_pred; // 状态协方差矩阵预测值
    Eigen::Matrix<double, STATE_DIM, STATE_DIM> Q;      // 过程噪声矩阵

    Eigen::Matrix<double, STATE_DIM, MEAS_DIM1> K1;     // 卡尔曼增益
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM2> K2;     // 卡尔曼增益
    Eigen::Matrix<double, STATE_DIM, MEAS_DIM3> K3;     // 卡尔曼增益

    Eigen::Matrix<double, MEAS_DIM1, STATE_DIM> H1;     // 测量矩阵
    Eigen::Matrix<double, MEAS_DIM2, STATE_DIM> H2;     // 测量矩阵
    Eigen::Matrix<double, MEAS_DIM3, STATE_DIM> H3;     // 测量矩阵

    Eigen::Matrix<double, MEAS_DIM1, MEAS_DIM1> R1;     // 测量噪声矩阵
    Eigen::Matrix<double, MEAS_DIM2, MEAS_DIM2> R2;     // 测量噪声矩阵
    Eigen::Matrix<double, MEAS_DIM3, MEAS_DIM3> R3;     // 测量噪声矩阵

public:
    ExtendedKalmanFilter(/* args */);
    ~ExtendedKalmanFilter();

    void InitX();

    void SetZ1(Eigen::Vector<double, MEAS_DIM1> z_in);
    void SetZ2(Eigen::Vector<double, MEAS_DIM2> z_in);
    void SetZ3(Eigen::Vector<double, MEAS_DIM3> z_in);

    void SetDelta_t(double t_in);
    void SetNoise_var1(double noise_in);
    void SetNoise_var2(double noise_in);
    void SetNoise_var3(double noise_in);

    Eigen::VectorXd GetX();
    Eigen::VectorXd GetX_pred();

    void InitParams();
    void InitF();
    void InitP();
    void InitQ();
    void InitK1();
    void InitK2();
    void InitK3();
    void InitH1();
    void InitH2();
    void InitH3();
    void InitR1();
    void InitR2();
    void InitR3();
    void Prediction();
    void Update();
};

#endif // EKF_H