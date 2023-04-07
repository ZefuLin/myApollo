#include "ekf.h"
using namespace std;

ExtendedKalmanFilter::ExtendedKalmanFilter(/* args */){}

ExtendedKalmanFilter::~ExtendedKalmanFilter(){
}

void ExtendedKalmanFilter::InitX(){
    x << 0,0,0,0,0,0,0,0,0;
    is_initialized = true;
    cout << "x is initialized! " << endl;
}
void ExtendedKalmanFilter::SetZ1(Eigen::Vector<double, MEAS_DIM1> z_in){
    z1 = z_in;
}
void ExtendedKalmanFilter::SetZ2(Eigen::Vector<double, MEAS_DIM2> z_in){
    z2 = z_in;
}
void ExtendedKalmanFilter::SetZ3(Eigen::Vector<double, MEAS_DIM3> z_in){
    z3 = z_in;
}

void ExtendedKalmanFilter::SetDelta_t(double t_in){
    delta_t = t_in;
}
void ExtendedKalmanFilter::SetNoise_var1(double noise_in){
    noise_var1 = noise_in;
}
void ExtendedKalmanFilter::SetNoise_var2(double noise_in){
    noise_var2 = noise_in;
}
void ExtendedKalmanFilter::SetNoise_var3(double noise_in){
    noise_var3 = noise_in;
}

Eigen::VectorXd ExtendedKalmanFilter::GetX(){
    return x;
}
Eigen::VectorXd ExtendedKalmanFilter::GetX_pred(){
    return x_pred;
}

void ExtendedKalmanFilter::InitF(){
    F << 1, 0, delta_t, 0, 0, 0, 0, 0, 0,
         0, 1, 0, delta_t, 0, 0, 0, 0, 0,
         0, 0, 1, 0, delta_t, 0, 0, 0, 0,
         0, 0, 0, 1, 0, delta_t, 0, 0, 0,
         0, 0, 0, 0, 1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1;
    cout << "F is initialized! " << endl;
}
void ExtendedKalmanFilter::InitP(){
    P << 10, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 10, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0.1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1;
    cout << "P is initialized! " << endl;
}

void ExtendedKalmanFilter::InitQ(){
    Q << 0.1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0.1, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 0.1, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 0.1, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 0.1, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 0.1, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 0.1, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 0.1;
    cout << "Q is initialized! " << endl;
}

void ExtendedKalmanFilter::InitK1(){
    K1 = Eigen::Matrix<double, STATE_DIM, MEAS_DIM1>::Identity();
}
void ExtendedKalmanFilter::InitK2(){
    K2 = Eigen::Matrix<double, STATE_DIM, MEAS_DIM2>::Identity();
}
void ExtendedKalmanFilter::InitK3(){
    K3 = Eigen::Matrix<double, STATE_DIM, MEAS_DIM3>::Identity();
}

void ExtendedKalmanFilter::InitH1(){
    H1 << 1, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 1, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 1, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 1, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1;
    cout << "H1 is initialized! " << endl;
}
void ExtendedKalmanFilter::InitH2(){
    H2 << 1, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 1, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 1, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1;
    cout << "H2 is initialized! " << endl;
}
void ExtendedKalmanFilter::InitH3(){
    H3 << 1, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 1, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 1;
    cout << "H3 is initialized! " << endl;
}
void ExtendedKalmanFilter::InitR1(){
    R1 << noise_var1, 0, 0, 0, 0, 0, 0, 0, 0,
          0, noise_var1, 0, 0, 0, 0, 0, 0, 0,
          0, 0, noise_var1, 0, 0, 0, 0, 0, 0,
          0, 0, 0, noise_var1, 0, 0, 0, 0, 0,
          0, 0, 0, 0, noise_var1, 0, 0, 0, 0,
          0, 0, 0, 0, 0, noise_var1, 0, 0, 0,
          0, 0, 0, 0, 0, 0, noise_var1, 0, 0,
          0, 0, 0, 0, 0, 0, 0, noise_var1, 0,
          0, 0, 0, 0, 0, 0, 0, 0, noise_var1;
    cout << "R1 is initialized! " << endl;
}

void ExtendedKalmanFilter::InitR2(){
    R2 << noise_var2, 0, 0, 0, 0,
          0, noise_var2, 0, 0, 0,
          0, 0, noise_var2, 0, 0,
          0, 0, 0, noise_var2, 0, 
          0, 0, 0, 0, noise_var2;
    cout << "R2 is initialized! " << endl;
}

void ExtendedKalmanFilter::InitR3(){
    R3 << noise_var3, 0, 0, 
          0, noise_var3, 0, 
          0, 0, noise_var3;
    cout << "R3 is initialized! " << endl;
}

void ExtendedKalmanFilter::InitParams(){
    InitX();
    InitF();
    InitP();
    InitQ();
    InitK1();
    InitK2();
    InitK3();
    InitH1();
    InitH2();
    InitH3();
    InitR1();
    InitR2();
    InitR3();
}

void ExtendedKalmanFilter::Prediction(){
    if(is_initialized){
        // 这次的x_pred就是下次的x
        x_pred = F * x;
        Eigen::MatrixXd Ft = F.transpose();
        P_pred = F * P * Ft + Q; 
    }else{
        cout << "The state vector is not initialized!" << endl;
        exit(0);
    }
}

void ExtendedKalmanFilter::Update(){
    // 下面是同时对wheel和gnss的数据做融合，然后取平均
    // z2 是wheel，z3是gnss

    // 关于重复运行的时候，输出结果不一样
    // 应该是打印的时候，虽然只是cout，但是对于eigen来说是传递了指针，所以相当于又操作了一遍
    // 所以才会每次打印出来都不是一个结果
    // 解决方案 上锁? 分离变量

    Eigen::Vector<double, MEAS_DIM2> y2 = z2 - H2 * x_pred;
    Eigen::Matrix<double, MEAS_DIM2, MEAS_DIM2> S2 = H2 * P_pred * H2.transpose() + R2;

    K2 = P_pred * H2.transpose() * S2.inverse();
    Eigen::Vector<double, STATE_DIM> x_pred_wheel = x_pred + K2 * y2;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM>  P_pred_wheel = 
        (Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() - K2 * H2) * P_pred;
    
    Eigen::Vector<double, MEAS_DIM3> y3 = z3 - H3 * x_pred; 
    Eigen::Matrix<double, MEAS_DIM3, MEAS_DIM3> S3 = H3 * P_pred * H3.transpose() + R3;
    
    K3 = P_pred * H3.transpose() * S3.inverse();
    Eigen::Vector<double, STATE_DIM> x_pred_gnss = x_pred + K3 * y3;
    Eigen::Matrix<double, STATE_DIM, STATE_DIM>  P_pred_gnss = 
        (Eigen::Matrix<double, STATE_DIM, STATE_DIM>::Identity() - K3 * H3) * P_pred;
    
    x_pred = 0.5 * x_pred_wheel + 0.5 * x_pred_gnss;
    P_pred = 0.5 * P_pred_wheel + 0.5 * P_pred_gnss;

    x = x_pred;
    P = P_pred;
    z2.setZero();
    z3.setZero();
}
    



