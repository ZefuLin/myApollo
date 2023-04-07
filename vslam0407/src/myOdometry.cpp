#include "myOdometry.h"

void calWheelOdom(double velocity[][4], double pose[][6], int fileNum){
    double const front_wheelspan = 1.570;//前轮距(mm)
    double const rear_wheelspan = 1.578;//后轮距(mm)
    double const delta_time = 0.012;//两帧之间间隔12ms 
    double const kmh2ms = 0.277777778;

    double accumulate_x_front;
    double accumulate_y_front;
    double accumulate_theta_front;
    double accumulate_x_rear;
    double accumulate_y_rear;
    double accumulate_theta_rear;

    for(int i=0;i<fileNum;i++){
        double v_left_front=velocity[i][0];//km/h
        double v_right_front=velocity[i][1];//km/h
        double v_left_rear=velocity[i][2];//km/h
        double v_right_rear=velocity[i][3];//km/h

        double forward_dis_front = (v_left_front + v_right_front) * kmh2ms * delta_time * 0.5;//车辆前进距离
        double forward_dis_rear = (v_left_rear + v_right_rear) * kmh2ms * delta_time * 0.5;//车辆前进距离
        double delta_dis_front = (-v_left_front + v_right_front) * delta_time;//前轮前进距离差
        double delta_dis_rear = (-v_left_rear + v_right_rear) * delta_time;//后轮前进距离差
        double delta_theta_front = delta_dis_front / front_wheelspan;//车辆前轮转弯角度
        double delta_theta_rear = delta_dis_rear / rear_wheelspan;//车辆后轮转弯角度

        double delta_x_front = forward_dis_front * cos(delta_theta_front);
        double delta_y_front = forward_dis_front * sin(delta_theta_front);
        double delta_x_rear = forward_dis_rear * cos(delta_theta_rear);
        double delta_y_rear = forward_dis_rear * sin(delta_theta_rear);

        double accumulate_x_front = accumulate_x_front 
                                    - delta_y_front * sin(accumulate_theta_front + delta_theta_front * 0.5)
                                    + delta_x_front * cos(accumulate_theta_front + delta_theta_front * 0.5);
        double accumulate_y_front = accumulate_y_front
                                    + delta_y_front * cos(accumulate_theta_front + delta_theta_front * 0.5)
                                    + delta_x_front * sin(accumulate_theta_front + delta_theta_front * 0.5);

        double accumulate_theta_front = accumulate_theta_front + delta_theta_front;

        double accumulate_x_rear = accumulate_x_rear 
                                    - delta_y_rear * sin(accumulate_theta_rear + delta_theta_rear * 0.5)
                                    + delta_x_rear * cos(accumulate_theta_rear + delta_theta_rear * 0.5);
        double accumulate_y_rear = accumulate_y_rear
                                    + delta_y_rear * cos(accumulate_theta_rear + delta_theta_rear * 0.5)
                                    + delta_x_rear * sin(accumulate_theta_rear + delta_theta_rear * 0.5);

        double accumulate_theta_rear = accumulate_theta_rear + delta_theta_rear;
        
        pose[i][0] = accumulate_x_front;
        pose[i][1] = accumulate_y_front;
        pose[i][2] = accumulate_theta_front;
        pose[i][3] = accumulate_x_rear;
        pose[i][4] = accumulate_y_rear;
        pose[i][5] = accumulate_theta_rear;
    }
}

/*
    使用EKF融合IMU和轮速计的信息
        IMU ->xyz,roll,pitch,yaw
        轮速计 -> x,y,yaw
    
    question:以下部件如何实现？
        测量方程
        状态转移方程
        状态转移矩阵
        测量矩阵
            测量值与状态值之间的关系
        噪声
            过程噪声，测量噪声
        协方差矩阵
        初始状态 
            以第一帧的位置，方向作为efk的初始状态
        观测量
            传感器的测量值
            暂定为imu和wheel odom的取个均值
        卡尔曼增益
*/

