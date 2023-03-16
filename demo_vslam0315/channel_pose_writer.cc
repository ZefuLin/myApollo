#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "cyber/demo_vslam/fileProcess.h"
#include "cyber/demo_vslam/proto/vslam.pb.h"
#include "yaml-cpp/yaml.h"


using apollo::cyber::demo_vslam::proto::CarPose;

int main(int argc, char *argv[]){
    apollo::cyber::Init(argv[0]);
    auto talker_node = apollo::cyber::CreateNode("pose_writer");
    auto talker = talker_node->CreateWriter<CarPose>("channel/pose");
    apollo::cyber::Rate rate(1.0);
    std::string content("car pose");
    YAML::Node config = YAML::LoadFile("/apollo/cyber/demo_vslam/conf/params.yaml");

    string path = config["imu_odom_path"].as<string>();
    int imu_file_num = config["imu_file_num"].as<int>();
    double pose[imu_file_num][7];
    readDataFromIMUOdometry(path, imu_file_num, pose);
    
    for(int i=0; i<imu_file_num;i++){
        auto msg = std::make_shared<CarPose>();
        msg->set_x(pose[i][0]);
        msg->set_y(pose[i][1]);
        msg->set_z(pose[i][2]);
        msg->set_qx(pose[i][3]);
        msg->set_qy(pose[i][4]);
        msg->set_qz(pose[i][5]);
        msg->set_qw(pose[i][6]);
        talker->Write(msg);
        AINFO << "pose_writer sent a message! No. " << i;
        // cout << "file " << i << endl;
        // cout << msg->y() << endl;
        // check过了 这个channel的数据没问题 3.15
        rate.Sleep();
    }
    return 0;
}
