#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "cyber/demo_vslam/fileProcess.h"
#include "cyber/demo_vslam/proto/vslam.pb.h"
#include "yaml-cpp/yaml.h"
// 用apollo/modules/localization/proto/gps.proto
    // optional apollo.common.Header header = 1;
    // optional apollo.localization.Pose localization = 2;
// #include "modules/localization/proto/gps.pb.h"

using apollo::cyber::demo_vslam::proto::CarPose;

// using apollo::modules::localization::proto::Gps

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

        // auto msg = std::make_shared<Gps>();
        // auto mutable_head = msg.mutable_transform();
        // mutable_head->set_timestamp_sec(localization.measurement_time());
       
        // auto mutable_translation = tf2_msg.mutable_transform()->mutable_translation();
        // mutable_translation->set_x(localization.pose().position().x());
        // mutable_translation->set_y(localization.pose().position().y());
        // mutable_translation->set_z(localization.pose().position().z());

        // auto mutable_rotation = tf2_msg.mutable_transform()->mutable_rotation();
        // mutable_rotation->set_qx(localization.pose().orientation().qx());
        // mutable_rotation->set_qy(localization.pose().orientation().qy());
        // mutable_rotation->set_qz(localization.pose().orientation().qz());
        // mutable_rotation->set_qw(localization.pose().orientation().qw());

        // msg->set_x(pose[i][0]);
        // msg->set_y(pose[i][1]);
        // msg->set_z(pose[i][2]);
        // msg->set_qx(pose[i][3]);
        // msg->set_qy(pose[i][4]);
        // msg->set_qz(pose[i][5]);
        // msg->set_qw(pose[i][6]);
        talker->Write(msg);
        rate.Sleep();
    }
    return 0;
}
