#include <string>
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "cyber/demo_vslam/fileProcess.h"
#include "cyber/demo_vslam/proto/vslam.pb.h"
#include "yaml-cpp/yaml.h"

using apollo::cyber::demo_vslam::proto::pointCloud;

int main(int argc, char *argv[]){
    apollo::cyber::Init(argv[0]);
    auto talker_node = apollo::cyber::CreateNode("pointCloud_writer");
    auto talker = talker_node->CreateWriter<pointCloud>("channel/pointCloud");
    apollo::cyber::Rate rate(1.0);
    std::string content("point cloud");
    YAML::Node config = YAML::LoadFile("/apollo/cyber/demo_vslam/conf/params.yaml");
    string bev_cloud_path = config["bev_cloud_path"].as<string>();
    int bevCloud_file_num = config["bevCloud_file_num"].as<int>();
    //一个点云文件就有3000个点,共有bevCloud_file_num个点云文件
    for(int j=0; j<bevCloud_file_num; j++){
        string current_cloud_path = bev_cloud_path + to_string(j) + ".pcd";
        double currentPointCloud[3000][6];
        readDataFeomBEVCloud(current_cloud_path, currentPointCloud);  
        auto msg = std::make_shared<pointCloud>();
        for(int i=0; i<3000;i++){
            msg->add_x(currentPointCloud[i][0]);
            msg->add_y(currentPointCloud[i][1]);
            msg->add_z(currentPointCloud[i][2]);

            msg->add_r(currentPointCloud[i][3]);
            msg->add_g(currentPointCloud[i][4]);
            msg->add_b(currentPointCloud[i][5]);
            //对有内容的msg才可以用set(index, value),初始化只能用add
        }
        talker->Write(msg);
        // check过了 这个channel的数据没问题 3.15 是component的proc在获取数据的时候有问题
        // cout << "file " << j << endl;
        // cout << msg->y(2999)/0.004 << endl;
        AINFO << "pointCloud_writer sent a message! No. " << j;
        rate.Sleep();
    }
    
    return 0;
}
