#include <string>
#include "cyber/cyber.h"
#include "cyber/time/rate.h"
#include "cyber/time/time.h"
#include "modules/localization/vslam/fileProcess.h"
#include "modules/localization/vslam/proto/vslam.pb.h"
#include "yaml-cpp/yaml.h"

#include "modules/drivers/proto/pointcloud.pb.h"

using apollo::drivers::PointCloud;
using namespace std;

int main(int argc, char *argv[]){
    apollo::cyber::Init(argv[0]);
    auto talker_node = apollo::cyber::CreateNode("pointCloud_writer");
    auto talker = talker_node->CreateWriter<PointCloud>("channel/pointCloud");
    apollo::cyber::Rate rate(1.0);
    std::string content("point cloud");
    YAML::Node config = YAML::LoadFile("/apollo/cyber/demo_vslam/conf/params.yaml");
    string bev_cloud_path = config["bev_cloud_path"].as<string>();
    int bevCloud_file_num = config["bevCloud_file_num"].as<int>();
    //一个点云文件就有3000个点,共有bevCloud_file_num个点云文件
    for(int j=0; j<bevCloud_file_num; j++){
        string current_cloud_path = bev_cloud_path + to_string(j) + ".pcd";
        double currentPointCloud[3000][6];
        readDataFromBEVCloud(current_cloud_path, currentPointCloud);  
        auto pointcloud = std::make_shared<PointCloud>();
        for(int i=0; i<3000;i++){
            pointcloud->add_point();
            pointcloud->mutable_point(i)->set_x(currentPointCloud[i][0]);
            pointcloud->mutable_point(i)->set_y(currentPointCloud[i][1]);
            pointcloud->mutable_point(i)->set_z(currentPointCloud[i][2]);

            uint32_t r,g,b,intensity;
            r = currentPointCloud[i][3];
            g = currentPointCloud[i][4];
            b = currentPointCloud[i][5];
            intensity = ((r*256+g)*256+b);
            pointcloud->mutable_point(i)->set_intensity(intensity);
            //对有内容的msg才可以用set(index, value),初始化只能用add
        }
        talker->Write(pointcloud);
        AINFO << "pointCloud_writer sent a message! No. " << j;
        rate.Sleep();
    }
    
    return 0;
}
