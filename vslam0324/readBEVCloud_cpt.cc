#include "modules/localization/vslam/readBEVCloud_cpt.h"

using apollo::drivers::PointCloud;
using namespace std;

bool readBEVCloudComponent::Init() {
    for (int i=0; i<1024; i++){
        for (int j=0; j<1024; j++){
            pointCloudFromBEV[i][j]=0;
        }
    }
    return true;
}

bool readBEVCloudComponent::Proc(const std::shared_ptr<GroundSignSegmentation>& out_segment_message) {
    //解析传入的地面标识符，转换成点云
    auto* mask_map = out_segment_message->mutable_mask_map();
    height = 1024;
    width = 1024;
    int lanes_num = (*mask_map)["lanes"].position_size();
    for (int i = 0; i < lanes_num; i++){
        int pos = (*mask_map)["lanes"].position(i);
        pointCloudFromBEV[pos/height][pos%width] = 1;   
    }

    int parking_lines_num = (*mask_map)["parking lines"].position_size();
    for (int i = 0; i < parking_lines_num; i++){
        int pos = (*mask_map)["parking lines"].position(i);
        pointCloudFromBEV[pos/height][pos%width] = 1;   
    }
    
    int guide_signs_num = (*mask_map)["guide signs"].position_size();
    for (int i = 0; i < guide_signs_num; i++){
        int pos = (*mask_map)["guide signs"].position(i);
        pointCloudFromBEV[pos/height][pos%width] = 1;   
    }

    apollo::cyber::Init("readBEVCloudComponent");
    auto talker_node = apollo::cyber::CreateNode("pointCloud_publisher");
    auto talker = talker_node->CreateWriter<PointCloud>("channel/pointCloud_from_bev");
    apollo::cyber::Rate rate(1.0);
    auto pointcloud = std::make_shared<PointCloud>();
    int intensity = ((255*256+255)*256+255);
    int point_num = 0;
    for (int row=0; row<height; row++){
        for (int col=0; col<width; col++){
            if (pointCloudFromBEV[row][col]==1){
                pointcloud->add_point();
                pointcloud->mutable_point(point_num)->set_x(row);
                pointcloud->mutable_point(point_num)->set_y(col);
                pointcloud->mutable_point(point_num)->set_z(0);
                pointcloud->mutable_point(point_num)->set_intensity(intensity);
                point_num++;
            }
        }
    }
    talker->Write(pointcloud);
    return true;
}


