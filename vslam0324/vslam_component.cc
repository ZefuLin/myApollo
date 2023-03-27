#include "modules/localization/vslam/vslam_component.h"

bool vslamComponent::Init() {
    keyframe=0;
    seq_glo=0;
    seq_src=0;
    YAML::Node config = YAML::LoadFile("/apollo/cyber/demo_vslam/conf/params.yaml");
    source_cloud_path = config["source_cloud_path"].as<std::string>();
    global_cloud_path = config["global_cloud_path"].as<std::string>();
    mapping_result_path = config["mapping_result_path"].as<std::string>();

	std::vector<double> imu2lidar_matrix = config["imu2lidar_matrix"].as<std::vector<double>>();
    imu2lidar(0,0) = imu2lidar_matrix[0];
	imu2lidar(0,1) = imu2lidar_matrix[1];
	imu2lidar(0,2) = imu2lidar_matrix[2];
	imu2lidar(1,0) = imu2lidar_matrix[3];
	imu2lidar(1,1) = imu2lidar_matrix[4];
	imu2lidar(1,2) = imu2lidar_matrix[5];
	imu2lidar(2,0) = imu2lidar_matrix[6];
	imu2lidar(2,1) = imu2lidar_matrix[7];
	imu2lidar(2,2) = imu2lidar_matrix[8]; 
	imu2lidar(0,3) = imu2lidar_matrix[9];
	imu2lidar(1,3) = imu2lidar_matrix[10];
	imu2lidar(2,3) = imu2lidar_matrix[11];

    YAML::Node node = YAML::LoadFile("/apollo/cyber/demo_vslam/conf/initPose.yaml");
    Eigen::Translation3d t_init;
    t_init.x() = node["init_pose"]["x"].as<double>();
    t_init.y() = node["init_pose"]["y"].as<double>();
    t_init.z() = node["init_pose"]["z"].as<double>();
    Eigen::Quaterniond q_init;
    q_init.x() = node["init_pose"]["qx"].as<double>();
    q_init.y() = node["init_pose"]["qy"].as<double>();
    q_init.z() = node["init_pose"]["qz"].as<double>();
    q_init.w() = node["init_pose"]["qw"].as<double>();
    Eigen::Affine3d tmp = t_init * q_init.toRotationMatrix()*imu2lidar;
    inverseMatrix = tmp.inverse();

    lastPoseX=0;
    lastPoseY=0;
    lastPoseZ=0;

    return true;
}

bool vslamComponent::Proc(const std::shared_ptr<Gps>& car_pose,
                          const std::shared_ptr<PointCloud>& point_cloud) {
    AINFO << "Start vslam Proc!";
    
    double pose[7];
    auto *car_pose_msg = car_pose->mutable_localization();
    
    pose[0] = car_pose_msg->mutable_position()->x();
    pose[1] = car_pose_msg->mutable_position()->y();
    pose[2] = car_pose_msg->mutable_position()->z();
    
    pose[3] = car_pose_msg->mutable_orientation()->qx();
    pose[4] = car_pose_msg->mutable_orientation()->qy();
    pose[5] = car_pose_msg->mutable_orientation()->qz();
    pose[6] = car_pose_msg->mutable_orientation()->qw();
    

    double pointCloud[3000][6];
    for (int i=0; i<3000; i++){
        
        pointCloud[i][0] = point_cloud->mutable_point(i)->x();
        pointCloud[i][1] = point_cloud->mutable_point(i)->y();
        pointCloud[i][2] = point_cloud->mutable_point(i)->z();
        int intensity = point_cloud->mutable_point(i)->intensity();
        pointCloud[i][3] = int(intensity/(256*256)%256);
        pointCloud[i][4] = int(intensity/256%256);
        pointCloud[i][5] = int(intensity%256);
    }
    
    relocation(pose,pointCloud,
            seq_src,
            inverseMatrix,imu2lidar,
            mapping_result_path
           );
    return true;
}