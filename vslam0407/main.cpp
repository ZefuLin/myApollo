#include "mapping.h"
#include "relocation.h"
#include "myOdometry.h"
#include "fileProcess.h"
#include "ekf.h"
#include "odom.h"
#include <yaml-cpp/yaml.h>
#include<vector>

using namespace std;

int main(int argc, char** argv){
	YAML::Node config = YAML::LoadFile("../config/params.yaml");
	
	int pointcloud_file_num = config["pointcloud_file_num"].as<int>();
	int gnss_odom_file_num = config["gnss_odom_file_num"].as<int>();
	string odometryFilePath = config["odometryFilePath"].as<string>();
	string root_source_cloud = config["root_source_cloud"].as<string>();
	string transSrcCloud_path = config["transSrcCloud_path"].as<string>();
	string gnss_odom_trajectory_path = config["gnss_odom_trajectory_path"].as<string>();
	double gnss_pose[gnss_odom_file_num][7];

	std::vector<int> frame_id = config["frame_id"].as<vector<int>>();

	readDataFromIMUOdometry(odometryFilePath, gnss_odom_file_num, gnss_pose);

	int mapping_flag = config["mapping_flag"].as<int>();
	int relocation_flag = config["relocation_flag"].as<int>();
	int odom_flag = config["odom_flag"].as<int>();
	int get_pointCloud_flag = config["get_pointCloud_flag"].as<int>();
	int manual_flag = config["manual_flag"].as<int>();

	if(get_pointCloud_flag){
		string rootPath = "/home/lin/Projects/bev2cloud/data/bevImage/";
		string savePath = "/home/lin/Projects/bev2cloud/data/bevCloud/";
		for(int i=0;i<gnss_odom_file_num;i++){
			stringstream buf;
        	buf << setfill('0') << setw(6) << i;
        	string strfileNum;
        	strfileNum = buf.str();
        	// string filePath = rootPath + to_string(i) + ".jpg";
        	string filePath = rootPath + strfileNum + ".png";
        	// string pcdSavedPath = savePath + to_string(i) + ".pcd";
        	string pcdSavedPath = savePath + strfileNum + ".pcd";
        	calCloudFromBEV(filePath,pcdSavedPath);
   		}
	}

	if(manual_flag){
		vector<int> file_id;
		for (vector<int>::iterator id = frame_id.begin(); id!=frame_id.end(); id++){
			file_id.push_back(*id);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr manual_glo(new pcl::PointCloud<pcl::PointXYZRGB>());
		string manual_glo_cloud_path = "/home/lin/Projects/bev2cloud/data/manual_glo_cloud.pcd";
		for (int i=0; i<file_id.size(); i++){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr manual_src(new pcl::PointCloud<pcl::PointXYZRGB>());
			string manual_src_cloud_path = transSrcCloud_path + to_string(file_id[i]) + ".pcd";
			pcl::io::loadPCDFile<pcl::PointXYZRGB>(manual_src_cloud_path, *manual_src);
			cout << manual_src_cloud_path << " done！" << endl;
			*manual_glo = *manual_glo + *manual_src;
		}
		pcl::io::savePCDFileBinary(manual_glo_cloud_path, *manual_glo);
	}

	if(odom_flag){
		string wheel_odom_path = config["wheel_odom_path"].as<string>();
		string wheel_odom_trajectory_path = config["wheel_odom_trajectory_path"].as<string>();
		string ekf_odom_trajectory_path = config["ekf_odom_trajectory_path"].as<string>();
		
		int wheel_odom_file_num = config["wheel_odom_file_num"].as<int>();

		string imu_odom_path = config["imu_odom_path"].as<string>();
		string imu_odom_trajectory_path = config["imu_odom_trajectory_path"].as<string>();
		int imu_odom_file_num = config["imu_odom_file_num"].as<int>();


		double velocity[wheel_odom_file_num][4];
		double imu[imu_odom_file_num][9];
		double wheelOdomPose[wheel_odom_file_num][7];
		double imuOdomPose[imu_odom_file_num][7];
		double ekfpose[wheel_odom_file_num][7];

		readVelocityData(wheel_odom_path, wheel_odom_file_num, velocity);
		// readIMUData(imu_odom_path, imu_odom_file_num, imu);

		wheelOdometry(wheel_odom_file_num, velocity, wheelOdomPose);
		// imuOdometry(imu_odom_file_num, imu, imuOdomPose);
		imuAndWheelOdometry(wheel_odom_file_num, imu, velocity, gnss_pose, ekfpose);

		for(int i=0; i<wheel_odom_file_num; i++){
			cout << "gnss : " << endl;
			cout << "x = " << gnss_pose[i][0] << endl;
			cout << "y = " << gnss_pose[i][1] << endl;
			cout << "z = " << gnss_pose[i][2] << endl;
            std::vector<double> gnss_pose_euler =  QuaternionToEuler(
															gnss_pose[i][6],
															gnss_pose[i][3], 
															gnss_pose[i][4], 
															gnss_pose[i][5]);
			cout << "gnss_pose_euler yaw = " << gnss_pose_euler[2] << endl;

			cout << "ekf : " << endl;
			cout << "x = " << ekfpose[i][0] << endl;
			cout << "y = " << ekfpose[i][1] << endl;
			cout << "z = " << ekfpose[i][2] << endl;
			std::vector<double> ekf_pose_euler =  QuaternionToEuler(
															ekfpose[i][6],
															ekfpose[i][3], 
															ekfpose[i][4], 
															ekfpose[i][5]);
			cout << "ekf_pose_euler yaw = " << ekf_pose_euler[2] << endl;
		}
		

		if(mapping_flag) mapping(ekfpose);
		// if(mapping_flag) mapping(gnss_pose);
		// if(relocation_flag) relocation(ekfpose);

		drawOdometryTrajectory(wheel_odom_trajectory_path, wheelOdomPose, wheel_odom_file_num);
		// drawOdometryTrajectory(imu_odom_trajectory_path, imuOdomPose, imu_odom_file_num);
		drawOdometryTrajectory(gnss_odom_trajectory_path, gnss_pose, imu_odom_file_num);
		drawOdometryTrajectory(ekf_odom_trajectory_path, ekfpose, wheel_odom_file_num);
	}
	
	return 0;
}