#include "fileProcess.h"
#include "relocation.h"

using namespace std;

void broadcastCurrentPose(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn){
	int cloudSize = cloudIn->size();
	double x=0,y=0,z=0;
	if(cloudSize==1){
		const auto &pointFrom = cloudIn->points[0];
		x = pointFrom.x;
		y = pointFrom.y;
		z = pointFrom.z;
	}else{
		cout << "point num error!" << endl;
	}
	cout << "The ego car position is:" << endl;
	cout << "x = " << x << endl;
	cout << "y = " << y << endl;
	cout << "z = " << z << endl;
}

void relocation(double pose[][7]){
/*
	mapping之后要加的东西:
		读最后一个global点云
		读当前新生成的current点云
		icp
			迭代次数10次就行
		输出pose
			xyz roll pitch yaw
*/ 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ego_global(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::visualization::PCLVisualizer viewer("viewer");
	viewer.addPointCloud(ego_global, "ego_global");

	YAML::Node config = YAML::LoadFile("../config/params.yaml");
	string odometryFilePath = config["odometryFilePath"].as<string>();
	string root_source_cloud = config["root_source_cloud"].as<string>();
	string icp_result = config["icp_result"].as<string>();
	string init_global_cloud_path = config["init_global_cloud"].as<string>();
	int imu_file_num = config["imu_file_num"].as<int>();

	// 读全局点云
	string final_global_cloud_path = icp_result + "glo919.pcd";
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::io::loadPCDFile<pcl::PointXYZRGB>(final_global_cloud_path, *final_global_cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGB>());

	int glo_size = final_global_cloud->size();
	tmp->resize(glo_size);
	for (int i = 0; i < glo_size; ++i)
	{
		const auto &pointFrom = final_global_cloud->points[i];
		tmp->points[i].x = double(pointFrom.x);
		tmp->points[i].y = double(pointFrom.y);
		tmp->points[i].z = double(pointFrom.z);
		tmp->points[i].r = 255;
		tmp->points[i].g = 255;
		tmp->points[i].b = 255;

	}
	*ego_global = *ego_global + *tmp;

	double quaterniondNum[imu_file_num][4];
	double translation3dNum[imu_file_num][3];
	
	for(int i=0;i<imu_file_num;i++){
		translation3dNum[i][0] = pose[i][0];
		translation3dNum[i][1] = pose[i][1];
		translation3dNum[i][2] = pose[i][2];	
		
		for(int k=0;k<=3;k++){
			quaterniondNum[i][k] = pose[i][k+3];
		}
	}

	// 根据imu数据对current cloud做初步变换
	std::vector<double> imu2lidar_matrix = config["imu2lidar_matrix"].as<vector<double>>();
	Eigen::Affine3d imu2lidar;
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

	//根据第一帧的全局cloud得到inverse Matrix
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr init_global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB>(init_global_cloud_path, *init_global_cloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpGloCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	int cloudSize = init_global_cloud->size();
	tmpGloCloud->resize(cloudSize);
	for (int i = 0; i < cloudSize; ++i)
	{
		const auto &pointFrom = init_global_cloud->points[i];
		tmpGloCloud->points[i].x = double(pointFrom.x*0.004);
		tmpGloCloud->points[i].y = double(pointFrom.y*0.004);
		tmpGloCloud->points[i].z = double(pointFrom.z*0.004);
		tmpGloCloud->points[i].r = pointFrom.r;
		tmpGloCloud->points[i].g = pointFrom.g;
		tmpGloCloud->points[i].b = pointFrom.b;

	}
	*init_global_cloud = *tmpGloCloud;
	
	// 现在处理当前需要定位的帧
	// 这里之后应该改成根据实时输入的数据做icp
	// 数据包括 点云 和 对应的odometry数据
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ego_car_glo(new pcl::PointCloud<pcl::PointXYZRGB>());
	for(int j=1; j<imu_file_num; j++){
		// ego车在(0,0,0)点，求它的位置的时候只用这个点乘两次trans矩阵就行
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ego_point(new pcl::PointCloud<pcl::PointXYZRGB>());
		pcl::PointXYZRGB po;
		po.x = 0;
		po.x = 0;
		po.z = 0;
		po.r=255;
    	po.g=255;
    	po.b=255;
		ego_point->push_back(po);

		Eigen::Quaterniond q_init;
		q_init.x() = quaterniondNum[0][0];
		q_init.y() = quaterniondNum[0][1];
		q_init.z() = quaterniondNum[0][2];
		q_init.w() = quaterniondNum[0][3];
		double t_x_init = translation3dNum[0][0];
		double t_y_init = translation3dNum[0][1];
		double t_z_init = translation3dNum[0][2];

		Eigen::Translation3d t_init(t_x_init,t_y_init,t_z_init);
		Eigen::Affine3d scr2global_init = t_init * q_init.toRotationMatrix()*imu2lidar;
		Eigen::Affine3d inverseMatrix = scr2global_init.inverse();

		string source_cloud_path = root_source_cloud + to_string(j) + ".pcd";
		pcl::io::loadPCDFile<pcl::PointXYZRGB>(source_cloud_path, *source_cloud);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpSrcCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
		cloudSize = source_cloud->size();
		
		tmpSrcCloud->resize(cloudSize);
		for (int i = 0; i < cloudSize; ++i)
		{
			const auto &pointFrom = source_cloud->points[i];
			tmpSrcCloud->points[i].x = double(pointFrom.x*0.004);
			tmpSrcCloud->points[i].y = double(pointFrom.y*0.004);
			tmpSrcCloud->points[i].z = double(pointFrom.z*0.004);
			tmpSrcCloud->points[i].r = pointFrom.r;
			tmpSrcCloud->points[i].g = pointFrom.g;
			tmpSrcCloud->points[i].b = pointFrom.b;
		}
		*source_cloud = *tmpSrcCloud;

		Eigen::Quaterniond q;
		q.x() = quaterniondNum[j][0];
		q.y() = quaterniondNum[j][1];
		q.z() = quaterniondNum[j][2];
		q.w() = quaterniondNum[j][3];
		double t_x = translation3dNum[j][0];
		double t_y = translation3dNum[j][1];
		double t_z = translation3dNum[j][2];
		Eigen::Translation3d t(t_x,t_y,t_z);
		Eigen::Affine3d scr2global = t * q.toRotationMatrix();
		scr2global = scr2global*imu2lidar;
		scr2global = inverseMatrix * scr2global;
		*source_cloud =*myTransformPointCloud(source_cloud,scr2global);
		*ego_point = *myTransformPointCloud(ego_point,scr2global);
	
		cout << "-----------------------------frame " << j << "-----------------------------" << endl;
		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transSrcToGlobal(new pcl::PointCloud<pcl::PointXYZRGB>());
		icp.setMaxCorrespondenceDistance(0.3);
    	icp.setMaximumIterations(10);
    	icp.setTransformationEpsilon(1e-10);
   		icp.setEuclideanFitnessEpsilon(1e-6);
		icp.setInputSource(source_cloud);
		icp.setInputTarget(final_global_cloud);
		icp.align(*transSrcToGlobal);

		if (icp.hasConverged() == false) {
			cout << "ICP can not converge, use imu odometry data: " << icp.getFitnessScore() << endl;
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_downsampling(new pcl::PointCloud<pcl::PointXYZRGB>());
       	    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
			downSizeFilter.setInputCloud(ego_global);
        	downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        	downSizeFilter.filter(*global_downsampling);
			*ego_global = *global_downsampling;
			*ego_global = *ego_global + *ego_point;
			*ego_global = *ego_global + *source_cloud;
			*ego_car_glo = *ego_car_glo + *ego_point;
			broadcastCurrentPose(ego_point);
		}
		else{
			Eigen::Affine3d transWorldCurrent;
			transWorldCurrent = icp.getFinalTransformation().cast<double>();
			*ego_point = *myTransformPointCloud(ego_point,transWorldCurrent);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_downsampling(new pcl::PointCloud<pcl::PointXYZRGB>());
       	    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
			downSizeFilter.setInputCloud(ego_global);
        	downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        	downSizeFilter.filter(*global_downsampling);
			*ego_global = *global_downsampling;
			*ego_global = *ego_global + *ego_point;
			*ego_global = *ego_global + *transSrcToGlobal; 
			*ego_car_glo = *ego_car_glo + *ego_point;
			broadcastCurrentPose(ego_point);
		}
		
		string ego_global_path = config["ego_global_path"].as<string>();
		string ego_trajectory_path = config["ego_trajectory_path"].as<string>();
		
		pcl::io::savePCDFileBinary(ego_global_path, *ego_global);
		pcl::io::savePCDFileBinary(ego_trajectory_path, *ego_car_glo);
		viewer.updatePointCloud(ego_global, "ego_global");
		viewer.spinOnce(0.001);
		sleep(0.1);
	}//for(int j=1; j<imu_file_num; j++){
}//relocation 

