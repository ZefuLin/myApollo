//#include "fileProcess.h"
//#include "mapping.h"
//
//using namespace std;
//
//double lastPoseX=0;
//double lastPoseY=0;
//double lastPoseZ=0;
//double lastPoseRoll=0;
//double lastPosePitch=0;
//double lastPoseYaw=0;
//
//double currentX=0;
//double currentY=0;
//double currentZ=0;
//double currentRoll=0;
//double currentPitch=0;
//double currentYaw=0;
//int keyframe;
//
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr myTransformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, Eigen::Affine3d& transCur){
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());
//
//    int cloudSize = cloudIn->size();
//    cloudOut->resize(cloudSize);
//
//    for (int i = 0; i < cloudSize; ++i){
//        const auto &pointFrom = cloudIn->points[i];
//        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
//        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
//        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
//        cloudOut->points[i].r = pointFrom.r;
//        cloudOut->points[i].g = pointFrom.g;
//        cloudOut->points[i].b = pointFrom.b;
//        }
//    return cloudOut;
//}
//
//void mapping(double pose[][7]){
//	YAML::Node config = YAML::LoadFile("../config/params.yaml");
//	std::vector<double> imu2lidar_matrix = config["imu2lidar_matrix"].as<vector<double>>();
//	string root_source_cloud = config["root_source_cloud"].as<string>();
//	string init_global_cloud = config["init_global_cloud"].as<string>();
//	string result = config["icp_result"].as<string>();
//	int imu_file_num = config["imu_file_num"].as<int>();
//	int random_downsample_num = config["random_downsample_num"].as<int>();
//	int icp_maximum_iterations_mapping = config["icp_maximum_iterations_mapping"].as<int>();
//	float voxel_downsample_leaf_size = config["voxel_downsample_leaf_size"].as<float>();
//	float icp_max_correspondence_distance = config["icp_max_correspondence_distance"].as<float>();
//	double icp_transformation_epsilon = config["icp_transformation_epsilon"].as<double>();
//	double icp_euclidean_fitness_epsilon = config["icp_euclidean_fitness_epsilon"].as<double>();
//	double key_frame_dis = config["key_frame_dis"].as<double>();
//	int key_frame_id = 0;
//	Eigen::Affine3d trans_key2glo;
//	double quaterniondNum[imu_file_num][4];
//	double translation3dNum[imu_file_num][3];
//
//	for(int i=0;i<imu_file_num;i++){
//		translation3dNum[i][0] = pose[i][0];
//		translation3dNum[i][1] = pose[i][1];
//		translation3dNum[i][2] = pose[i][2];
//
//		for(int k=0;k<=3;k++){
//			quaterniondNum[i][k] = pose[i][k+3];
//		}
//	}
//
//	Eigen::Affine3d imu2lidar;
//	imu2lidar(0,0) = imu2lidar_matrix[0];
//	imu2lidar(0,1) = imu2lidar_matrix[1];
//	imu2lidar(0,2) = imu2lidar_matrix[2];
//	imu2lidar(1,0) = imu2lidar_matrix[3];
//	imu2lidar(1,1) = imu2lidar_matrix[4];
//	imu2lidar(1,2) = imu2lidar_matrix[5];
//	imu2lidar(2,0) = imu2lidar_matrix[6];
//	imu2lidar(2,1) = imu2lidar_matrix[7];
//	imu2lidar(2,2) = imu2lidar_matrix[8];
//	imu2lidar(0,3) = imu2lidar_matrix[9];
//	imu2lidar(1,3) = imu2lidar_matrix[10];
//	imu2lidar(2,3) = imu2lidar_matrix[11];
//
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpGloCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//	pcl::io::loadPCDFile<pcl::PointXYZRGB>(init_global_cloud, *global_cloud);
//
//	int cloudSize = global_cloud->size();
//	tmpGloCloud->resize(cloudSize);
//
//	for (int i = 0; i < cloudSize; ++i)
//	{
//		const auto &pointFrom = global_cloud->points[i];
//		tmpGloCloud->points[i].x = double(pointFrom.x*0.004);
//		tmpGloCloud->points[i].y = double(pointFrom.y*0.004);
//		tmpGloCloud->points[i].z = double(pointFrom.z*0.004);
//		tmpGloCloud->points[i].r = pointFrom.r;
//		tmpGloCloud->points[i].g = pointFrom.g;
//		tmpGloCloud->points[i].b = pointFrom.b;
//
//	}
//	*global_cloud = *tmpGloCloud;
//
//	Eigen::Quaterniond q_init;
//	q_init.x() = quaterniondNum[0][0];
//	q_init.y() = quaterniondNum[0][1];
//	q_init.z() = quaterniondNum[0][2];
//	q_init.w() = quaterniondNum[0][3];
//	double t_x_init = translation3dNum[0][0];
//	double t_y_init = translation3dNum[0][1];
//	double t_z_init = translation3dNum[0][2];
//
//	Eigen::Translation3d t_init(t_x_init,t_y_init,t_z_init);
//	Eigen::Affine3d scr2global_init = t_init * q_init.toRotationMatrix()*imu2lidar;
//	Eigen::Affine3d inverseMatrix = scr2global_init.inverse();
//
//	clock_t start,end;
//	start = clock();
//
//	for(int j=1;j<imu_file_num;j++){
//		string source_cloud_path = root_source_cloud + to_string(j) + ".pcd";
//		string global_cloud_path = result + "glo" + to_string(j) + ".pcd";
//		// string key_frame_path = root_source_cloud + to_string(key_frame_id) + ".pcd";
//		string key_frame_path = result + "glo" + to_string(key_frame_id) + ".pcd";
//
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr key_frame_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//		pcl::io::loadPCDFile<pcl::PointXYZRGB>(source_cloud_path, *source_cloud);
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpSrcCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
//		cloudSize = source_cloud->size();
//		tmpSrcCloud->resize(cloudSize);
//		for (int i = 0; i < cloudSize; ++i)
//		{
//			const auto &pointFrom = source_cloud->points[i];
//			tmpSrcCloud->points[i].x = double(pointFrom.x*0.004);
//			tmpSrcCloud->points[i].y = double(pointFrom.y*0.004);
//			tmpSrcCloud->points[i].z = double(pointFrom.z*0.004);
//			tmpSrcCloud->points[i].r = pointFrom.r;
//			tmpSrcCloud->points[i].g = pointFrom.g;
//			tmpSrcCloud->points[i].b = pointFrom.b;
//		}
//		*source_cloud = *tmpSrcCloud;
//
//		Eigen::Quaterniond q_key_frame;
//		q_key_frame.x() = quaterniondNum[key_frame_id][0];
//		q_key_frame.y() = quaterniondNum[key_frame_id][1];
//		q_key_frame.z() = quaterniondNum[key_frame_id][2];
//		q_key_frame.w() = quaterniondNum[key_frame_id][3];
//		double t_x_key_frame = translation3dNum[key_frame_id][0];
//		double t_y_key_frame = translation3dNum[key_frame_id][1];
//		double t_z_key_frame = translation3dNum[key_frame_id][2];
//		Eigen::Translation3d t_key_frame(t_x_key_frame,t_y_key_frame,t_z_key_frame);
//		Eigen::Affine3d scr2global_key_frame = t_key_frame * q_key_frame.toRotationMatrix()*imu2lidar;
//		Eigen::Affine3d inverseMatrix_key_frame = scr2global_key_frame.inverse();
//
//		Eigen::Quaterniond q;
//		q.x() = quaterniondNum[j][0];
//		q.y() = quaterniondNum[j][1];
//		q.z() = quaterniondNum[j][2];
//		q.w() = quaterniondNum[j][3];
//		double t_x = translation3dNum[j][0];
//		double t_y = translation3dNum[j][1];
//		double t_z = translation3dNum[j][2];
//		Eigen::Translation3d t(t_x,t_y,t_z);
//		Eigen::Affine3d scr2global = t * q.toRotationMatrix();
//		scr2global = scr2global * imu2lidar;
//
//		pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
//		icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
//    	icp.setMaximumIterations(icp_maximum_iterations_mapping);
//    	icp.setTransformationEpsilon(icp_transformation_epsilon);
//    	icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon);
//
//		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transSrcToGlobal(new pcl::PointCloud<pcl::PointXYZRGB>());
//
//		if(key_frame_id>1110){
//			pcl::io::loadPCDFile<pcl::PointXYZRGB>(key_frame_path, *key_frame_cloud);
//			// scr2global = inverseMatrix * scr2global;
//			scr2global = inverseMatrix_key_frame * scr2global;
//			*source_cloud =*myTransformPointCloud(source_cloud,scr2global);
//			icp.setInputSource(source_cloud);
//			icp.setInputTarget(key_frame_cloud);
//			icp.align(*transSrcToGlobal);
//		}else{
//			scr2global = inverseMatrix * scr2global;
//			*source_cloud =*myTransformPointCloud(source_cloud,scr2global);
//			icp.setInputSource(source_cloud);
//			icp.setInputTarget(global_cloud);
//			icp.align(*transSrcToGlobal);
//		}
//
//		if (icp.hasConverged() == false) {
//			cout << "ICP failed, the score is " << icp.getFitnessScore() << endl;
//			cout << "Frame No." << j << " failed to finish ICP" << endl;
//			keyframe++;
//			if(keyframe>5){
//				// 5次icp失败就用一次imu的
//				*global_cloud = *global_cloud + *source_cloud;
//				pcl::io::savePCDFileBinary(global_cloud_path, *global_cloud);
//				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
//				cout << global_cloud_path << ": done (imu)!" << endl;
//				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
//				keyframe=0;
//				key_frame_id = j;
//			}
//        }//if (icp.hasConverged() == false) {
//		else{
//			Eigen::Affine3d transWorldCurrent;
//			transWorldCurrent = icp.getFinalTransformation().cast<double>();
//			pcl::getTranslationAndEulerAngles(transWorldCurrent,currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);
//			double dis=(currentX-lastPoseX)*(currentX-lastPoseX)+(currentY-lastPoseY)*(currentY-lastPoseY);
//			pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentFeatureCloudInWorld(new pcl::PointCloud<pcl::PointXYZRGB>());
//			*currentFeatureCloudInWorld =*myTransformPointCloud(source_cloud, transWorldCurrent);
//
//			cout << "mapping success:" << j <<  endl;
//			if((dis>key_frame_dis)){
//				*global_cloud = *global_cloud + *currentFeatureCloudInWorld;
//				pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_downsampling(new pcl::PointCloud<pcl::PointXYZRGB>());
//       	    	pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
//				downSizeFilter.setInputCloud(global_cloud);
//        		downSizeFilter.setLeafSize(voxel_downsample_leaf_size, voxel_downsample_leaf_size, voxel_downsample_leaf_size);
//        		downSizeFilter.filter(*global_downsampling);
//        		*global_cloud=*global_downsampling;
//				pcl::io::savePCDFileBinary(global_cloud_path, *global_cloud);
//				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
//				cout << global_cloud_path << ":done!" << endl;
//				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
//				keyframe=0;
//				key_frame_id = j;
//				trans_key2glo = transWorldCurrent;
//			}//if((dis>odomKeyFramDisThresh)){
//		lastPoseX=currentX;
//        lastPoseY=currentY;
//       	lastPoseZ=currentZ;
//
//		}//else
//	}//for(int j=0; j<=imu_file_num;j++){
//	end = clock();
//	cout << "time cost "<< (double)(end - start)/ CLOCKS_PER_SEC << "s"<< endl;
//}//mapping
//
