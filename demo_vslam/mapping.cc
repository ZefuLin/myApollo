#include "cyber/demo_vslam/mapping.h"

using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr myTransformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn, Eigen::Affine3d& transCur){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    for (int i = 0; i < cloudSize; ++i){
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].r = pointFrom.r;
        cloudOut->points[i].g = pointFrom.g;
        cloudOut->points[i].b = pointFrom.b;
        }
    return cloudOut;
}

void mapping(double pose[7], double pointCloud[3000][6],
             int &seq_glo, int &seq_src,
             double &lastPoseX,
             double &lastPoseY,
             double &lastPoseZ,
             Eigen::Affine3d& inverseMatrix,
             Eigen::Affine3d imu2lidar,
             int &keyframe,
             string global_cloud_path,
             string source_cloud_path
             ){
    
    string cur_global_cloud_path = global_cloud_path + "glo" + to_string(seq_glo) + ".pcd";
    string tar_global_cloud_path = global_cloud_path + "glo" + to_string(seq_src) + ".pcd";
    // string cur_source_cloud_path = source_cloud_path + to_string(seq_src) + ".pcd";

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(cur_global_cloud_path, *global_cloud);
    // pcl::io::loadPCDFile<pcl::PointXYZRGB>(cur_source_cloud_path, *source_cloud);

    for (int i=0; i<3000; i++){
        pcl::PointXYZRGB po;
        po.x = pointCloud[i][0];
        po.y = pointCloud[i][1];
        po.z = pointCloud[i][2];
        po.r = pointCloud[i][3];
        po.g = pointCloud[i][4];
        po.b = pointCloud[i][5];
        source_cloud->push_back(po);
    }

    Eigen::Translation3d t;
    t.x() = pose[0];
	t.y() = pose[1];
	t.z() = pose[2];
    Eigen::Quaterniond q;
	q.x() = pose[3];
	q.y() = pose[4];
	q.z() = pose[5];
	q.w() = pose[6];
	
    Eigen::Affine3d scr2global = t * q.toRotationMatrix();
    scr2global = scr2global*imu2lidar;
    scr2global = inverseMatrix * scr2global;
    *source_cloud =*myTransformPointCloud(source_cloud,scr2global);
    
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transSrcToGlobal(new pcl::PointCloud<pcl::PointXYZRGB>());
	double icp_max_correspondence_distance = 0.3;
    int icp_maximum_iterations_mapping = 1000;
    double icp_transformation_epsilon = 1e-10;
    double icp_euclidean_fitness_epsilon = 1e-6;

    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance);
    icp.setMaximumIterations(icp_maximum_iterations_mapping);
    icp.setTransformationEpsilon(icp_transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(icp_euclidean_fitness_epsilon);
	icp.setInputSource(source_cloud);
	icp.setInputTarget(global_cloud);
	icp.align(*transSrcToGlobal);
    
    if (icp.hasConverged() == false) {
        cout << "ICP failed, the score is " << icp.getFitnessScore() << endl;
        keyframe++;
        if(keyframe>5){
				// 5次icp失败就用一次imu的
				*global_cloud = *global_cloud + *source_cloud;
				pcl::io::savePCDFileBinary(tar_global_cloud_path, *global_cloud);
				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
				cout << tar_global_cloud_path << ": done (imu)!" << endl;
				cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
				keyframe=0;
                seq_glo = seq_src;
			}
    }else{
        cout << "icp success " << seq_src << endl;
        Eigen::Affine3d transWorldCurrent;
		transWorldCurrent = icp.getFinalTransformation().cast<double>();
        double currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw;
		pcl::getTranslationAndEulerAngles(transWorldCurrent,currentX,currentY,currentZ,currentRoll,currentPitch,currentYaw);
		double dis=(currentX-lastPoseX)*(currentX-lastPoseX)+(currentY-lastPoseY)*(currentY-lastPoseY);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentFeatureCloudInWorld(new pcl::PointCloud<pcl::PointXYZRGB>());
		*currentFeatureCloudInWorld =*myTransformPointCloud(source_cloud, transWorldCurrent);
        cout << "distance " << dis << endl;
		if((dis>0.5)){
			*global_cloud = *global_cloud + *currentFeatureCloudInWorld;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_downsampling(new pcl::PointCloud<pcl::PointXYZRGB>());
       	    pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilter;
			downSizeFilter.setInputCloud(global_cloud);
        	downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
        	downSizeFilter.filter(*global_downsampling);
        	*global_cloud=*global_downsampling;
            
			pcl::io::savePCDFileBinary(tar_global_cloud_path, *global_cloud);
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
			cout << tar_global_cloud_path << ":done!" << endl;
			cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" << endl;
			keyframe=0;
            seq_glo = seq_src;
		}
		lastPoseX=currentX;
        lastPoseY=currentY;
       	lastPoseZ=currentZ;
    }
    seq_src++;
}
