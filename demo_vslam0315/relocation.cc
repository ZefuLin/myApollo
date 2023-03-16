#include "cyber/demo_vslam/relocation.h"

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

void relocation(double pose[7], double pointCloud[3000][6],
             int &seq_glo,int &seq_src,
             double &lastPoseX,
             double &lastPoseY,
             double &lastPoseZ,
             Eigen::Affine3d& inverseMatrix,
             Eigen::Affine3d imu2lidar,
             int &keyframe,
             std::string mapping_result_path,
             std::string source_cloud_path)
{
    // string cur_source_cloud_path = source_cloud_path + to_string(seq_src) + ".pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapping_result(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(mapping_result_path, *mapping_result);
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ego_point(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointXYZRGB po;
	po.x = 0;
	po.x = 0;
	po.z = 0;
	po.r=0;
    po.g=0;
    po.b=0;
	ego_point->push_back(po);

    Eigen::Affine3d scr2global = t * q.toRotationMatrix();
    scr2global = scr2global*imu2lidar;
    scr2global = inverseMatrix * scr2global;
    *source_cloud =*myTransformPointCloud(source_cloud,scr2global);
    *ego_point = *myTransformPointCloud(ego_point,scr2global);

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

    cout << "------------------- frame " << seq_src << "-------------------" << endl;
    if (icp.hasConverged() == false) {
			cout << "ICP can not converge, use imu odometry data: " << icp.getFitnessScore() << endl;
			broadcastCurrentPose(ego_point);
		}
		else{
			Eigen::Affine3d transWorldCurrent;
			transWorldCurrent = icp.getFinalTransformation().cast<double>();
			*ego_point = *myTransformPointCloud(ego_point,transWorldCurrent);
			broadcastCurrentPose(ego_point);
		}
}