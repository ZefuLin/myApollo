#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include "cyber/demo_vslam/fileProcess.h"

void readDataFromIMUOdometry(string path, int fileNum, double pose[][7]){
    for(int i=0;i<fileNum;i++){
        ifstream infile;
        stringstream buf;
        buf << setfill('0') << setw(6) << i;
        string strfileNum;
        strfileNum = buf.str();
        string filePath = path + strfileNum + ".txt";
        infile.open(filePath, ios::in);
        if(!infile){
            cout << "file does not exist!!" << endl;
			cout << filePath << endl;
            exit(1);
        }
        char line[1000];
		int j=0;
		while (infile.getline(line,100,' '))
		{	
			string num = line;
			double a;
			stringstream ss;
			ss << num;
			ss >> a;
			pose[i][j] = a;
			j++;
		}
	   infile.close();
    }
    YAML::Node node;
    ofstream writeYAML("/apollo/cyber/demo_vslam/conf/initPose.yaml");
    node["init_pose"]["x"] = pose[0][0];
    node["init_pose"]["y"] = pose[0][1];
    node["init_pose"]["z"] = pose[0][2];
    node["init_pose"]["qx"] = pose[0][3];
    node["init_pose"]["qy"] = pose[0][4];
    node["init_pose"]["qz"] = pose[0][5];
    node["init_pose"]["qw"] = pose[0][6];
    writeYAML << node << endl;
    writeYAML.close();
}

void readDataFeomBEVCloud(string path, double pointCloud[][6]){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *currentCloud);

    int cloudSize = currentCloud->size();
    for (int i=0; i<cloudSize;i++){
        const auto &pointFrom = currentCloud->points[i];
        pointCloud[i][0] = double(pointFrom.x*0.004);
        pointCloud[i][1] = double(pointFrom.y*0.004);
        pointCloud[i][2] = double(pointFrom.z*0.004);

        pointCloud[i][3] = pointFrom.r;
        pointCloud[i][4] = pointFrom.g;
        pointCloud[i][5] = pointFrom.b;
    }
}
