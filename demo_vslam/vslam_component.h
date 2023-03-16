#include <string>
#include <memory>
#include <vector>
#include "cyber/component/component.h"
#include "cyber/demo_vslam/proto/vslam.pb.h"
#include "cyber/demo_vslam/mapping.h"

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"

#include "yaml-cpp/yaml.h"

using apollo::cyber::Component;
using apollo::cyber::demo_vslam::proto::CarPose;
using apollo::cyber::demo_vslam::proto::pointCloud;

class vslamComponent : public Component<CarPose, pointCloud> {
    public:
        bool Init() override;
        bool Proc(const std::shared_ptr<CarPose>& car_pose,
            const std::shared_ptr<pointCloud>& point_cloud) override; 
    private:
        int keyframe;
        int seq_glo;
        int seq_src;
        Eigen::Affine3d inverseMatrix = Eigen::Affine3d::Identity(); 
        Eigen::Affine3d imu2lidar = Eigen::Affine3d::Identity(); 
        std::string global_cloud_path;
        std::string source_cloud_path;
        double lastPoseX;
        double lastPoseY;
        double lastPoseZ;
};
CYBER_REGISTER_COMPONENT(vslamComponent)

