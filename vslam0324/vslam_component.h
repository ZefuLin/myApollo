#include <string>
#include <memory>
#include <vector>
#include "cyber/component/component.h"
#include "modules/localization/vslam/proto/vslam.pb.h"
#include "modules/localization/vslam/mapping.h"
#include "modules/localization/vslam/relocation.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"

#include "yaml-cpp/yaml.h"

using apollo::cyber::Component;
using apollo::drivers::PointCloud;
using apollo::localization::Gps;

class vslamComponent : public Component<Gps, PointCloud> {
    public:
        bool Init() override;
        bool Proc(const std::shared_ptr<Gps>& car_pose,
            const std::shared_ptr<PointCloud>& point_cloud) override; 
    private:
        int keyframe;
        int seq_glo;
        int seq_src;
        Eigen::Affine3d inverseMatrix = Eigen::Affine3d::Identity(); 
        Eigen::Affine3d imu2lidar = Eigen::Affine3d::Identity(); 
        std::string global_cloud_path;
        std::string source_cloud_path;
        std::string mapping_result_path;
        double lastPoseX;
        double lastPoseY;
        double lastPoseZ;
};
CYBER_REGISTER_COMPONENT(vslamComponent)

