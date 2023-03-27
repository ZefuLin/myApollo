#include <string>
#include <memory>
#include <vector>
#include "cyber/component/component.h"
#include "modules/localization/vslam/proto/vslam.pb.h"
#include "modules/localization/vslam/mapping.h"
#include "modules/localization/vslam/relocation.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/localization/proto/gps.pb.h"
#include "modules/localization/vslam/proto/ground_sign_segmentation.pb.h"

#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"

#include "yaml-cpp/yaml.h"


using apollo::cyber::Component;
using apollo::perception::onboard::GroundSignSegmentation;

static const std::vector<std::string> CLASS_MAP = {
  "background", "lanes", "parking lines", "guide signs", "speed bumps", 
  "free space", "obstacles", "no-stop sign", "walls", "person", "bike/rider", 
  "motor", "cars", "self-car",
};

class readBEVCloudComponent : public Component<GroundSignSegmentation> {
    public:
        bool Init() override;
        bool Proc(const std::shared_ptr<GroundSignSegmentation>& out_segment_message) override; 
    private:
    double pointCloudFromBEV[1024][1024];
    int height;
    int width;
        
};
CYBER_REGISTER_COMPONENT(readBEVCloudComponent)
