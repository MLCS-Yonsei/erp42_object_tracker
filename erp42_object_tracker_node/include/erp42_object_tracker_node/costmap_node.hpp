#ifndef ERP42_OBJ_COSTMAP_
#define ERP42_OBJ_COSTMAP_

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <memory>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"

#include "common/bounding_box.hpp"
#include "common/color.hpp"
#include "common/parameter.hpp"
#include "common/publisher.hpp"
#include "common/time.hpp"
#include "common/transform.hpp"
#include "common/types/object.hpp"
#include "common/types/type.h"
#include "object_builders/object_builder_manager.hpp"

class ObjectCostmap
{
public:

    ObjectCostmap(){};

    ~ObjectCostmap(){};

    bool updateParams();

    void spinNode();    

    ros::NodeHandle nh = ros::NodeHandle();
    
private:

    // Node handle
    ros::NodeHandle private_nh = ros::NodeHandle("~");
    ros::AsyncSpinner spiner = ros::AsyncSpinner(1);
    // RSO Publisher
    ros::Publisher segments_coarse_pub_;
    // ROS Subscriber
    ros::Subscriber pcs_segmented_sub_;

    // core
    std::unique_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_ = nullptr;

    // params
    const std::string param_ns_prefix_ = "costmapconverter";  // NOLINT
    std::string local_frame_id_, global_frame_id_;    // NOLINT

    void pointcloudCallback(const autosense_msgs::PointCloud2ArrayConstPtr &segments_msg);

};

#endif // ERP42_OBJ_COSTMAP_

