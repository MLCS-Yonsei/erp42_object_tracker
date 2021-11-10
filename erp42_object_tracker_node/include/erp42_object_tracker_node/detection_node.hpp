#ifndef ERP42_OBJ_DETECTOR_
#define ERP42_OBJ_DETECTOR_

#include <cmath>
#include <pcl_conversions/pcl_conversions.h>  // pcl::fromROSMsg
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"

#include "common/parameter.hpp"              // common::getSegmenterParams
#include "common/publisher.hpp"              // common::publishCloud
#include "common/time.hpp"                   // common::Clock
#include "common/types/type.h"               // PointICloudPtr
#include "segmenters/segmenter_manager.hpp"  // segmenter::createGroundSegmenter

#include "roi_filters/roi.hpp"  // roi::applyROIFilter

class ObjectDetector
{
public:

    ObjectDetector();

    ~ObjectDetector();

    bool updateParams();

    void spinNode();    

    ros::NodeHandle nh = ros::NodeHandle();
    
private:

    // Node handle
    ros::NodeHandle private_nh = ros::NodeHandle("~");
    ros::AsyncSpinner spiner = ros::AsyncSpinner(1);
    // ROS Publisher & Subscriber
    ros::Publisher pcs_array_segmented_pub_;
    ros::Publisher pcs_segmented_pub_; 
    ros::Subscriber pointcloud_sub_; 
    
    // params
    const std::string param_ns_prefix_ = "detect";  // NOLINT
    std::string frame_id_;                          // NOLINT
    bool use_roi_filter_;
    autosense::ROIParams params_roi_;
    
    /// @note Core components
    boost::shared_ptr<autosense::segmenter::BaseSegmenter> ground_remover_;
    boost::shared_ptr<autosense::segmenter::BaseSegmenter> segmenter_;
    
    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& ros_pc2);

};

#endif // ERP42_OBJ_DETECTOR_