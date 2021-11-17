#ifndef ERP42_OBJ_TRACKER_
#define ERP42_OBJ_TRACKER_

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <memory>
// #include <Eigen/Dense>
// #include <Eigen/Core>

#include "common/msgs/autosense_msgs/PointCloud2Array.h"
#include "common/msgs/autosense_msgs/TrackingFixedTrajectoryArray.h"
#include "common/msgs/autosense_msgs/TrackingObjectArray.h"

#include "common/bounding_box.hpp"
#include "common/color.hpp"
#include "common/parameter.hpp"
#include "common/publisher.hpp"
#include "common/time.hpp"
#include "common/transform.hpp"
#include "common/types/object.hpp"
#include "common/types/type.h"
#include "object_builders/object_builder_manager.hpp"
#include "tracking/tracking_worker_manager.hpp"

// IHGP
#include "ihgp/Matern32model.hpp"
#include "ihgp/InfiniteHorizonGP.hpp"

class ObjectTracker
{
public:

    ObjectTracker(){};

    ~ObjectTracker(){};

    bool updateParams();

    void spinNode();    

    ros::NodeHandle nh = ros::NodeHandle();
    
private:

    // Node handle
    ros::NodeHandle private_nh = ros::NodeHandle("~");
    ros::AsyncSpinner spiner = ros::AsyncSpinner(1);
    // ROS Subscriber
    ros::Subscriber pcs_segmented_sub_;
    tf::TransformListener tf_listener_;
    // std::unique_ptr<tf::TransformListener> tf_listener_;
    // ROS Publisher
    ros::Publisher marker_pub; // obstacle pose visualization 
    ros::Publisher pose_pub;
    

    //***************************************
    // params
    const std::string param_ns_prefix_ = "tracking";  // NOLINT
    std::string local_frame_id_, global_frame_id_;    // NOLINT
    int tf_timeout_ms_ = 0;
    double threshold_contian_IoU_ = 0.0;
    autosense::TrackingWorkerParams tracking_params_;
    std::unique_ptr<autosense::object_builder::BaseObjectBuilder> object_builder_ = nullptr;

    //***************************************
    int first_frame = true;
    std::vector<int> objIDs; // obj lists
    std::vector<std::vector<pcl::PointXYZI>> stack_obj; // t~(t-10) cluster Centers stack
    std::vector<std_msgs::ColorRGBA> colorset; // rviz msg colorset

    // IHGP state space model
    int next_obj_num = 0;
    int spin_counter = 0;
    std::vector<InfiniteHorizonGP*> GPs_x;
    std::vector<InfiniteHorizonGP*> GPs_y;
    std::vector<tf::StampedTransform> ego_tf_stack;
    
    float frequency = 10; // node frequency
    float dt_gp = 1/frequency; 
    float id_threshold;
    int data_length = 10;

    float lpf_tau; 
    double logSigma2_x;
    double logMagnSigma2_x;
    double logLengthScale_x;

    double logSigma2_y;
    double logMagnSigma2_y;
    double logLengthScale_y;

    //***************************************


    void pointcloudCallback(const autosense_msgs::PointCloud2ArrayConstPtr &segments_msg);

    void registerNewObstacle(pcl::PointXYZI centroid);

    void unregisterOldObstacle(double now);

    void updateObstacleQueue(const int i, pcl::PointXYZI centroid);

    void fill_with_linear_interpolation(const int i, pcl::PointXYZI centroid);

    float euc_dist(Eigen::Vector3d P1, Eigen::Vector3d P2);

    std::vector<std::vector<pcl::PointXYZI>> callIHGP(std::vector<int> this_objIDs);

    pcl::PointXYZI LPF_pos(std::vector<pcl::PointXYZI> centroids);

    pcl::PointXYZI IHGP_fixed_pos(std::vector<pcl::PointXYZI> centroids, int n);

    pcl::PointXYZI IHGP_fixed_vel(std::vector<pcl::PointXYZI> centroids, int n);

    void publishMarkers(std::vector<std::vector<pcl::PointXYZI>> pos_vel_s, \
                        std::string frame_id, \
                        std::vector<int> this_objIDs);

};

#endif // ERP42_OBJ_TRACKER_