#include "erp42_object_tracker_node/costmap_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_node"); 

    ObjectCostmap converter;

    try
    {
        if(!converter.updateParams())
        {
            ROS_ERROR("FAILED TO INITIALIZE %s", ros::this_node::getName().c_str());
            exit(1);
        }
        else
        {
            ROS_INFO("%s INITIALIZED SUCCESSFULLY!", ros::this_node::getName().c_str());
            while (converter.nh.ok()) 
            {
                converter.spinNode();
            }
        }
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("costmap_node: Error occured: %s", e.what());
        exit(1);
    }  

    ros::waitForShutdown();
    ROS_INFO("costmap_node exited...");
}

void ObjectCostmap::spinNode()
{
    spiner.start();
}

bool ObjectCostmap::updateParams()
{
    // Load ROS parameters from rosparam server
    private_nh.getParam(param_ns_prefix_ + "/local_frame_id", local_frame_id_);
    private_nh.getParam(param_ns_prefix_ + "/global_frame_id", global_frame_id_);

    std::string sub_pcs_segmented_topic;
    int sub_pcs_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/sub_pcs_segmented_topic", sub_pcs_segmented_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pcs_queue_size", sub_pcs_queue_size);

    // Init subscribers and publishers
    pcs_segmented_sub_ = nh.subscribe<autosense_msgs::PointCloud2Array>(sub_pcs_segmented_topic, sub_pcs_queue_size, &ObjectCostmap::pointcloudCallback, this);

    return true;
}

void ObjectCostmap::pointcloudCallback(const autosense_msgs::PointCloud2ArrayConstPtr &segments_msg)
{
    // // get msg header data
    // const double kTimeStamp = segments_msg->header.stamp.toSec();
    // std_msgs::Header header;
    // header.frame_id = local_frame_id_;
    // header.stamp = segments_msg->header.stamp;

    // std::vector<autosense::PointICloudPtr> segment_clouds;
    // for (size_t i = 0u; i < segments_msg->clouds.size(); ++i) 
    // {
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     pcl::fromROSMsg(segments_msg->clouds[i], *cloud);
    //     segment_clouds.push_back(cloud);
    // }

}
