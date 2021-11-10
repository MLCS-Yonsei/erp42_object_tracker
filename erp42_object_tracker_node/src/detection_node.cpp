/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */


#include "erp42_object_tracker_node/detection_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "detection_node"); 

    ObjectDetector detector;

    try
    {
        if(!detector.updateParams())
        {
            ROS_ERROR("FAILED TO INITIALIZE %s", ros::this_node::getName().c_str());
            exit(1);
        }
        else
        {
            ROS_INFO("%s INITIALIZED SUCCESSFULLY!", ros::this_node::getName().c_str());
            while (detector.nh.ok()) {
                detector.spinNode();
            }
        }
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("detection_node: Error occured: %s", e.what());
        exit(1);
    }  

    ros::waitForShutdown();
    ROS_INFO("detection_node exited...");
}

ObjectDetector::ObjectDetector()
{
    ;
}

ObjectDetector::~ObjectDetector()
{
    // delete ros parames
    ;
}

void ObjectDetector::spinNode()
{
    spiner.start();
}

bool ObjectDetector::updateParams()
{
    /// @brief Load ROS parameters from rosparam server

    // Subscriber (PointCloud2)
    std::string sub_pc_topic, pub_pcs_array_segmented_topic, pub_pcs_segmented_topic;
    int sub_pc_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/frame_id", frame_id_);
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_topic", sub_pc_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pc_queue_size", sub_pc_queue_size);
    private_nh.getParam(param_ns_prefix_ + "/pub_pcs_array_segmented_topic", pub_pcs_array_segmented_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_pcs_segmented_topic", pub_pcs_segmented_topic);

    /// @note Important to use roi filter for "Ground remover"
    params_roi_ = autosense::common::getRoiParams(private_nh, param_ns_prefix_);

    // Ground remover & non-ground segmenter
    std::string ground_remover_type, non_ground_segmenter_type;
    private_nh.param<std::string>(param_ns_prefix_ + "/ground_remover_type", ground_remover_type, "GroundPlaneFittingSegmenter");
    private_nh.param<std::string>(param_ns_prefix_ + "/non_ground_segmenter_type", non_ground_segmenter_type, "RegionEuclideanSegmenter");

    autosense::SegmenterParams param = autosense::common::getSegmenterParams(private_nh, param_ns_prefix_);
    param.segmenter_type = ground_remover_type;
    ground_remover_ = autosense::segmenter::createGroundSegmenter(param);

    param.segmenter_type = non_ground_segmenter_type;
    segmenter_ = autosense::segmenter::createNonGroundSegmenter(param);

    pcs_array_segmented_pub_ = nh.advertise<autosense_msgs::PointCloud2Array>(pub_pcs_array_segmented_topic, 1);
    pcs_segmented_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_pcs_segmented_topic, 1);
    pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(sub_pc_topic, sub_pc_queue_size, &ObjectDetector::pointcloudCallback, this);

    return true;
}

void ObjectDetector::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& ros_pc2)
{
    autosense::common::Clock clock;

    // Convert sensor_msgs::PointCloud to autosense::PointICloudPtr
    autosense::PointICloudPtr cloud(new autosense::PointICloud);
    pcl::fromROSMsg(*ros_pc2, *cloud);

    std_msgs::Header header = ros_pc2->header;
    header.frame_id = frame_id_;
    header.stamp = ros::Time::now();

    /// @brief ROI Filter / Ground Remover / Cluster
    // ROI Filter
    autosense::roi::approxVoxelGridFilter<autosense::PointI>(params_roi_.voxel_size, params_roi_.min_point_per_voxel, cloud);
    autosense::roi::applyROIFilter<autosense::PointI>(params_roi_, cloud);
    
    // Ground Remover
    std::vector<autosense::PointICloudPtr> cloud_clusters;
    autosense::PointICloudPtr cloud_ground(new autosense::PointICloud);
    autosense::PointICloudPtr cloud_nonground(new autosense::PointICloud);

    ground_remover_->segment(*cloud, cloud_clusters);
    *cloud_ground = *cloud_clusters[0];
    *cloud_nonground = *cloud_clusters[1];

    // reset clusters
    cloud_clusters.clear();

    // Segmenter
    segmenter_->segment(*cloud_nonground, cloud_clusters);

    //---------------------------------- temp debugging -------------------------------------------------
    std::vector<autosense::PointICloudPtr>::iterator it_cluster = cloud_clusters.begin();
    for (it_cluster=cloud_clusters.begin(); it_cluster<cloud_clusters.end(); it_cluster++)
    {
        int cluster_idx = std::distance(cloud_clusters.begin(), it_cluster);
        int each_cluster_size = cloud_clusters[cluster_idx]->points.size();
        std::cout<<each_cluster_size;

        float length_max = 0.0;
        for (size_t idx_1 = 0u; idx_1 < cloud_clusters[cluster_idx]->points.size(); ++idx_1) 
        {
            for (size_t idx_2 = 0u; idx_2 < cloud_clusters[cluster_idx]->points.size(); ++idx_2) 
            {
                float x_1 = cloud_clusters[cluster_idx]->points[idx_1].x;
                float y_1 = cloud_clusters[cluster_idx]->points[idx_1].y;
                float x_2 = cloud_clusters[cluster_idx]->points[idx_2].x;
                float y_2 = cloud_clusters[cluster_idx]->points[idx_2].y;
                float dist = sqrt(pow((x_1-x_2),2) + pow((y_1-y_2),2));

                if (dist > length_max)
                {
                    length_max = dist;
                }
            }
        }

        if (length_max > 2.2)
        {
            cloud_clusters.erase(it_cluster);
            it_cluster--;
        }
    }
    //-------------------------------------------------------------------------------------------

    autosense::common::publishPointCloudArray<autosense::PointICloudPtr>(pcs_array_segmented_pub_, header, cloud_clusters);
    autosense::common::publishClustersCloud<autosense::PointI>(pcs_segmented_pub_, header, cloud_clusters);

    ROS_INFO_STREAM("Cloud processed. Took " << clock.takeRealTime() << "ms.\n");
}