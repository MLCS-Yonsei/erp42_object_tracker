/*
 * Copyright (C) 2019 by AutoSense Organization. All rights reserved.
 * Gary Chan <chenshj35@mail2.sysu.edu.cn>
 */


#include "erp42_object_tracker_node/tracking_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracking_node"); 

    ObjectTracker tracker;

    try
    {
        if(!tracker.updateParams())
        {
            ROS_ERROR("FAILED TO INITIALIZE %s", ros::this_node::getName().c_str());
            exit(1);
        }
        else
        {
            ROS_INFO("%s INITIALIZED SUCCESSFULLY!", ros::this_node::getName().c_str());
            while (tracker.nh.ok()) {
                tracker.spinNode();
            }
        }
    }
    catch (ros::Exception& e)
    {
        ROS_ERROR("tracking_node: Error occured: %s", e.what());
        exit(1);
    }  

    ros::waitForShutdown();
    ROS_INFO("tracking_node exited...");
}

void ObjectTracker::spinNode()
{
    spiner.start();
}

bool ObjectTracker::updateParams()
{
    // Load ROS parameters from rosparam server
    private_nh.getParam(param_ns_prefix_ + "/local_frame_id", local_frame_id_);
    private_nh.getParam(param_ns_prefix_ + "/global_frame_id", global_frame_id_);
    private_nh.getParam(param_ns_prefix_ + "/tf_timeout_ms", tf_timeout_ms_);

    std::string sub_pcs_segmented_topic;
    int sub_pcs_queue_size;
    private_nh.getParam(param_ns_prefix_ + "/sub_pcs_segmented_topic", sub_pcs_segmented_topic);
    private_nh.getParam(param_ns_prefix_ + "/sub_pcs_queue_size", sub_pcs_queue_size);

    std::string pub_segments_coarse_topic, pub_segments_predict_topic, pub_segments_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_coarse_topic", pub_segments_coarse_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_predict_topic", pub_segments_predict_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_segments_topic", pub_segments_topic);

    std::string pub_output_objects_topic, pub_output_trajectories_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_output_objects_topic", pub_output_objects_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_output_trajectories_topic", pub_output_trajectories_topic);

    std::string pub_tracking_objects_topic, pub_tracking_objects_cloud_topic,
        pub_tracking_objects_velocity_topic,
        pub_tracking_objects_trajectory_topic;
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_topic", pub_tracking_objects_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_cloud_topic", pub_tracking_objects_cloud_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_velocity_topic", pub_tracking_objects_velocity_topic);
    private_nh.getParam(param_ns_prefix_ + "/pub_tracking_objects_trajectory_topic", pub_tracking_objects_trajectory_topic);

    private_nh.param<double>(param_ns_prefix_ + "/threshold_contian_IoU", threshold_contian_IoU_, 1.0);

    //************************************
    private_nh.getParam(param_ns_prefix_ + "/IHGP/frequency", frequency);
    private_nh.getParam(param_ns_prefix_ + "/IHGP/id_threshold", id_threshold);
    private_nh.getParam(param_ns_prefix_ + "/IHGP/data_length", data_length);

    private_nh.getParam(param_ns_prefix_ + "/IHGP/lpf_tau", lpf_tau);
    private_nh.getParam(param_ns_prefix_ + "/IHGP/logSigma2_x", logSigma2_x);
    private_nh.getParam(param_ns_prefix_ + "/IHGP/logMagnSigma2_x", logMagnSigma2_x);
    private_nh.getParam(param_ns_prefix_ + "/IHGP/logLengthScale_x", logLengthScale_x);

    private_nh.getParam(param_ns_prefix_ + "/IHGP/logSigma2_y", logSigma2_y);
    private_nh.getParam(param_ns_prefix_ + "/IHGP/logMagnSigma2_y", logMagnSigma2_y);
    private_nh.getParam(param_ns_prefix_ + "/IHGP/logLengthScale_y", logLengthScale_y);

    //************************************

    tracking_params_ = autosense::common::getTrackingWorkerParams(nh, param_ns_prefix_);

    // Init core compoments
    object_builder_ = autosense::object_builder::createObjectBuilder();
    if (nullptr == object_builder_) 
    {
        ROS_FATAL("Failed to create object_builder_.");
        return -1;
    }
    // tracking_worker_ = autosense::tracking::createTrackingWorker(tracking_params_);
    // if (nullptr == tracking_worker_) 
    // {
    //     ROS_FATAL("Failed to create tracking_worker_.");
    //     return -1;
    // }

    // Init subscribers and publishers
    pcs_segmented_sub_ = nh.subscribe<autosense_msgs::PointCloud2Array>(sub_pcs_segmented_topic, sub_pcs_queue_size, &ObjectTracker::pointcloudCallback, this);
    // tf_listener_.reset(new tf::TransformListener);
    // segments
    // segments_coarse_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(pub_segments_coarse_topic, 1);
    // segments_predict_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(pub_segments_predict_topic, 1);
    // segments_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(pub_segments_topic, 1);
    // // tracking infos for debugging
    // tracking_objects_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(pub_tracking_objects_topic, 1);
    // tracking_objects_cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>(pub_tracking_objects_cloud_topic, 1);
    // tracking_objects_velocity_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(pub_tracking_objects_velocity_topic, 1);
    // tracking_objects_trajectory_pub_ = private_nh.advertise<visualization_msgs::MarkerArray>(pub_tracking_objects_trajectory_topic, 1);
    // // the whole tracking output
    // tracking_output_objects_pub_ = private_nh.advertise<autosense_msgs::TrackingObjectArray>(pub_output_objects_topic, 1);
    // tracking_output_trajectories_pub_ = private_nh.advertise<autosense_msgs::TrackingFixedTrajectoryArray>(pub_output_trajectories_topic, 1);

    marker_pub = private_nh.advertise<visualization_msgs::MarkerArray>("tracker_viz", 1); // rviz visualization
    pose_pub = private_nh.advertise<sensor_msgs::PointCloud>("pose_marker", 1);

    return true;
}

void ObjectTracker::pointcloudCallback(const autosense_msgs::PointCloud2ArrayConstPtr &segments_msg)
{
    // get msg header data
    const double kTimeStamp = segments_msg->header.stamp.toSec();
    std_msgs::Header header;
    header.frame_id = local_frame_id_;
    header.stamp = segments_msg->header.stamp;

    tf::StampedTransform transform;
    try
    {
        tf_listener_.lookupTransform("/map", "/velodyne", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    if (first_frame==true)
    {
        for (int i = 0; i < data_length; ++i)
        { 
            ego_tf_stack.push_back(transform);
        }
        first_frame==false;
    }
    else
    {
        ego_tf_stack.erase(ego_tf_stack.begin());
        ego_tf_stack.push_back(transform);
    }

    std::vector<pcl::PointXYZI> clusterCentroids;
    std::vector<autosense::PointICloudPtr> segment_clouds;
    for (size_t i = 0u; i < segments_msg->clouds.size(); ++i) 
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(segments_msg->clouds[i], *cloud);

        // cloud center (sum average)
        float x = 0.0;
        float y = 0.0;
        int numPts = 0;
        for (auto it=0; it<=cloud->points.size(); ++it)
        {
            if (std::abs(cloud->points[it].x) > 1e+10 || std::abs(cloud->points[it].y) > 1e+10)
            {
                continue;
            }
            x += cloud->points[it].x;
            y += cloud->points[it].y;
            numPts++;
        }

        pcl::PointXYZI centroid;
        // centroid.x = x / numPts + transform.getOrigin().x();
        // centroid.y = y / numPts + transform.getOrigin().y();
        centroid.x = x / numPts;
        centroid.y = y / numPts;
        centroid.z = 0.0;
        centroid.intensity = kTimeStamp;

        // Get the centroid of the cluster
        clusterCentroids.push_back(centroid);

        segment_clouds.push_back(cloud);
    }

    //****************************************************************************
    // exit callback if no obstacles
    if (clusterCentroids.size()==0)
    {
        ROS_INFO("No obstacles around");
        return;
    }

    std::vector<int> this_objIDs; // Obj lists for this timestamp
    for (const auto& obj: clusterCentroids)
    {
        bool registered = false;
        int update_index = 0;
        int objID = 0;

        std::vector<int>::iterator it;
        for (it=objIDs.begin(); it<objIDs.end(); ++it)
        {
            int index = std::distance(objIDs.begin(), it);

            // compare new cluster with final poses of registered objs
            Eigen::Vector3d P_now; // Position (now)
            Eigen::Vector3d P_last; // Position (last observation)
            P_now << obj.x, obj.y, 0;
            P_last << stack_obj[index][data_length-1].x, stack_obj[index][data_length-1].y, 0;

            float dist = euc_dist(P_now, P_last);
            if (dist < id_threshold)
            {
                // if there is lost data, linear interpolation for sparse data
                if (obj.intensity - stack_obj[index][data_length-1].intensity > 3*dt_gp)
                {
                    fill_with_linear_interpolation(index, obj);
                }

                registered = true;
                update_index = index;
                objID = *it;
                break;
            }
        }

        if (registered) // if already registered object, update data
        {
            updateObstacleQueue(update_index, obj);
            this_objIDs.push_back(objID);
        }
        else // if new object detects, register new GP model
        {
            this_objIDs.push_back(next_obj_num);
            registerNewObstacle(obj);
        }            
    }

    // call IHGP
    std::vector<std::vector<pcl::PointXYZI>> pos_vel_s;
    pos_vel_s = callIHGP(this_objIDs);

    // Publish TEB_Obstacle msg & rviz marker
    std::string frame_id = header.frame_id;
    publishMarkers(pos_vel_s, frame_id, this_objIDs);

    // object builder
    // autosense::common::Clock clock_builder;
    // std::vector<autosense::ObjectPtr> objects;
    // object_builder_->build(segment_clouds, &objects);

    // // visualize initial coarse segments
    // autosense::common::publishObjectsMarkers(segments_coarse_pub_, header, autosense::common::MAGENTA.rgbA, objects);

    // unregister old unseen obstacles 
    unregisterOldObstacle(kTimeStamp);

}

void ObjectTracker::registerNewObstacle(pcl::PointXYZI centroid)
{
    // register objID list
    objIDs.push_back(next_obj_num);
    next_obj_num += 1;

    // fill every data with current input
    std::vector<pcl::PointXYZI> centroids;
    for (int i = 0; i < data_length; ++i)
    { 
        centroids.push_back(centroid);
    }
    stack_obj.push_back(centroids);

    // Set IHGP hyperparameters
    Matern32model model_x;
    Matern32model model_y;
    model_x.setSigma2(exp(logSigma2_x));
    model_x.setMagnSigma2(exp(logMagnSigma2_x)); 
    model_x.setLengthScale(exp(logLengthScale_x));

    model_y.setSigma2(exp(logSigma2_y));
    model_y.setMagnSigma2(exp(logMagnSigma2_y)); 
    model_y.setLengthScale(exp(logLengthScale_y));

    // GP initialization
    GPs_x.push_back(new InfiniteHorizonGP(dt_gp,model_x.getF(),model_x.getH(),model_x.getPinf(),model_x.getR(),model_x.getdF(),model_x.getdPinf(),model_x.getdR()));
    GPs_y.push_back(new InfiniteHorizonGP(dt_gp,model_y.getF(),model_y.getH(),model_y.getPinf(),model_y.getR(),model_y.getdF(),model_y.getdPinf(),model_y.getdR()));

    // Set rviz msg color
    std_msgs::ColorRGBA color;
    color.r = (float)std::rand()/(float)RAND_MAX;
    color.g = (float)std::rand()/(float)RAND_MAX;
    color.b = (float)std::rand()/(float)RAND_MAX;
    color.a = 0.8;
    colorset.push_back(color);

    ROS_WARN_STREAM("++++++++++++New Obstacle++++++++++++");
}

void ObjectTracker::unregisterOldObstacle(double now)
{
    spin_counter += 1;

    // remove old obstacles every period (sec)
    double period = 5; // 5 sec
    if (spin_counter > period * frequency)
    {
        std::vector<int>::iterator it_objID = objIDs.begin();
        std::vector<std::vector<pcl::PointXYZI>>::iterator it_stack = stack_obj.begin();
        std::vector<InfiniteHorizonGP*>::iterator it_GPx = GPs_x.begin();
        std::vector<InfiniteHorizonGP*>::iterator it_GPy = GPs_y.begin();
        std::vector<std_msgs::ColorRGBA>::iterator it_color = colorset.begin();

        for (it_objID=objIDs.begin(); it_objID<objIDs.end();)
        {
            int index = std::distance(objIDs.begin(), it_objID);

            // if obj be unseen for period, remove
            if (now - stack_obj[index][data_length-1].intensity > period)
            {
                it_objID = objIDs.erase(it_objID); // remove objID 
                it_stack = stack_obj.erase(it_stack); // remove stack_obj
                it_GPx = GPs_x.erase(it_GPx); // remove GPs_x 
                it_GPy = GPs_y.erase(it_GPy); // remove GPs_y 
                it_color = colorset.erase(it_color);

                ROS_WARN_STREAM("--------- Old Obstacle -------");
            }
            else
            {
                it_objID++;
                it_stack++;
                it_GPx++;
                it_GPy++;
                it_color++;
            }
        }
        
        spin_counter = 0;
    }  
}

void ObjectTracker::updateObstacleQueue(const int i, pcl::PointXYZI centroid)
{
    // now update objects_centroids
    stack_obj[i].erase(stack_obj[i].begin());
    stack_obj[i].push_back(centroid);
}

void ObjectTracker::fill_with_linear_interpolation(const int i, pcl::PointXYZI centroid)
{
    ROS_INFO_STREAM("obj[" << i << "] tracking loss");

    // linear interpolation
    pcl::PointXYZI last_centroid = stack_obj[i][data_length-1];  
    double dx_total = centroid.x - last_centroid.x; // dx between last timestamp and this timestamp
    double dy_total = centroid.y - last_centroid.y; // dy between last timestamp and this timestamp
    double dz_total = 0;
    double dt_total = centroid.intensity - last_centroid.intensity; // dt between last timestamp and this timestamp
    int lost_num = (int)round(dt_total/dt_gp) - 1; // # of lost data

    // (lost_num) times of linear interpolation
    for (int j=0; j < lost_num; ++j)
    {
        pcl::PointXYZI last_center = stack_obj[i][data_length-1];

        pcl::PointXYZI center;
        center.x = last_center.x + dx_total/lost_num;
        center.y = last_center.y + dy_total/lost_num;
        center.z = last_center.z + dz_total/lost_num;
        center.intensity = last_center.intensity + dt_gp;

        stack_obj[i].erase(stack_obj[i].begin());
        stack_obj[i].push_back(center);
    }
}

float ObjectTracker::euc_dist(Eigen::Vector3d P1, Eigen::Vector3d P2)
{
    return std::sqrt((P1(0)-P2(0))*(P1(0)-P2(0)) + (P1(1)-P2(1))*(P1(1)-P2(1)) + (P1(2)-P2(2))*(P1(2)-P2(2)));
}

std::vector<std::vector<pcl::PointXYZI>> ObjectTracker::callIHGP(std::vector<int> this_objIDs)
{
    // define vector for position and velocity
    std::vector<std::vector<pcl::PointXYZI>> pos_vel_s; // vector stack for objects pose and velocity
    int output_size = 2*this_objIDs.size();
    pos_vel_s.reserve(output_size);

    // ROS_ERROR_STREAM(this_objIDs.size());

    // call IHGP
    for (const int& n: this_objIDs)
    {
        // objID index for call GPs_x,y
        auto iter = std::find(objIDs.begin(), objIDs.end(), n);
        int index = std::distance(objIDs.begin(), iter);

        std::vector<pcl::PointXYZI> pos_vel; // vector stack for one object pose and velocity
        pos_vel.reserve(2);

        pcl::PointXYZI pos = LPF_pos(stack_obj[index]);
        // pcl::PointXYZI pos = IHGP_fixed_pos(stack_obj[index], index);
        pcl::PointXYZI vel = IHGP_fixed_vel(stack_obj[index], index);

        // Nan exception
        if (isnan(vel.x) != 0 || isnan(vel.y) != 0)
        {
            // ROS_WARN_STREAM("Pose: "<<pos.x<<" / "<<pos.y);
            ROS_ERROR_STREAM("NaN detected in GP. Please restart this node. "<<vel.x<<" / "<<vel.y);
        }

        // obstacle velocity bounding
        float obs_max_vx = 6; // (m/s)
        float obs_max_vy = 6;
        if (vel.x > obs_max_vx) {vel.x = obs_max_vx;}
        else if (vel.x < -obs_max_vx) {vel.x = -obs_max_vx;}
        if (vel.y > obs_max_vy) {vel.y = obs_max_vy;}
        else if (vel.y < -obs_max_vy) {vel.y = -obs_max_vy;}

        // float linear_vel = std::sqrt(std::pow(vel.x,2)+std::pow(vel.y,2));
        // ROS_WARN_STREAM("Velocity:"<<linear_vel);

        pos_vel.push_back(pos);
        pos_vel.push_back(vel); // e.g. pos_vel = [pos3, vel3]
        pos_vel_s.push_back(pos_vel); // e.g. pos_vel_s = [[pos1, vel1], [pos3, vel3], ...]
    }
    
    return pos_vel_s;
}

pcl::PointXYZI ObjectTracker::LPF_pos(std::vector<pcl::PointXYZI> centroids)
{
    pcl::PointXYZI centroid_lpf;
    centroid_lpf.x = (lpf_tau/(lpf_tau+dt_gp))*centroids[data_length-2].x + (dt_gp/(lpf_tau+dt_gp))*centroids[data_length-1].x;
    centroid_lpf.y = (lpf_tau/(lpf_tau+dt_gp))*centroids[data_length-2].y + (dt_gp/(lpf_tau+dt_gp))*centroids[data_length-1].y;
    centroid_lpf.z = 0;
    centroid_lpf.intensity = centroids[data_length-1].intensity;

    return centroid_lpf;
}

pcl::PointXYZI ObjectTracker::IHGP_fixed_pos(std::vector<pcl::PointXYZI> centroids, int n)
{
    // initialize AKHA and MF vector 
    GPs_x[n]->init_step();
    GPs_y[n]->init_step();

    // Data pre-precessing
    // set mean as last value
    double mean_x;
    double mean_y;
    int gp_data_len = data_length-1;

    mean_x = centroids[gp_data_len].x;
    mean_y = centroids[gp_data_len].y;

    // update IHGP queue through data
    for (int k=0; k<=gp_data_len; k++)
    {
        GPs_x[n]->update(centroids[k].x - mean_x);
        GPs_y[n]->update(centroids[k].y - mean_y);
    }

    // Pull out the marginal mean and variance estimates
    std::vector<double> Eft_x = GPs_x[n]->getEft();
    std::vector<double> Eft_y = GPs_y[n]->getEft();

    // make return_data as PCL::PointXYZI
    pcl::PointXYZI predicted_centroid;
    predicted_centroid.x = Eft_x[gp_data_len] + mean_x;
    predicted_centroid.y = Eft_y[gp_data_len] + mean_y;
    predicted_centroid.z = 0.0;
    predicted_centroid.intensity = centroids[data_length-1].intensity;
  
    return predicted_centroid;
}

pcl::PointXYZI ObjectTracker::IHGP_fixed_vel(std::vector<pcl::PointXYZI> centroids, int n)
{
    // initialize AKHA and MF vector 
    GPs_x[n]->init_step();
    GPs_y[n]->init_step();

    // Data pre-precessing
    // set mean as last value
    double mean_x;
    double mean_y;
    std::vector<double> vx_raw; 
    std::vector<double> vy_raw; 

    // calculate mean
    int gp_data_len = data_length-2;

    for (int k=0; k<=gp_data_len; k++)
    {
        double ego_vel = ego_tf_stack[k+1].getOrigin().x() - ego_tf_stack[k].getOrigin().x();
        double vel = (centroids[k+1].x - centroids[k].x + ego_vel)/dt_gp;
        
        ROS_WARN_STREAM("Velocity X: "<<ego_vel<<", "<<vel);

        vx_raw.push_back(vel);
        mean_x += vel;

        ego_vel = ego_tf_stack[k+1].getOrigin().y() - ego_tf_stack[k].getOrigin().y();
        vel = (centroids[k+1].y - centroids[k].y + ego_vel)/dt_gp;

        ROS_WARN_STREAM("Velocity Y: "<<ego_vel<<", "<<vel);

        vy_raw.push_back(vel);
        mean_y += vel;
    }
    mean_x = mean_x/(gp_data_len+1);
    mean_y = mean_y/(gp_data_len+1);
   
    
    // update IHGP queue through data
    for (int k=0; k<=gp_data_len; k++)
    {
        GPs_x[n]->update(vx_raw[k] - mean_x);
        GPs_y[n]->update(vy_raw[k] - mean_y);
    }

    // Pull out the marginal mean and variance estimates
    std::vector<double> Eft_x = GPs_x[n]->getEft();
    std::vector<double> Eft_y = GPs_y[n]->getEft();

    // ROS_WARN_STREAM("GP: "<<Eft_x[gp_data_len]<<" + "<<mean_x<<" / "<<Eft_y[gp_data_len]<<" + "<<mean_y);

    // make return_data as PCL::PointXYZI
    pcl::PointXYZI predicted_centroid;
    predicted_centroid.x = Eft_x[gp_data_len] + mean_x;
    predicted_centroid.y = Eft_y[gp_data_len] + mean_y;
    predicted_centroid.z = 0.0;
    predicted_centroid.intensity = centroids[data_length-1].intensity;
  
    return predicted_centroid;
}

void ObjectTracker::publishMarkers(std::vector<std::vector<pcl::PointXYZI>> pos_vel_s, std::string frame_id, std::vector<int> this_objIDs)
{
    // pose marker
    sensor_msgs::PointCloud pose_msg;
    pose_msg.header.frame_id = frame_id;

    sensor_msgs::ChannelFloat32 color_info;
    color_info.name = "intensity";

    for (int i=0; i<pos_vel_s.size(); i++)
    {   
        auto iter = std::find(objIDs.begin(), objIDs.end(), this_objIDs[i]);
        int index = std::distance(objIDs.begin(), iter);
        
        geometry_msgs::Point32 point;
        point.x = pos_vel_s[i][0].x;
        point.y = pos_vel_s[i][0].y;
        point.z = 0;
        pose_msg.points.push_back(point);

        float color = 255 * colorset[index].g;
        color_info.values.push_back(color);
    }
    pose_msg.channels.push_back(color_info);
    pose_pub.publish(pose_msg);

    visualization_msgs::MarkerArray obstacleMarkers;

    visualization_msgs::Marker m_remove;
    m_remove.id = 0;
    m_remove.action = 3;
    obstacleMarkers.markers.push_back(m_remove);

    // velocity text marker
    // for (int i=0; i<pos_vel_s.size(); i++)
    // {
    //     visualization_msgs::Marker m;

    //     m.id = 2*this_objIDs[i]+1;
    //     // m.id = this_objIDs[i];

    //     m.header.frame_id = frame_id;
    //     m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //     m.action = visualization_msgs::Marker::ADD;
    //     m.scale.z = 0.22; // text size
        
    //     m.color.r=1.0;
    //     m.color.g=1.0;
    //     m.color.b=1.0;
    //     m.color.a=1.0;

    //     m.pose.position.x = pos_vel_s[i][0].x;
    //     m.pose.position.y = pos_vel_s[i][0].y;
    //     m.pose.position.z = 0.0;  

    //     float velocity = round(sqrt(pow(pos_vel_s[i][1].x, 2) + pow(pos_vel_s[i][1].y, 2))*100) / 100;
    //     std::ostringstream velocity_3f;
    //     velocity_3f << std::setprecision(2) << velocity;
    //     std::string text = velocity_3f.str();
    //     m.text = text; 

    //     obstacleMarkers.markers.push_back(m);
    // } 

    // velocity array marker
    for (int i=0; i<pos_vel_s.size(); i++)
    {
        visualization_msgs::Marker m;
        m.id = this_objIDs[i] + 2*pos_vel_s.size();
        // m.id = this_objIDs[i] + 1*pos_vel_s.size();
        //m.id = this_objIDs[i];
        m.header.frame_id = frame_id;
        m.type = visualization_msgs::Marker::ARROW;
        m.action = visualization_msgs::Marker::ADD;
        // arrow orientation    arrow width         arrow height
        m.scale.x = 0.08;         m.scale.y = 0.15;         m.scale.z = 0.2;
        
        m.color.r=0.75;
        m.color.g=0.0;
        m.color.b=0.0;
        m.color.a=1.0;
        geometry_msgs::Point arrow_yaw;
        // specify start point for arrow
        arrow_yaw.x = pos_vel_s[i][0].x;
        arrow_yaw.y = pos_vel_s[i][0].y;
        arrow_yaw.z = 0.0;
        m.points.push_back(arrow_yaw);
        // specify start point for arrow
        arrow_yaw.x = pos_vel_s[i][0].x + 1*pos_vel_s[i][1].x;
        arrow_yaw.y = pos_vel_s[i][0].y + 1*pos_vel_s[i][1].y;
        arrow_yaw.z = 0.0;
        m.points.push_back(arrow_yaw);
        obstacleMarkers.markers.push_back(m);
    } 

    marker_pub.publish(obstacleMarkers);
}