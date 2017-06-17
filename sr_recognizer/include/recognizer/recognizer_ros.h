/* Copyright 2017 ShadowRobot */

#ifndef RECOGNIZER_RECOGNIZER_ROS_H
#define RECOGNIZER_RECOGNIZER_ROS_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <sr_recognizer/RecognizerAction.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <iostream>
#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include <v4r/io/filesystem.h>
#include <v4r/apps/ObjectRecognizer.h>
#include <v4r/apps/change_detection.h>

typedef pcl::PointXYZRGB PointT;

class RecognizerROS
{
private:
    ros::NodeHandle nh_;  // NodeHandle instance must be created before next line. Otherwise strange error occurs.
    actionlib::SimpleActionServer<sr_recognizer::RecognizerAction> as_;
    ros::Publisher vis_pc_pub_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    sr_recognizer::RecognizerFeedback feedback_;
    sr_recognizer::RecognizerResult result_;

    pcl::PointCloud<PointT>::Ptr inputCloudPtr;
    pcl::PointCloud<PointT>::Ptr inputCloudPtr_old;
    pcl::PointCloud<PointT>::Ptr inputCloudPtr_empty;
    std::vector<typename v4r::ObjectHypothesis<PointT>::Ptr > ohs;
    std::string cfg_path;

    pcl::PointCloud<PointT>::Ptr kinectCloudPtr;
    std::string topic_;
    bool KINECT_OK_;
    int change;
    int init_empty;

public:
    explicit RecognizerROS(std::string name):
        as_(nh_, name, boost::bind(&RecognizerROS::recognize_cb, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~RecognizerROS(void)
    {
    }

    std::vector<std::string> arguments;
    boost::shared_ptr<v4r::apps::ObjectRecognizer<PointT>> rec;
    boost::shared_ptr<v4r::apps::ObjectRecognizerParameter> param;
    boost::shared_ptr<v4r::apps::ChangeDetector<PointT>> detector;

    void checkCloudArrive(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool checkKinect();
    bool initialize();
    void recognize_cb(const sr_recognizer::RecognizerGoalConstPtr &goal);
    typename pcl::PointCloud<PointT>::Ptr getScene();
};

#endif  // RECOGNIZER_RECOGNIZER_ROS_H
