#pragma once

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>
// for static broadcaster
#include "tf2_ros/static_transform_broadcaster.h"
// for dynamic broadcaster
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
using namespace std::chrono_literals;

class MyRobotNode : public rclcpp::Node
{
public:
    MyRobotNode(std::string node_name) : Node(node_name)
    {
        // parameter to decide whether to execute the broadcaster or not
        RCLCPP_INFO(this->get_logger(), "Broadcaster demo started");

        // initialize the transform broadcaster
        aruco_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Load a buffer of transforms
        aruco_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        aruco_tf_buffer_->setUsingDedicatedThread(true);
        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();

        
        //************************Listener******************************
        RCLCPP_INFO(this->get_logger(), "Listener demo started");

        // load a buffer of transforms
        aruco_tf_listener_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
        aruco_tf_listener_buffer_->setUsingDedicatedThread(true);
        aruco_transform_listener_ =std::make_shared<tf2_ros::TransformListener>(*aruco_tf_listener_buffer_);


        //********************Subscriber**************************
        aruco_subscriber_=this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",10, std::bind(&MyRobotNode::aruco_marker_sub_cb,this,std::placeholders::_1));

    }


private:
    /*!< Boolean parameter to whether or not start the broadcaster */
    bool param_broadcast_;
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> aruco_tf_buffer_;
    /*!< MyRobotNode object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> aruco_tf_broadcaster_;
    /*!< Utils object to access utility functions*/
    std::shared_ptr<Utils> utils_ptr_;
    /*!< Wall timer object for the broadcaster*/
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    /*!< Wall timer object for the static broadcaster*/
    rclcpp::TimerBase::SharedPtr static_broadcast_timer_;

    /**
     * @brief Timer to broadcast the transform
     *
     */
    void aruco_broadcaster(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr);
    

    //******************Listener***********************

     /*!< Boolean variable to store the value of the parameter "listen" */
    bool param_listen_;
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::unique_ptr<tf2_ros::Buffer> aruco_tf_listener_buffer_;
    /*!< Transform listener object */
    std::shared_ptr<tf2_ros::TransformListener> aruco_transform_listener_{nullptr};
    /*!< Wall timer object */
    rclcpp::TimerBase::SharedPtr listen_timer_;

    /**
     * @brief Listen to a transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void aruco_listen_transform(const std::string &source_frame, const std::string &target_frame);


    //***********Subscriber**********************
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscriber_;
    void aruco_marker_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
};
