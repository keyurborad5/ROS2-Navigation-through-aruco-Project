#pragma once

#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include<nav_msgs/msg/odometry.hpp>
// for static broadcaster
#include "tf2_ros/static_transform_broadcaster.h"
// for dynamic broadcaster
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
using namespace std::chrono_literals;

class MyRobotNode : public rclcpp::Node
{
public:
    
    MyRobotNode(std::string node_name) : Node(node_name)
    {
        // parameter to decide whether to execute the broadcaster or not
        // RCLCPP_INFO(this->get_logger(), "Broadcaster demo started");

        //declaring node parameter
        this->declare_parameter("aruco_marker_0", "right_90");
        this->declare_parameter("aruco_marker_1","left_90");
        this->declare_parameter("aruco_marker_2","end");
        

        //Retriving node parameter by making them as attribute
        aruco_marker_0_=this->get_parameter("aruco_marker_0").as_string();
        aruco_marker_1_=this->get_parameter("aruco_marker_1").as_string();
        aruco_marker_2_=this->get_parameter("aruco_marker_2").as_string();
        

        // initialize the transform broadcaster
        aruco_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        part_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Load a buffer of transforms
        aruco_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        aruco_tf_buffer_->setUsingDedicatedThread(true);
        part_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        part_tf_buffer_->setUsingDedicatedThread(true);
        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();

        
        //************************Listener******************************
        // RCLCPP_INFO(this->get_logger(), "Listener demo started");

        // load a buffer of transforms
        aruco_tf_listener_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
        aruco_tf_listener_buffer_->setUsingDedicatedThread(true);
        aruco_transform_listener_ =std::make_shared<tf2_ros::TransformListener>(*aruco_tf_listener_buffer_);

        base_link_tf_listener_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
        base_link_tf_listener_buffer_->setUsingDedicatedThread(true);
        base_link_transform_listener_ =std::make_shared<tf2_ros::TransformListener>(*base_link_tf_listener_buffer_);

        part_tf_listener_buffer_ =std::make_unique<tf2_ros::Buffer>(this->get_clock());
        part_tf_listener_buffer_->setUsingDedicatedThread(true);
        part_transform_listener_ =std::make_shared<tf2_ros::TransformListener>(*part_tf_listener_buffer_);



        //********************Subscriber**************************
        aruco_cam_subscriber_=this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::aruco_cam_sub_cb,this,std::placeholders::_1));
        rclcpp::QoS qos(10); qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        part_cam_subscriber_=this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("mage/advanced_logical_camera/image",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::part_cam_sub_cb,this,std::placeholders::_1));
        
        odom_subscriber_=this->create_subscription<nav_msgs::msg::Odometry>("odom",rclcpp::SensorDataQoS(), std::bind(&MyRobotNode::odom_sub_cb,this,std::placeholders::_1));

        //********************Publisher*****************************
        cmd_val_publisher_=this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

        std::this_thread::sleep_for(std::chrono::seconds(5));

        cmd_val_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),std::bind(&MyRobotNode::cmd_val_pub_cb,this));
        

    }


private:

    std::string aruco_marker_0_;
    std::string aruco_marker_1_;
    std::string aruco_marker_2_;
    /*!< Boolean parameter to whether or not start the broadcaster */
    bool param_broadcast_;
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> aruco_tf_buffer_;
    /*!< MyRobotNode object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> aruco_tf_broadcaster_;
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> part_tf_buffer_;
    /*!< MyRobotNode object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> part_tf_broadcaster_;
    /*!< Utils object to access utility functions*/
    std::shared_ptr<Utils> utils_ptr_;
    double part_color_;
    uint8_t part_type_;
    int marker_id_;
    double target_rad_ =current_yaw_;
    double old_rad_ =current_yaw_;

    std::vector<double> part_vector_;
    std::vector<std::vector<double>> parts_vector_{0};
    
    

    /**
     * @brief Timer to broadcast the transform
     *
     */
    void aruco_broadcaster(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr);

    /**
     * @brief Timer to broadcast the transform
     *
     */
    void part_broadcaster(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr);
    

    //******************Listener***********************

     /*!< Boolean variable to store the value of the parameter "listen" */
    bool param_listen_;
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::unique_ptr<tf2_ros::Buffer> aruco_tf_listener_buffer_;
    std::unique_ptr<tf2_ros::Buffer> base_link_tf_listener_buffer_;
    /*!< Transform listener object */
    std::shared_ptr<tf2_ros::TransformListener> aruco_transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> base_link_transform_listener_{nullptr};

    double aruco_x_pos_;
    double aruco_y_pos_;
    double base_link_x_pos_;
    double base_link_y_pos_;
    double current_yaw_;
    bool flag1_=true;
    bool end_flag_=false;
    bool end_flag2_=false;

    
    /**
     * @brief Listen to a aruco transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void aruco_listen_transform(const std::string &source_frame, const std::string &target_frame);
     /**
     * @brief Listen to a base transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void base_link_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::unique_ptr<tf2_ros::Buffer> part_tf_listener_buffer_;
    /*!< Transform listener object */
    std::shared_ptr<tf2_ros::TransformListener> part_transform_listener_{nullptr};
    
    /**
     * @brief Listen to a part transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void part_listen_transform(const std::string &source_frame, const std::string &target_frame);


    //***********Subscriber**********************
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_cam_subscriber_;
    void aruco_cam_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr part_cam_subscriber_;
    void part_cam_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    void odom_sub_cb(nav_msgs::msg::Odometry::SharedPtr msg);
    //*******************Publisher**********************
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_val_publisher_;
    
    rclcpp::TimerBase::SharedPtr cmd_val_timer_;
    void cmd_val_pub_cb();
    

    //************************Calculating Distance****************************
    double distance(double x1,double y1,double x2, double y2);
    double convertToMinusPiToPi(double radians);
    void detected_part_locations(std::vector<double> vec);
   
};
