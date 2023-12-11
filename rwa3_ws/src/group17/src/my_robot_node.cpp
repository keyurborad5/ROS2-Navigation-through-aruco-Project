#include <my_robot_node.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utils.hpp>
// needed for the listener
#include <tf2/exceptions.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

// allows to use, 50ms, etc
using namespace std::chrono_literals;



void MyRobotNode::aruco_broadcaster(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped aruco_dynamic_transform_stamped;

    // RCLCPP_INFO(this->get_logger(), "Broadcasting dynamic_frame");
    aruco_dynamic_transform_stamped.header.stamp = msg->header.stamp;

    aruco_dynamic_transform_stamped.header.frame_id = msg->header.frame_id;
    aruco_dynamic_transform_stamped.child_frame_id = "aruco_mark";

    aruco_dynamic_transform_stamped.transform.translation.x = msg->poses[0].position.x;
    aruco_dynamic_transform_stamped.transform.translation.y = msg->poses[0].position.y;
    aruco_dynamic_transform_stamped.transform.translation.z = msg->poses[0].position.z;

    //geometry_msgs::msg::Quaternion quaternion = utils_ptr_->set_quaternion_from_euler(M_PI, 0.0, M_PI / 3);
    aruco_dynamic_transform_stamped.transform.rotation.x = msg->poses[0].orientation.x;
    aruco_dynamic_transform_stamped.transform.rotation.y = msg->poses[0].orientation.y;
    aruco_dynamic_transform_stamped.transform.rotation.z = msg->poses[0].orientation.z;
    aruco_dynamic_transform_stamped.transform.rotation.w = msg->poses[0].orientation.w;
    // Send the transform
    aruco_tf_broadcaster_->sendTransform(aruco_dynamic_transform_stamped);
    RCLCPP_INFO_STREAM(this->get_logger(), "Aruco Broadcasting_dynamic_frame : "<<msg->poses[0].position.x);
}

void MyRobotNode::aruco_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = aruco_tf_listener_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;
    pose_out.orientation = t_stamped.transform.rotation;

    // RCLCPP_INFO_STREAM(this->get_logger(), target_frame << " in " << source_frame << ":\n"
    //                                                     << "x: " << pose_out.position.x << "\t"
    //                                                     << "y: " << pose_out.position.y << "\t"
    //                                                     << "z: " << pose_out.position.z << "\n"
    //                                                     << "qx: " << pose_out.orientation.x << "\t"
    //                                                     << "qy: " << pose_out.orientation.y << "\t"
    //                                                     << "qz: " << pose_out.orientation.z << "\t"
    //                                                     << "qw: " << pose_out.orientation.w << "\n");
}

void MyRobotNode::aruco_marker_sub_cb(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
    
    RCLCPP_INFO_STREAM(this->get_logger(),"Aruco marker callback ");
    
    aruco_broadcaster(msg);

    aruco_listen_transform("odom", "aruco_mark");


    

}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyRobotNode>("my_robot_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
}