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
    double small_val=100000;
    long unsigned j=0;
    RCLCPP_INFO_STREAM(this->get_logger(),"number of markerID : "<<msg->marker_ids.size());
    if(msg->marker_ids.size()!=0){
        for( long unsigned int i=0 ; i<=msg->marker_ids.size();++i){
                
                if(small_val>msg->poses[i].position.z){
                    small_val=msg->poses[i].position.z;
                    j=i-1;
                }
        }
            RCLCPP_INFO_STREAM(this->get_logger(),"closest distance: "<<msg->poses[j].position.z);
    }

    // if (j>3){j=0;}
    marker_id_=msg->marker_ids[j];

    geometry_msgs::msg::TransformStamped aruco_dynamic_transform_stamped;

     RCLCPP_INFO(this->get_logger(), "Broadcasting dynamic_frame");
    aruco_dynamic_transform_stamped.header.stamp = msg->header.stamp;

    aruco_dynamic_transform_stamped.header.frame_id = msg->header.frame_id;
    aruco_dynamic_transform_stamped.child_frame_id = "aruco_mark";

    aruco_dynamic_transform_stamped.transform.translation.x = msg->poses[j].position.x;
    aruco_dynamic_transform_stamped.transform.translation.y = msg->poses[j].position.y;
    aruco_dynamic_transform_stamped.transform.translation.z = msg->poses[j].position.z;

    //geometry_msgs::msg::Quaternion quaternion = utils_ptr_->set_quaternion_from_euler(M_PI, 0.0, M_PI / 3);
    aruco_dynamic_transform_stamped.transform.rotation.x = msg->poses[j].orientation.x;
    aruco_dynamic_transform_stamped.transform.rotation.y = msg->poses[j].orientation.y;
    aruco_dynamic_transform_stamped.transform.rotation.z = msg->poses[j].orientation.z;
    aruco_dynamic_transform_stamped.transform.rotation.w = msg->poses[j].orientation.w;
    // Send the transform
    aruco_tf_broadcaster_->sendTransform(aruco_dynamic_transform_stamped);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Aruco Broadcasting_dynamic_frame : "<<msg->poses[0].position.x);
}
void MyRobotNode::part_broadcaster(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    geometry_msgs::msg::TransformStamped part_dynamic_transform_stamped;

    // RCLCPP_INFO(this->get_logger(), "Broadcasting dynamic_frame");
    part_dynamic_transform_stamped.header.stamp = this->get_clock()->now();

    part_dynamic_transform_stamped.header.frame_id = "logical_camera_link";
    part_dynamic_transform_stamped.child_frame_id = "part";

    part_color_= msg->part_poses[0].part.color;
    part_type_= msg->part_poses[0].part.type;

    part_dynamic_transform_stamped.transform.translation.x = msg->part_poses[0].pose.position.x;
    part_dynamic_transform_stamped.transform.translation.y = msg->part_poses[0].pose.position.y;
    part_dynamic_transform_stamped.transform.translation.z = -msg->part_poses[0].pose.position.z;

    //geometry_msgs::msg::Quaternion quaternion = utils_ptr_->set_quaternion_from_euler(M_PI, 0.0, M_PI / 3);
    part_dynamic_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    part_dynamic_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    part_dynamic_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    part_dynamic_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;
    // Send the transform
    part_tf_broadcaster_->sendTransform(part_dynamic_transform_stamped);
    // RCLCPP_INFO_STREAM(this->get_logger(), "PART advance cam Broadcasting_dynamic_frame : "<<msg->part_poses[0].pose.position.x;);
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
    aruco_x_pos_ = t_stamped.transform.translation.x;
    aruco_y_pos_ = t_stamped.transform.translation.y;

    // RCLCPP_INFO_STREAM(this->get_logger(), target_frame << " in " << source_frame << ":\n"
    //                                                     << "x: " << pose_out.position.x << "\t"
    //                                                     << "y: " << pose_out.position.y << "\t"
    //                                                     << "z: " << pose_out.position.z << "\n"
    //                                                     << "qx: " << pose_out.orientation.x << "\t"
    //                                                     << "qy: " << pose_out.orientation.y << "\t"
    //                                                     << "qz: " << pose_out.orientation.z << "\t"
    //                                                     << "qw: " << pose_out.orientation.w << "\n");
}
void MyRobotNode::base_link_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = base_link_tf_listener_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
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
    base_link_x_pos_ = t_stamped.transform.translation.x;
    base_link_y_pos_ = t_stamped.transform.translation.y;


    // RCLCPP_INFO_STREAM(this->get_logger(), target_frame << " in " << source_frame << ":\n"
    //                                                     << "x: " << pose_out.position.x << "\t"
    //                                                     << "y: " << pose_out.position.y << "\t"
    //                                                     << "z: " << pose_out.position.z << "\n"
    //                                                     << "qx: " << pose_out.orientation.x << "\t"
    //                                                     << "qy: " << pose_out.orientation.y << "\t"
    //                                                     << "qz: " << pose_out.orientation.z << "\t"
    //                                                     << "qw: " << pose_out.orientation.w << "\n");
}
void MyRobotNode::part_listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = part_tf_listener_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
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

    tf2::Quaternion q( 
        pose_out.orientation.x,
        pose_out.orientation.y,
        pose_out.orientation.z,
        pose_out.orientation.w);
    std::array<double, 3> euler = utils_ptr_->set_euler_from_quaternion(q);
    std::vector<double> detection={part_color_,pose_out.position.x,pose_out.position.y,pose_out.position.z,euler[0],euler[1],euler[2]};
    detected_part_locations(detection);
    // for(size_t i; i<parts_vector_.size();++i){
    //     if(parts_vector_[i][0])

    // }

    RCLCPP_INFO_STREAM(this->get_logger(), target_frame << " in " << source_frame << ":\n"
                                                        << "x: " << pose_out.position.x << "\t"
                                                        << "y: " << pose_out.position.y << "\t"
                                                        << "z: " << pose_out.position.z << "\n"
                                                        << "qx: " << pose_out.orientation.x << "\t"
                                                        << "qy: " << pose_out.orientation.y << "\t"
                                                        << "qz: " << pose_out.orientation.z << "\t"
                                                        << "qw: " << pose_out.orientation.w << "\n");
}
 void MyRobotNode::detected_part_locations(std::vector<double> vec){
    
    int count=0;
    if(!parts_vector_.empty()){
        for(long unsigned int i=0;i<parts_vector_.size();++i){
            if( parts_vector_[i][0] ==vec[0]){
                count+=1;
            }
    }
    }
    if(count==0){
        parts_vector_.push_back(vec);
        RCLCPP_INFO_STREAM(this->get_logger(), "Added part to the list");
        for (const auto& vect : parts_vector_) {
            for ( const auto element : vect) {
                RCLCPP_INFO_STREAM(this->get_logger(),  element << ' ');
            }
            
        }
    }
    }
    // if (std::any_of(parts_vector_.begin(), parts_vector_.end(),[&](const std::vector<int>& existingVector) {
    //                            return vec[0] == existingVector[0];
    //                        })) {parts_vector_=parts_vector_;
            
    //     }
    //     else{


    //         parts_vector_.push_back(vec);
    //         RCLCPP_INFO_STREAM(this->get_logger(), 'Added part to the list');

    //         // std::cout << "Vector of Vectors:" << std::endl;
    //         for (const auto& vect : parts_vector_) {
    //             for (int element : vect) {
    //                 RCLCPP_INFO_STREAM(this->get_logger(),  element << ' ');
    //             }
                
    //         }

    // }
 

void MyRobotNode::aruco_cam_sub_cb(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){

   if(!msg->marker_ids.empty()){
    aruco_broadcaster(msg);

    aruco_listen_transform("odom", "aruco_mark");  
    base_link_listen_transform("odom", "base_link");

   }
    else{
   
     RCLCPP_INFO_STREAM(this->get_logger(),"NO Aruco marker FOUND ");
    }

}
void MyRobotNode::part_cam_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    if(!msg->part_poses.empty()){
    RCLCPP_INFO_STREAM(this->get_logger(),"Part Advance Camera callback : ");

   
    
    part_broadcaster(msg);

    part_listen_transform("odom", "part");   
    }
    else{    RCLCPP_INFO_STREAM(this->get_logger(),"NO Part Detected : ");}

}

void MyRobotNode::odom_sub_cb(nav_msgs::msg::Odometry::SharedPtr msg){
    tf2::Quaternion q( 
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    std::array<double, 3> euler = utils_ptr_->set_euler_from_quaternion(q);
    //RCLCPP_INFO_STREAM(this->get_logger(), "Euler angles : "<< euler[0]<<","<<euler[1]<<","<<euler[2]<<"\n");
    current_yaw_=euler[2];




}

double MyRobotNode::convertToMinusPiToPi(double radians) {
    // Ensure that radians is between -2*pi and 2*pi
    radians = fmod(radians, 2 * M_PI);

    // Map the angle to the range (-pi, pi]
    if (radians <= -M_PI) {
        radians += 2 * M_PI;
    } else if (radians > M_PI) {
        radians -= 2 * M_PI;
    }

    return radians;
}

std::string MyRobotNode::num2color(int c){
    switch(c){
        case 1:
        return "green";
        break;
        case 2:
        return "blue";
        break;
        case 3:
        return "orange";
        break;
        default:
        return "invalid";
        break;
        
    }
}


void MyRobotNode::cmd_val_pub_cb(){
    geometry_msgs::msg::Twist linear;
    double dist=MyRobotNode::distance(aruco_x_pos_,aruco_y_pos_,base_link_x_pos_,base_link_y_pos_);
    double kp= 0.8;
    
    double rotate_rad;

    switch (marker_id_)
    {
    case 0:
        rotate_rad = -80*M_PI/180;
        break;
    case 1:
        rotate_rad = 80*M_PI/180;
        break;
    case 2:
        rotate_rad = 0*M_PI/180;
        end_flag_=true;
        break;
    
    default:
        break;
    }



    if(dist>=1.0 && flag1_==true){
        
        linear.angular.z=0.0*(old_rad_-current_yaw_);
        linear.linear.x = 0.2;
        RCLCPP_INFO_STREAM(this->get_logger(),"Going_forward by 0.1m/s, dist: "<< dist);
        target_rad_=current_yaw_+rotate_rad;
        target_rad_=convertToMinusPiToPi(target_rad_);
        if (end_flag_==true){
            end_flag2_=true;
        }



        
    }
    else{
         linear.linear.x = 0.0;
         RCLCPP_INFO_STREAM(this->get_logger(),"Stopped ,, dist: "<< dist);
        flag1_=false;
        linear.angular.z=kp*(target_rad_-current_yaw_);
        cmd_val_publisher_->publish(linear);
        RCLCPP_INFO_STREAM(this->get_logger(),"NOWW TURNING, : "<< current_yaw_);
        if(abs(target_rad_-current_yaw_)<0.01){
            flag1_=true; old_rad_=target_rad_;
            if (end_flag2_==true){
                linear.linear.x = 0.0;
                flag1_=false;
                RCLCPP_INFO_STREAM(this->get_logger(),"You are at your destination " );
                for (long unsigned int i=0;i<parts_vector_.size();++i){
                    //std::string str=num2color(parts_vector_[i][0]);
                    RCLCPP_INFO_STREAM(this->get_logger(),num2color(parts_vector_[i][0]).c_str()<<" battery detected at xyz=["<<parts_vector_[i][1]<<","
                                                                                                            <<parts_vector_[i][2]<<","
                                                                                                            <<parts_vector_[i][3]<<"] rpy=["
                                                                                                            <<parts_vector_[i][4]<<","
                                                                                                            <<parts_vector_[i][5]<<","
                                                                                                            <<parts_vector_[i][6]<<"]");
                }
            }
        }
        
        
        
    }
    
    
    cmd_val_publisher_->publish(linear);
}
   
double MyRobotNode::distance(double x1,double y1,double x2, double y2){
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyRobotNode>("my_robot_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
}