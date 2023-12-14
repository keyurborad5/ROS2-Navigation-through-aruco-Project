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


// Broadcaster to assign frame for the aurco marker
void MyRobotNode::aruco_broadcaster(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{  
    //A logic to detect the aruco marker which is closer to the camera
    double small_val=100000;
    long unsigned j=0;
    //RCLCPP_INFO_STREAM(this->get_logger(),"number of markerID : "<<msg->marker_ids.size());
    if(msg->marker_ids.size()!=0){
        for( long unsigned int i=0 ; i<msg->marker_ids.size();++i){
                
                if(small_val>msg->poses[i].position.z){
                    small_val=msg->poses[i].position.z;
                    j=i;
                }
        }
            // RCLCPP_INFO_STREAM(this->get_logger(),"closest distance: "<<msg->poses[j].position.z);
    }

    //an attribute that would be needed to decide the direction of turn
    marker_id_=msg->marker_ids[j];

    geometry_msgs::msg::TransformStamped aruco_dynamic_transform_stamped;

    //  RCLCPP_INFO(this->get_logger(), "Broadcasting dynamic_frame");
    aruco_dynamic_transform_stamped.header.stamp = msg->header.stamp;

    aruco_dynamic_transform_stamped.header.frame_id = msg->header.frame_id;
    aruco_dynamic_transform_stamped.child_frame_id = "aruco_mark";

    aruco_dynamic_transform_stamped.transform.translation.x = msg->poses[j].position.x;
    aruco_dynamic_transform_stamped.transform.translation.y = msg->poses[j].position.y;
    aruco_dynamic_transform_stamped.transform.translation.z = msg->poses[j].position.z;

    aruco_dynamic_transform_stamped.transform.rotation.x = msg->poses[j].orientation.x;
    aruco_dynamic_transform_stamped.transform.rotation.y = msg->poses[j].orientation.y;
    aruco_dynamic_transform_stamped.transform.rotation.z = msg->poses[j].orientation.z;
    aruco_dynamic_transform_stamped.transform.rotation.w = msg->poses[j].orientation.w;
    // Send the transform
    aruco_tf_broadcaster_->sendTransform(aruco_dynamic_transform_stamped);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Aruco Broadcasting_dynamic_frame : "<<msg->poses[0].position.x);
}

//A broadcaster to assign a frame for the detected object
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

    part_dynamic_transform_stamped.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
    part_dynamic_transform_stamped.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
    part_dynamic_transform_stamped.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
    part_dynamic_transform_stamped.transform.rotation.w = msg->part_poses[0].pose.orientation.w;
    // Send the transform
    part_tf_broadcaster_->sendTransform(part_dynamic_transform_stamped);
    // RCLCPP_INFO_STREAM(this->get_logger(), "PART advance cam Broadcasting_dynamic_frame : "<<msg->part_poses[0].pose.position.x;);
}

// A listerner that would tansform a detected arucomarker in odom frame
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

    // initialising this attributes that could be used in a diatance method to calc distance between robot and aruco marker
    aruco_x_pos_ = t_stamped.transform.translation.x;
    aruco_y_pos_ = t_stamped.transform.translation.y;

}

// A listerner that would tansform a base_link frame in odom frame
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

    // initialising this attributes that could be used in a diatance method to calc distance between robot and aruco marke
    base_link_x_pos_ = t_stamped.transform.translation.x;
    base_link_y_pos_ = t_stamped.transform.translation.y;
}

// A listerner that would tansform a detected part in odom frame
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

    //converting detected geometery quaternion to tf2 quaternion
    tf2::Quaternion q( 
        pose_out.orientation.x,
        pose_out.orientation.y,
        pose_out.orientation.z,
        pose_out.orientation.w);
    //Utlising Utlis to convert quaternion to rpy
    std::array<double, 3> euler = utils_ptr_->set_euler_from_quaternion(q);
    //initialising local variable as vector of data of detected object
    std::vector<double> detection={part_color_,pose_out.position.x,pose_out.position.y,pose_out.position.z,euler[0],euler[1],euler[2]};
    //passing the above vector to logg all the detected parts
    detected_part_locations(detection);
    

    // RCLCPP_INFO_STREAM(this->get_logger(), target_frame << " in " << source_frame << ":\n"
    //                                                     << "x: " << pose_out.position.x << "\t"
    //                                                     << "y: " << pose_out.position.y << "\t"
    //                                                     << "z: " << pose_out.position.z << "\n"
    //                                                     << "qx: " << pose_out.orientation.x << "\t"
    //                                                     << "qy: " << pose_out.orientation.y << "\t"
    //                                                     << "qz: " << pose_out.orientation.z << "\t"
    //                                                     << "qw: " << pose_out.orientation.w << "\n");
}

// Method to logg the detected parts location in odom frame without repeatation
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
    
 
//Aruco camera callback method
void MyRobotNode::aruco_cam_sub_cb(ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){

   if(!msg->marker_ids.empty()){//to check if the topic is pusblishing null
    //once detected marker, then we can broadcast a new frame at the detected location
    aruco_broadcaster(msg);

    //after broadcasting we can listen the aruco frame wrt to any frame ,here we want wrt odom frame
    aruco_listen_transform("odom", "aruco_mark");  
    //listen the base_link frame wrt to odom frame to get the location of robot
    base_link_listen_transform("odom", "base_link");

   }
    else{
   
     RCLCPP_INFO_STREAM(this->get_logger(),"NO Aruco marker FOUND ");
    }

}

//Part's Advanced logical camera callback method
void MyRobotNode::part_cam_sub_cb(mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
    
    if(!msg->part_poses.empty()){//Very necesaary otherwise node will die by it self; checking if topic is publishing or not
    
    //RCLCPP_INFO_STREAM(this->get_logger(),"Part Advance Camera callback : ");

    //once detected part, then we can broadcast a new frame at the detected location 
    part_broadcaster(msg);
    //after broadcasting we can listen the part frame wrt to any frame ,here we want wrt odom frame
    part_listen_transform("odom", "part");   
    }
    else{    //RCLCPP_INFO_STREAM(this->get_logger(),"NO Part Detected : ");
    }

}

//Callback method to get the location and orientaton of the robot
void MyRobotNode::odom_sub_cb(nav_msgs::msg::Odometry::SharedPtr msg){
    tf2::Quaternion q( 
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    //converting quaternion to euler rpy
    std::array<double, 3> euler = utils_ptr_->set_euler_from_quaternion(q);
    //RCLCPP_INFO_STREAM(this->get_logger(), "Euler angles : "<< euler[0]<<","<<euler[1]<<","<<euler[2]<<"\n");
    //Assigning Z rotation value to a attribute that would be used to make turns
    current_yaw_=euler[2];
}

//Method to calculate eular distance
double MyRobotNode::distance(double x1,double y1,double x2, double y2){
    return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}

//Method to convert any given radians in between -pi to pi
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

//Method to convert the color_type attribute of part to actual color
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

//callback function to publlish command velocity to the robot
void MyRobotNode::cmd_val_pub_cb(){
    geometry_msgs::msg::Twist move;
    //implementing distance method to cal didtanc btw aruco markera and robot
    double dist=MyRobotNode::distance(aruco_x_pos_,aruco_y_pos_,base_link_x_pos_,base_link_y_pos_);
    //declaring local propotional gain for making turn
    double kp= 0.8;
    
    double rotate_rad;

    //using to make decision of direction of turns
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
        end_flag_=true;// flag to indicate the end of maze
        break;
    
    default:
        break;
    }


    // Implementing logic of movement
    if(dist>=1.0 && flag1_==true){//linear movement
        
        move.angular.z=0.0;
        move.linear.x = 0.2;
        RCLCPP_INFO_STREAM(this->get_logger(),"Going_forward by 0.1m/s, dist: "<< dist);
        target_rad_=current_yaw_+rotate_rad;
        target_rad_=convertToMinusPiToPi(target_rad_);
        if (end_flag_==true){
            end_flag2_=true;
        }



        
    }
    else{// rotational movement
         move.linear.x = 0.0;
        //  RCLCPP_INFO_STREAM(this->get_logger(),"Stopped ,, dist: "<< dist);
        flag1_=false;
        move.angular.z=kp*(target_rad_-current_yaw_);//send command for rotation
        //cmd_val_publisher_->publish(move);
        RCLCPP_INFO_STREAM(this->get_logger(),"NOWW TURNING, : "<< current_yaw_);
        if(abs(target_rad_-current_yaw_)<0.01){//condition of stop turning
            flag1_=true; 
            if (end_flag2_==true){//flag for indicating end of program
                move.linear.x = 0.0;
                flag1_=false;
                RCLCPP_INFO_STREAM(this->get_logger(),"You are at your destination " );
                for (long unsigned int i=0;i<parts_vector_.size();++i){
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
    
    //publishing the command velocity
    cmd_val_publisher_->publish(move);
}
   


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyRobotNode>("my_robot_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
}