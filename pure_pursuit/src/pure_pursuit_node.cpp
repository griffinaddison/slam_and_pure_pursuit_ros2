#include <sstream>
#include <string>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <memory>
#include <functional>


/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

using std::placeholders::_1;

using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose;

    rclcpp::Node::SharedPtr node;

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // TODO: create ROS subscribers and publishers

        pub_marker = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);
        sub_pose = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 100 , std::bind(&PurePursuit::pose_callback, this, _1));


    }

    // void pose_callback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& pose_msg)  
    // void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg) //stub code had &pose_msg, the & caused build errors. also said ConstPtr instead of ConstSharedPtr, which also made errors
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) //stub code had &pose_msg, the & caused build errors. also said ConstPtr instead of ConstSharedPtr, which also made errors

    {

        RCLCPP_INFO(this->get_logger(), "Published Marker");



        // MARKERS
        visualization_msgs::msg::Marker marker1;
        marker1.header.frame_id = "map";
        // marker1.header.stamp = ros::Time::now();
        marker1.type = visualization_msgs::msg::Marker::SPHERE;
        marker1.action = visualization_msgs::msg::Marker::MODIFY;
        marker1.pose.position.x = 1.0;
        marker1.pose.position.y = 2.0;
        marker1.pose.position.z = 0.5;
        marker1.pose.orientation.x = 0.0;
        marker1.pose.orientation.y = 0.0;
        marker1.pose.orientation.z = 0.0;
        marker1.pose.orientation.w = 1.0;
        marker1.scale.x = 1.0;
        marker1.scale.y = 1.0;
        marker1.scale.z = 1.0;
        marker1.color.a = 1.0;
        marker1.color.r = 0.0;
        marker1.color.g = 0.0;
        marker1.color.b = 0.0;

        pub_marker->publish(marker1);

        // RCLCPP_DEBUG(this->get_logger(), "Debug message");




        // TODO: find the current waypoint to track using methods mentioned in lecture

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle.
    }

    ~PurePursuit() {} // destructor, which is called when the object is destroyed,
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::get_logger("rclcpp").set_level(rclcpp::LoggingSeverity::Debug);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}