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
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <chrono>
#include <memory>
#include <functional>


//libraries for reading csv
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include <Eigen/Dense>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2/LinearMath/Matrix3x3.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>
//import euler_from_quaternion
#include <tf2_eigen/tf2_eigen.h>




/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

using std::placeholders::_1;

using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_marker;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_pose;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive;
    std::vector<std::vector<float>> positions;

    rclcpp::Node::SharedPtr node;

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // TODO: create ROS subscribers and publishers

        pub_marker = this->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", 10);
        sub_pose = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 100 , std::bind(&PurePursuit::pose_callback, this, _1));
        pub_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        //read csv and convert to 2d float array
        std::ifstream file("/home/griffin/Documents/f1tenth_ws/src/lab-5-slam-and-pure-pursuit-team-10/pure_pursuit/src/interpolated_path.csv"); //make sure to place this file
        std::string line;

        // RCLCPP_INFO(this->get_logger(), "reading csv");

        if (!file)
        {
            RCLCPP_INFO(this->get_logger(), "file not found");
        }

        // RCLCPP_INFO(this->get_logger(), std::getline(file, line) ? "true" : "false"); //this line is just to check if the file is empty

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::vector<float> pos;

            std::string token;

            //handle case with column headers
            if (line[0] == 'x')
            {
                continue;
            }
            while (std::getline(ss, token, ','))
            {
                pos.push_back(std::stof(token));
            }

            // Add the x y position values to the 2D array
            positions.push_back({pos[0], pos[1]});

            // RCLCPP_INFO(this->get_logger(), "added: \tx: %f, \ty: %f", pos[0], pos[1]);
        }

        // RCLCPP_INFO(this->get_logger(), "done reading csv");

        //print out size of csv
        // RCLCPP_INFO(this->get_logger(), "size of csv: %d", positions.size());

        // Print out the x y positions
        // for (auto const& pos : positions)
        // {
        //     std::cout << "(" << pos[0] << ", " << pos[1] << ")\n";
        // }

        





    }

    // void pose_callback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped>& pose_msg)  
    // void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg) //stub code had &pose_msg, the & caused build errors. also said ConstPtr instead of ConstSharedPtr, which also made errors
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) //stub code had &pose_msg, the & caused build errors. also said ConstPtr instead of ConstSharedPtr, which also made errors

    {



        double car_x = pose_msg->pose.pose.position.x;
        double car_y = pose_msg->pose.pose.position.y;
        double car_z = pose_msg->pose.pose.position.z;


        //////////////////////////////////////// WAYPOINT MARKERS ////////////////////////////////////////

        //create the top level marker array
        visualization_msgs::msg::MarkerArray marker_array;
        marker_array.markers.resize(5);

        // Sphere Marker
        marker_array.markers[0].header.frame_id = "map";
        // marker_array.markers[0].header.stamp = node->now();
        marker_array.markers[0].id = 0;
        marker_array.markers[0].type = visualization_msgs::msg::Marker::SPHERE;
        marker_array.markers[0].action = visualization_msgs::msg::Marker::MODIFY;
        marker_array.markers[0].pose.position.x = 1.0;
        marker_array.markers[0].pose.position.y = 2.0;
        marker_array.markers[0].pose.position.z = 1.0;
        marker_array.markers[0].scale.x = 1.0;
        marker_array.markers[0].scale.y = 1.0;
        marker_array.markers[0].scale.z = 1.0;
        marker_array.markers[0].color.r = 1.0;
        marker_array.markers[0].color.g = 0.0;
        marker_array.markers[0].color.b = 0.0;
        marker_array.markers[0].color.a = 1.0;

        // CubeList Marker
        marker_array.markers[1].header.frame_id = "map";
        // marker_array.markers[1].header.stamp = node->now();
        marker_array.markers[1].id = 1;
        marker_array.markers[1].type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker_array.markers[1].action = visualization_msgs::msg::Marker::MODIFY; 
        marker_array.markers[1].scale.x = 0.1;
        marker_array.markers[1].scale.y = 0.1;
        marker_array.markers[1].scale.z = 0.1;
        marker_array.markers[1].color.r = 0.0;
        marker_array.markers[1].color.g = 1.0;
        marker_array.markers[1].color.b = 0.0;
        marker_array.markers[1].color.a = 1.0;


        // Add points to the CubeList marker
        for (std::vector<float> pos : positions) 
        {
            geometry_msgs::msg::Point point;
            point.x = pos[0];
            point.y = pos[1];
            point.z = 0.0;

            marker_array.markers[1].points.push_back(point);
        }



        ///////////////////////////////// WAYPOINT MARKERS END////////////////////////////////////////////




        // TODO: find the current waypoint to track using methods mentioned in lecture

        //pick the closest point to the car
        int closest_point_index = 0;
        double closest_point_distance = 99799.9;
        for (int i = 0; i < positions.size(); i++)
        {
            // RCLCPP_INFO(this->get_logger(), "iterating");
            double distance = sqrt(pow(positions[i][0] - car_x, 2) + pow(positions[i][1] - car_y, 2));
            if (distance < closest_point_distance)
            {
                closest_point_distance = distance;
                closest_point_index = i;
                // RCLCPP_INFO(this->get_logger(), "closest point index: %d", closest_point_index);
            }
        }
        //place a blue marker on this point and add it to the marker array
        marker_array.markers[2].header.frame_id = "map";
        // marker_array.markers[2].header.stamp = node->now();
        marker_array.markers[2].id = 2;
        marker_array.markers[2].type = visualization_msgs::msg::Marker::CUBE;
        marker_array.markers[2].action = visualization_msgs::msg::Marker::MODIFY;
        marker_array.markers[2].pose.position.x = positions[closest_point_index][0];
        marker_array.markers[2].pose.position.y = positions[closest_point_index][1];
        marker_array.markers[2].pose.position.z = 0.0;
        marker_array.markers[2].scale.x = 0.2;
        marker_array.markers[2].scale.y = 0.2;
        marker_array.markers[2].scale.z = 0.2;
        marker_array.markers[2].color.r = 0.0;
        marker_array.markers[2].color.g = 0.0;
        marker_array.markers[2].color.b = 1.0;
        marker_array.markers[2].color.a = 1.0;




        //now step forward in positions until we find the first point that is at least the lookahead distance away
        int lookahead_point_index = closest_point_index;
        double lookahead_distance = 1.0;
        double lookahead_point_distance = 0.0;
        while (lookahead_point_distance < lookahead_distance)
        {
            lookahead_point_index++;
            //wrap around if we reach the end of the array
            if (lookahead_point_index >= positions.size())
            {
                lookahead_point_index = 0;
            }
            lookahead_point_distance = sqrt(pow(positions[lookahead_point_index][0] - car_x, 2) + pow(positions[lookahead_point_index][1] - car_y, 2));
        }
        
        double goalPointX_map = positions[lookahead_point_index][0];
        double goalPointY_map = positions[lookahead_point_index][1];
        //place a green marker on this point and add it to the marker array
        marker_array.markers[3].header.frame_id = "map";
        // marker_array.markers[3].header.stamp = node->now();
        marker_array.markers[3].id = 3;
        marker_array.markers[3].type = visualization_msgs::msg::Marker::CUBE;
        marker_array.markers[3].action = visualization_msgs::msg::Marker::MODIFY;
        marker_array.markers[3].pose.position.x = positions[lookahead_point_index][0];
        marker_array.markers[3].pose.position.y = positions[lookahead_point_index][1];
        marker_array.markers[3].pose.position.z = 0.0;
        marker_array.markers[3].scale.x = 0.2;
        marker_array.markers[3].scale.y = 0.2;
        marker_array.markers[3].scale.z = 0.2;
        marker_array.markers[3].color.r = 0.5;
        marker_array.markers[3].color.g = 0.0;
        marker_array.markers[3].color.b = 0.5;
        marker_array.markers[3].color.a = 1.0;
        






    






        // //find the lookahead point in the vehicle frame of reference
        // double lookahead_point_x = positions[lookahead_point_index][0] - car_x;
        // double lookahead_point_y = positions[lookahead_point_index][1] - car_y;

        // //find the angle between the car's heading and the lookahead point
        // double angle = atan2(lookahead_point_y, lookahead_point_x) - car_yaw;

        // //find the distance between the car and the lookahead point
        // double distance = sqrt(pow(lookahead_point_x, 2) + pow(lookahead_point_y, 2));

        // //place a yellow marker at the lookahead point in the vehicle frame of reference
        // marker_array.markers[4].header.frame_id = "base_link";
        // // marker_array.markers[4].header.stamp = node->now();
        // marker_array.markers[4].id = 4;
        // marker_array.markers[4].type = visualization_msgs::msg::Marker::CUBE;
        // marker_array.markers[4].action = visualization_msgs::msg::Marker::MODIFY;
        // marker_array.markers[4].pose.position.x = distance;
        // marker_array.markers[4].pose.position.y = 0.0;
        // marker_array.markers[4].pose.position.z = 0.0;
        // marker_array.markers[4].scale.x = 0.2;
        // marker_array.markers[4].scale.y = 0.2;
        // marker_array.markers[4].scale.z = 0.2;
        // marker_array.markers[4].color.r = 1.0;
        // marker_array.markers[4].color.g = 1.0;
        // marker_array.markers[4].color.b = 0.0;
        // marker_array.markers[4].color.a = 1.0;





        // TODO: transform goal point to vehicle frame of reference

        //get car_yaw from odometry message
        double car_roll = 0.0;
        double car_pitch = 0.0;
        double car_yaw = 0.0;

        // tf2::Matrix3x3(pose_msg->pose.pose.orientation).getRPY(car_roll, car_pitch, car_yaw);
        // get roll pitch and yaw from quaternion
        tf2::Quaternion q(
            pose_msg->pose.pose.orientation.x,
            pose_msg->pose.pose.orientation.y,
            pose_msg->pose.pose.orientation.z,
            pose_msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(car_roll, car_pitch, car_yaw);
        


        //find homogeneous transform from map frame to vehicle frame
        Eigen::Matrix4d T_map_vehicle;
        T_map_vehicle << cos(car_yaw), -sin(car_yaw), 0, car_x,
            sin(car_yaw), cos(car_yaw), 0, car_y,
            0, 0, 1, 0,
            0, 0, 0, 1;

        //find homogeneous transform from vehicle frame to map frame
        Eigen::Matrix4d T_vehicle_map = T_map_vehicle.inverse();

        //find the homogeneous transform from map frame to goal point
        Eigen::Matrix4d T_map_goal;
        T_map_goal << 1, 0, 0, goalPointX_map,
            0, 1, 0, goalPointY_map,
            0, 0, 1, 0,
            0, 0, 0, 1;
        
        //find the homogeneous transform from vehicle frame to goal point
        Eigen::Matrix4d T_vehicle_goal = T_vehicle_map * T_map_goal;

        //print the current heading and distance of the goal point in the vehicle frame
        double goalPointX_car = T_vehicle_goal(0, 3);
        double goalPointY_car = T_vehicle_goal(1, 3);
        double goal_point_distance = sqrt(pow(goalPointX_car, 2) + pow(goalPointY_car, 2));
        double goal_point_angle = atan2(goalPointY_car, goalPointX_car);

        // RCLCPP_INFO(this->get_logger(), "goal:\t distance: %f, \theading: %f", goal_point_distance, car_yaw);
        // RCLCPP_INFO(node->get_logger(), "goal:\t \theading: %f", car_yaw);
        // RCLCPP_INFO(this->get_logger(), "goal: distance: %f", goal_point_distance);

        //to check, find transformation from map to goal point using car to goal point
        Eigen::Matrix4d T_map_goal_check = T_map_vehicle * T_vehicle_goal;

        //add a yellow marker at this point in the map frame
        marker_array.markers[4].header.frame_id = "map";
        // marker_array.markers[4].header.stamp = node->now();
        marker_array.markers[4].id = 4;
        marker_array.markers[4].type = visualization_msgs::msg::Marker::CUBE;
        marker_array.markers[4].action = visualization_msgs::msg::Marker::MODIFY;
        marker_array.markers[4].pose.position.x = T_map_goal_check(0, 3);
        marker_array.markers[4].pose.position.y = T_map_goal_check(1, 3);
        marker_array.markers[4].pose.position.z = 1.0;
        marker_array.markers[4].scale.x = 0.2;
        marker_array.markers[4].scale.y = 0.2;
        marker_array.markers[4].scale.z = 0.2;
        marker_array.markers[4].color.r = 1.0;
        marker_array.markers[4].color.g = 1.0;
        marker_array.markers[4].color.b = 0.0;
        marker_array.markers[4].color.a = 1.0;



        pub_marker->publish(marker_array);


        


        

        // TODO: calculate curvature/steering angle

        double lateral_displacement = T_vehicle_goal(1, 3);
        double curvature = (2 * lateral_displacement) / pow(lookahead_distance, 2);

        
    



        // TODO: publish drive message, don't forget to limit the steering angle.

        //create a drive message
        double wheel_base = 0.33;
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.frame_id = "base_link";
        // drive_msg.header.stamp = node->now();
        drive_msg.drive.steering_angle = atan(curvature * wheel_base);
        drive_msg.drive.speed = 3.0;

        //publish the drive message
        pub_drive->publish(drive_msg);


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