/*
* Hello World and Crash for Turtlebot
* Moves Turtlebot forward until you ctrl + c
* Tested using TurtleBot 2, ROS Indigo, Ubuntu 14.04
*/

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    //init the ROS node
    ROS_INFO_STREAM("Hello World");
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;

    //init publisher
    ros::Publisher cmd_vel_pub_;
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    
    //init direction that turtlebot should go
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

    //and let's go forward by setting X to a positive value
    base_cmd.linear.x = 0.25;

    ROS_INFO_STREAM("And Crashing ... ctrl + c to stop me :)");

    while(nh.ok()) { //have we ctrl + C?  If no... keep going!
        //"publish" sends the command to turtlebot to keep going
    	cmd_vel_pub_.publish(base_cmd);
    }
}

