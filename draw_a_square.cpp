/*
* Inspired by: http://wiki.ros.org/mini_max/Tutorials/Moving%20the%20Base
* Turtlebot draws a ~0.5m square until you ctrl + c
* Tested using TurtleBot 2, ROS Indigo, Ubuntu 14.04
*/

#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
    //init the ROS node
    ROS_INFO_STREAM("Draw a 0.5m square.  Repeat until ctrl + c");
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle nh;

    //init publisher
    ros::Publisher cmd_vel_pub_;
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    
    //init direction that turtlebot should go
    geometry_msgs::Twist base_cmd;
    geometry_msgs::Twist base_cmd_turn_left;

    //defaults
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    base_cmd_turn_left.linear.x = base_cmd_turn_left.linear.y = base_cmd_turn_left.angular.z = 0;
    

    //for 2d movement there are only two factors in play.  linear.x (speed) and angular.z (left or right; 0 is neutral)
    base_cmd.linear.x = 0.25;
    base_cmd.angular.z = 0; //base_cmd will be used to go forward so set a neutral z value
    
    //base_cmd_turn_left will be used to turn turtlebot 90 degrees
    base_cmd_turn_left.linear.x = 0.25; //m/s
    base_cmd_turn_left.angular.z = 1.57/2; //45 deg/s * 2 sec = 90 degrees 

    ros::Rate r(5.0);

    while(nh.ok()) { //have we ctrl + C?  If no... keep going!
        //"publish" sends the command to turtlebot to keep going

        //go forward for 2 seconds
        for(int n=10; n>0; n--) {
		cmd_vel_pub_.publish(base_cmd);
		r.sleep();
	}

        //turn 90 degrees (takes 2 seconds)
    	for(int n=10; n>0; n--) {
                cmd_vel_pub_.publish(base_cmd_turn_left);
                r.sleep();
        }

    }
}

