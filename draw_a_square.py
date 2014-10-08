#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class DrawASquare():
    def __init__(self):
        # initiliaze
        rospy.init_node('drawasquare', anonymous=False)

        # What to do you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        # You may need to change cmd_vel_mux/input/teleop to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
     
        r = rospy.Rate(5);

        # let's go forward at 0.2 m/s
        move_cmd = Twist()
        move_cmd.linear.x = 0.2

        #let's turn at 45 deg/s
        turn_cmd = Twist()
        turn_cmd.linear.x = 0.2
        turn_cmd.angular.z = 1.57/2; #45 deg/s in radians/s

        while not rospy.is_shutdown():
            for x in range(0,10):
                self.cmd_vel.publish(move_cmd)
                r.sleep()
            for x in range(0,10):
                self.cmd_vel.publish(turn_cmd)
                r.sleep()            
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Drawing Squares")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        DrawASquare()
    except:
        rospy.loginfo("node terminated.")

