#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class LetsCrash():
    def __init__(self):
        # initiliaze
        rospy.init_node('lets_crash', anonymous=False)

        # What to do you ctrl + c    
        rospy.on_shutdown(self.shutdown)
        
        # You may need to change cmd_vel_mux/input/teleop to /cmd_vel if you're not using TurtleBot2
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/teleop', Twist)
     
        r = rospy.Rate(10);

        # let's go forward at 0.2 m/s
        move_cmd = Twist()
        move_cmd.linear.x = 0.2

        while not rospy.is_shutdown():
            self.cmd_vel.publish(move_cmd)
            r.sleep()
                        
        
    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop Crashing")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)
 
if __name__ == '__main__':
    try:
        LetsCrash()
    except:
        rospy.loginfo("Let's crash node terminated.")

