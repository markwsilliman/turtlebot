#!/usr/bin/env python

'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

'''

#Coffee Bot

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
import json
import urllib2
import time #for sleep()
import roslib
from kobuki_msgs.msg import PowerSystemEvent, AutoDockingAction, AutoDockingGoal, SensorState #for kobuki base power and auto docking
from kobuki_msgs.msg import ButtonEvent #for kobuki base's b0 button
from smart_battery_msgs.msg import SmartBatteryStatus #for netbook battery
import math #for comparing if Kobuki's power has changed using fabs

class turtlebot_coffee():
    ######## CHANGE THE FOLLOWING VALUES #########
    server_public_dns = 'http://ec2-54-200-33-28.us-west-2.compute.amazonaws.com' #must start with http:// .  Don't include a trailing "/"
    near_docking_station_x = -1.88 #x coordinate for pose approx 1 meter from docking station
    near_docking_station_y = 0.06 #y coordinate for pose approx 1 meter from docking station    
    ######## END CHANGE THE FOLLOWING VALUES #########

    ####### OPTIONVAL VALUES TO CHANGE ##########
    kobuki_base_max_charge = 160 
    #we're using the extended battery.  Your battery may have a different max charge value.  The maximum charge for kobuki base can be determined by running:
    #rostopic echo /mobile_base/sensors/core 
    #and viewing the "battery" value.  This value is used to determine kobuki's battery status %.
    ####### END OPTIONVAL VALUES TO CHANGE ##########
    
    # defaults
    move_base = False # _init_ converts this to a MoveBaseAction that is used to set the goals
    battery_is_low = False # is kobuki's battery low?
    battery_is_low_netbook = False # is the notebook's battery low?
    netbook_previous_battery_level = 100 # what was the previous netbook battery level (used to know if it's changed)
    kobuki_previous_battery_level = 1000 #1000 isn't possible.  Just a large fake # so the script starts believing the battery is fine
    charging_at_dock_station = False #can't leave docking station until it's full because battery was low
    proactive_charging_at_dock_station = False #can leave docking station as soon as a coffee request comes in because battery is fine
    count_no_one_needs_coffee_in_a_row = 0 #keeps track of how many times in a row we receive "no one needs coffee".  If there is considerable down time TurtleBot will return to the docking station to proactively charge itself.
    how_many_no_one_needs_coffee_before_proactive_charging = 5 # how many times should we receive "no one needs coffee" prior to proactively returning to the docking station
    cannot_move_until_b0_is_pressed = False # should TurtleBot stay still until B0 is pressed (e.g. while the person is brewing coffee)?

    def __init__(self):
	#initialize ros node
        rospy.init_node('turtlebot_coffee', anonymous=False)

	#what to do if shut down (e.g. ctrl + C or failure)
	rospy.on_shutdown(self.shutdown)
	
	#tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	rospy.loginfo("wait for the action server to come up")
	#allow up to 30 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(30))

	#monitor Kobuki's power and charging status.  If an event occurs (low battery, charging, not charging etc) call function SensorPowerEventCallback
	rospy.Subscriber("/mobile_base/sensors/core",SensorState,self.SensorPowerEventCallback)

	#monitor netbook's battery power
	rospy.Subscriber("/laptop_charge/",SmartBatteryStatus,self.NetbookPowerEventCallback)

	#to avoid TurtleBot from driving to another pose while someone is making coffee ... TurtleBot isn't allowed to move until the person presses the B0 button.  To implement this we need to monitor the kobuki button events
	rospy.Subscriber("/mobile_base/events/button",ButtonEvent,self.ButtonEventCallback)

    def deliver_coffee(self):
	#if someone is currently making coffee don't move!
	if(self.cannot_move_until_b0_is_pressed):
		rospy.loginfo("Waiting for button B0 to be pressed.")
		time.sleep(2)
		return True

	#before we deliver the next coffee... how is power looking? If low go recharge first at the docking station.
	if(self.INeedPower()):
		return True

	#Power is fine so let's see if anyone needs coffee...
	rospy.loginfo("Anyone need coffee?")
	#we'll send a goal to the robot to tell it to move to a pose that's near the docking station
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	
	#call the server and "pop" the next pending customer's pose (if one is pending) from the stack
	data = json.load(urllib2.urlopen(self.server_public_dns + "/turtlebot-server/coffee_queue.php?pop"))
	if(data["status"] == "pending"): #someone is pending coffee!  Oh ya... let's get moving
		#If we're at the charging station back up 0.2 meters to avoid collision with dock
		self.DoWeNeedToBackUpFromChargingStation()

		#Where are they?  Set the person's pose
		goal.target_pose.pose = Pose(Point(float(data["point"]["x"]), float(data["point"]["y"]), float(data["point"]["z"])), Quaternion(float(data["quat"]["x"]), float(data["quat"]["y"]), float(data["quat"]["z"]), float(data["quat"]["w"])))

		#start moving
        	self.move_base.send_goal(goal)

		#allow TurtleBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60)) 

	
		if not success:
			#failed to reach goal (e.g. TurtleBot can't find a way to go to the location)
        	        self.move_base.cancel_goal()
        	        rospy.loginfo("The base failed to reach the desired pose")
			#tell the server that this pose failed (so it won't try it again)
			data = json.load(urllib2.urlopen(self.server_public_dns + "/turtlebot-server/coffee_queue.php?update&id=" + data["id"] + "&status=failed"))
    		else:
			# We made it!
			state = self.move_base.get_state()
			if state == GoalStatus.SUCCEEDED:
			    rospy.loginfo("Hooray, reached the desired pose!  Press B0 to allow TurtleBot to continue.")
			    #tell TurtleBot not to move until the customer presses B0
			    self.cannot_move_until_b0_is_pressed = True
			    self.count_no_one_needs_coffee_in_a_row = 0 #reset to 0
			    #tell the server that the pose was completed
			    data = json.load(urllib2.urlopen(self.server_public_dns + "/turtlebot-server/coffee_queue.php?update&id=" + data["id"] + "&status=complete"))

	else: #no one needs coffee :(		
		self.count_no_one_needs_coffee_in_a_row = self.count_no_one_needs_coffee_in_a_row + 1 #increment so we know how many times in a row no one needed coffee
		rospy.loginfo("No one needs coffee #" + str(self.count_no_one_needs_coffee_in_a_row))
		#considering there is nothing to do... should we charge?
		if(self.count_no_one_needs_coffee_in_a_row > self.how_many_no_one_needs_coffee_before_proactive_charging and not self.charging_at_dock_station):
			rospy.loginfo("Battery is fine but considering no one wants coffee ... Going to docking station.")
			self.DockWithChargingStation() #tell TurtleBot to dock with the charging station
			self.proactive_charging_at_dock_station = True
		else:
			time.sleep(2) #wait 2 seconds before asking the server if there are pending coffee needs

	return True

    def ButtonEventCallback(self,data):
	#From https://github.com/yujinrobot/kobuki/blob/f99e495b2b3be1e62495119809c58ccb58909f67/kobuki_testsuite/scripts/test_events.py
	if ( data.button == ButtonEvent.Button0 ) :
		self.cannot_move_until_b0_is_pressed = False

    def DoWeNeedToBackUpFromChargingStation(self):
	#if you set a goal while it's docked it tends to run into the docking station while turning.  Tell it to back up a little before initiliazing goals.
	if(self.proactive_charging_at_dock_station or self.charging_at_dock_station):
		rospy.loginfo("We're at the docking station.  Back up before next goal.")
		self.proactive_charging_at_dock_station = False
		cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
		# Twist is a datatype for velocity
		move_cmd = Twist()
		# let's go forward at 0.1 m/s
		move_cmd.linear.x = -0.1
		# let's turn at 0 radians/s
		move_cmd.angular.z = 0

		r = rospy.Rate(10);
		# as long as you haven't ctrl + c keeping doing...
		temp_count = 0
		#go back at 0.1 m/s for 2 seconds
		while (not rospy.is_shutdown() and temp_count < 20):
			# publish the velocity
			cmd_vel.publish(move_cmd)
			# wait for 0.1 seconds (10 HZ) and publish again
			temp_count = temp_count + 1
			r.sleep()
		#make sure TurtleBot stops by sending a default Twist()
		cmd_vel.publish(Twist())
		return True

    def INeedPower(self):
	#are we currently charging at the docking station?  If yes only continue if we're not fully charged
	if(self.charging_at_dock_station and (self.battery_is_low or self.battery_is_low_netbook)):
		rospy.loginfo("I'm charging and will continue when I'm sufficiently charged")
		time.sleep(30)
		return True
	#are we not currently charging and is either battery low?  If yes, go to docking station.
	if(not self.charging_at_dock_station and (self.battery_is_low or self.battery_is_low_netbook)):
		rospy.loginfo("Battery is low. Going to docking station.")
		self.DockWithChargingStation() #tell TurtleBot to dock with the charging station
		return True
	return False

    def SensorPowerEventCallback(self,data):
	#kobuki's batttery value tends to bounce up and down 1 constantly so only report if difference greater than 1
	if(math.fabs(int(data.battery) - self.kobuki_previous_battery_level) > 2):
		rospy.loginfo("Kobuki's battery is now: " + str(round(float(data.battery) / float(self.kobuki_base_max_charge) * 100)) + "%")
		self.kobuki_previous_battery_level = int(data.battery)

	if(int(data.charger) == 0) :
		if(self.charging_at_dock_station):
			rospy.loginfo("Stopped charging at docking station")
		self.charging_at_dock_station = False
	else:
		if(not self.charging_at_dock_station):
			rospy.loginfo("Charging at docking station")
		self.charging_at_dock_station = True

	
	if ( round(float(data.battery) / float(self.kobuki_base_max_charge) * 100) < 50) :
		if(not self.battery_is_low):
			rospy.loginfo("Kobuki battery is low")
		self.battery_is_low = True
	elif ( round(float(data.battery) / float(self.kobuki_base_max_charge) * 100) > 60): #the logic of not using the same value (e.g. 50) for both the battery is low & battery is fine is that it'll leave and immediatly return for more power.  The reason why we don't use == 100 is that we hope that proactive charging between coffee deliveries will charge it soon and we don't want people waiting.
		if(self.battery_is_low):
			rospy.loginfo("Kobuki battery is fine")
		self.battery_is_low = False

		
    def NetbookPowerEventCallback(self,data):
	#has the netbook's power level changed?
	if(int(data.percentage) != self.netbook_previous_battery_level):
		rospy.loginfo("Notebook's battery is now: " + str(data.percentage) + "%")
		self.netbook_previous_battery_level = int(data.percentage)
	#is the netbook's power low?
	if(int(data.percentage) < 50): #50 is the percent of total power
		self.battery_is_low_netbook = True
	elif(int(data.percentage) > 60): #the logic of not using the same value (e.g. 50) for both the battery is low & battery is fine is that it'll leave and immediatly return for more power.  The reason why we don't use == 100 is that we hope that proactive charging between coffee deliveries will charge it soon and we don't want people waiting.
		self.battery_is_low_netbook = False

    def DockWithChargingStation(self):
	#before we can run auto-docking we need to be close to the docking station..
	if(not self.GoCloseToTheChargingStation()):
		return False
	
	#We're close to the docking station... so let's dock
	return self.WereCloseDock()

    def WereCloseDock(self):
	#The following will start the AutoDockingAction which will automatically find and dock TurtleBot with the docking station as long as it's near the docking station when started
	self._client = actionlib.SimpleActionClient('/dock_drive_action', AutoDockingAction)
	rospy.loginfo("waiting for auto_docking server")
	self._client.wait_for_server()
	rospy.loginfo("auto_docking server found")
	goal = AutoDockingGoal()
	rospy.loginfo("Sending auto_docking goal and waiting for result (times out in 180 seconds and will try again if required)")
	self._client.send_goal(goal)

	#Give the auto docking script 180 seconds.  It can take a while if it retries.
	success = self._client.wait_for_result(rospy.Duration(180))

	if success:
		rospy.loginfo("Auto_docking succeeded")
		self.charging_at_dock_station = True #The callback which detects the docking status can take up to 3 seconds to update which was causing coffee bot to try and redock (presuming it failed) even when the dock was successful.  Therefore hardcoding this value after success.
		return True
	else:
		rospy.loginfo("Auto_docking failed")
		return False
		

    def GoCloseToTheChargingStation(self):
	#the auto docking script works well as long as you are roughly 1 meter from the docking station.  So let's get close first...
	rospy.loginfo("Let's go near the docking station")

	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()

	#set a Pose near the docking station
	goal.target_pose.pose = Pose(Point(float(self.near_docking_station_x), float(self.near_docking_station_y), float(0)), Quaternion(float(0), float(0), float(0.892), float(-1.5)))

	#start moving
	self.move_base.send_goal(goal)

	#allow TurtleBot up to 60 seconds to get close to 
	success = self.move_base.wait_for_result(rospy.Duration(60)) 

	if not success:
	        self.move_base.cancel_goal()
	        rospy.loginfo("The base failed to reach the desired pose near the charging station")
		return False
	else:
		# We made it!
		state = self.move_base.get_state()
		if state == GoalStatus.SUCCEEDED:
		    rospy.loginfo("Hooray, reached the desired pose near the charging station")
		    return True



    def shutdown(self):
        rospy.loginfo("Stop")


if __name__ == '__main__':
    delivery_checks = 0 #just for troubleshooting to see how many times we called the server to check for pending coffee
    try:
        coffebot = turtlebot_coffee()
        #keep checking for deliver_coffee until we shutdown the script with ctrl + c
	while(coffebot.deliver_coffee() and not rospy.is_shutdown()):
                delivery_checks = delivery_checks + 1

    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")

