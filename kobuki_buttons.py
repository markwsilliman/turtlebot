#!/usr/bin/env python

# This script is inspired by https://github.com/yujinrobot/kobuki/blob/f99e495b2b3be1e62495119809c58ccb58909f67/kobuki_testsuite/scripts/test_events.py
# They deserve all the credit therefore I'm including their copyright notice / BSD - Mark Silliman

'''
# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Yujin Robot
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Yujin Robot nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
'''

# Monitor the kobuki's button status

import roslib
import rospy
from kobuki_msgs.msg import ButtonEvent

class kobuki_button():

	def __init__(self):
		rospy.init_node("kobuki_button")		

		#monitor kobuki's button events
		rospy.Subscriber("/mobile_base/events/button",ButtonEvent,self.ButtonEventCallback)

		#rospy.spin() tells the program to not exit until you press ctrl + c.  If this wasn't there... it'd subscribe and then immediatly exit (therefore stop "listening" to the thread).
		rospy.spin();


	
	def ButtonEventCallback(self,data):
	    if ( data.state == ButtonEvent.RELEASED ) :
		state = "released"
	    else:
		state = "pressed"  
	    if ( data.button == ButtonEvent.Button0 ) :
		button = "B0"
	    elif ( data.button == ButtonEvent.Button1 ) :
		button = "B1"
	    else:
		button = "B2"
	    rospy.loginfo("Button %s was %s."%(button, state))
	

if __name__ == '__main__':
	try:
		kobuki_button()
	except rospy.ROSInterruptException:
		rospy.loginfo("exception")
