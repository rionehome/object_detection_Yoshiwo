#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse
import os

def detectOn(req):
	rospy.loginfo("ON")
	os.system("rosrun yoshiwo_pivate_lesson darknet.py")
	os.system("rosservice call /detect_off")
	return EmptyResponse()

def detectOff(req):
	rospy.loginfo("OFF")
	return EmptyResponse()
	
def server():
	rospy.init_node("server")
	s1 = rospy.Service("detect_on", Empty, detectOn)
	s2 = rospy.Service("detect_off", Empty, detectOff)
	print("waiting")
	rospy.spin()
	
if __name__=='__main__':
	server()
