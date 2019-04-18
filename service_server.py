#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse
import os

def service1(req):
	rospy.loginfo("ON")
	os.system("python darknet_python/darknet.py")
	return EmptyResponse()

def service2(req):
	rospy.loginfo("OFF")
	return EmptyResponse()
	
def server():
	rospy.init_node("server")
	s1 = rospy.Service("call_on", Empty, service1)
	s2 = rospy.Service("call_off", Empty, service2)
	print("waiting")
	rospy.spin()
	
if __name__=='__main__':
	server()
