#! /usr/bin/env python
import rospy
from std_srvs.srv import Empty, EmptyResponse

def call_service():
	rospy.loginfo("wait for service")
	rospy.wait_for_service("detect_on")
	try:
		service = rospy.ServiceProxy("detect_on",Empty)
		response = service()
	except rospy.ServiceException, e:
		print("Service failed: %s" % e)
		
if __name__=='__main__':
	call_service()
