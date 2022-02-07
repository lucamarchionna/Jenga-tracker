#!/usr/bin/env python3

import rospy
from tracker_visp.srv import FirstLayerPose
from tracker_visp.msg import ReferenceBlock
from std_msgs.msg import Bool

def service_handle(req):
  rospy.loginfo("Service called")
  found=req.found
  cTlayer1=req.cTlayer1
  print(found.data)
  print(cTlayer1)
  ready=Bool()
  if found.data:
    ready.data=True 
  else:
    ready.data=False
  return ready

if __name__ == "__main__":
  rospy.init_node('Ros_server_motion')
  s=rospy.Service('/FirstLayerPose',FirstLayerPose,service_handle)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo('Shutting down...')