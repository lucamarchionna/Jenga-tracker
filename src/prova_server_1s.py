#!/usr/bin/env python3

import rospy
from tracker_visp.srv import FirstLayerPose
from tracker_visp.msg import ReferenceBlock
from std_msgs.msg import Bool

search_top=True
search_bottom=False

def service_handle(req):
  global search_top
  global search_bottom
  rospy.loginfo("Service called")
  found_top=req.found_top.data
  found_bottom=req.found_bottom.data
  cTlayer1=req.cTo
  print(found_top)
  print(found_bottom)
  print(cTlayer1)
  ready=Bool()
  if search_top and found_top:
    ready.data=True 
    search_top=False
    search_bottom=True
    print("Found top, searching bottom")
  elif search_bottom and found_bottom:
    ready.data=False
    search_top=False
    search_bottom=False
    print("Found top and then bottom, Finish")
  else:
    ready.data=True
  return ready

if __name__ == "__main__":
  rospy.init_node('Ros_server_motion')
  s=rospy.Service('/FirstLayerPose',FirstLayerPose,service_handle)
  rospy.loginfo("Start service, waiting...")
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo('Shutting down...')