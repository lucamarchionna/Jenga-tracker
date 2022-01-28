#!/usr/bin/env python3

import cv_bridge
import rospy
import cv2
import numpy as np
from tracker_visp.srv import YolactInitializeCaoPose
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Transform
from std_msgs.msg import String
from cv_bridge import CvBridge
import pyrealsense2 as rs
import time

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams are possible
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) #color enabled

# Start streaming
profile=pipeline.start(config)

cv2.namedWindow('frame')

# Streaming loop
try:
    k=0
    i=1
    while (k!=ord('q')):
        # Get frameset of color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        # Validate that both frames are valid
        if not color_frame:
            continue

        img = np.asanyarray(color_frame.get_data())
        cv2.imshow('frame',img)
        k=cv2.waitKey(10)
finally:
    cv2.destroyAllWindows()
    pipeline.stop()

# %%

img_name="../yolact_cpu/input_images/"+str(time.time())+".png"
cv2.imwrite(img_name,img)

#img = cv2.imread("../yolact_cpu/input_images/1.png") 

cam_mtx=np.array([609.756,0,327.643,0,610.165,248.235,0,0,1])
cam_dist=np.array([0.0344852993,0.793592898,0.00571190879,-0.00303585594,-3.21784069])
h,w,_=img.shape

cvbridge=CvBridge()
msg_image=cvbridge.cv2_to_imgmsg(cv2.cvtColor(img,cv2.COLOR_BGR2RGB),encoding='rgb8')
#msg_image=cvbridge.cv2_to_imgmsg(img, encoding='rgb8')

cam_param=CameraInfo()
cam_param.height=h
cam_param.width=w
cam_param.K=cam_mtx
cam_param.D=cam_dist
cam_param.distortion_model="plumb_bob"
rospy.wait_for_service('Pose_cao_initializer')
try:
    rospy.sleep(1)
    yolact_service = rospy.ServiceProxy('/Pose_cao_initializer', YolactInitializeCaoPose)
    rospy.sleep(0.5)
    resp = yolact_service(msg_image, cam_param)
    print(resp)
except rospy.ServiceException as e:
    print("Service call failed: %s"%e)
