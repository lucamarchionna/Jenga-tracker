#!/usr/bin/env python3

import math
import rospy
from copy import copy
from tracker_visp.srv import *
from tracker_visp.msg import *
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

import pyquaternion
import numpy as np

search_top=True
search_bottom=False
cT_first = ReferenceBlock()

def toHomogeneousMatrix(pose):

    matrix=np.eye(4, dtype=float)

    if isinstance(pose, np.ndarray):  
        if pose.shape[0] == 3:
            q0 = 1.0
            q1 = 0.0
            q2 = 0.0
            q3 = 0.0

            matrix[0][3] = pose[0]
            matrix[1][3] = pose[1]
            matrix[2][3] = pose[2]

        # else:
        #     q = get_quaternions(pose)


    else:
        q0 = pose.orientation.w 
        q1 = pose.orientation.x
        q2 = pose.orientation.y
        q3 = pose.orientation.z

        q = np.ones(4)
        q = [q0, q1, q2, q3]
        norm = np.linalg.norm(q)

        q0 = q0 / norm
        q1 = q1 / norm
        q2 = q2 / norm
        q3 = q3 / norm

        matrix[0][3] = pose.position.x
        matrix[1][3] = pose.position.y
        matrix[2][3] = pose.position.z

    rot_matrix=np.array([ 
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)], 
        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
    ])

    
    matrix[0:3, 0:3]=rot_matrix

    return matrix

def quat2mat(quat):
    q0=quat[0]
    q1=quat[1]
    q2=quat[2]
    q3=quat[3]
    rot_matrix=np.array([ 
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)], 
        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
    ])

    return rot_matrix 

def toPoseStampedMsg(homogT):
    target = PoseStamped()

    target.pose.position=Point(homogT[0,3],homogT[1,3],homogT[2,3])

    quat = np.zeros(4)
    quat[0] = 0.5*math.sqrt(homogT[0][0]+homogT[1][1]+homogT[2][2]+1)
    quat[1] = (homogT[2][1]-homogT[1][2])/(4*quat[0])
    quat[2] = (homogT[0][2]-homogT[2][0])/(4*quat[0])
    quat[3] = (homogT[1][0]-homogT[0][1])/(4*quat[0])

    norm = np.linalg.norm(quat)
    quat = quat/ norm
    
    target.pose.orientation.w= quat[0]
    target.pose.orientation.x= quat[1]
    target.pose.orientation.y= quat[2]
    target.pose.orientation.z= quat[3]

    return target       

def blockEst_from_location(block_location):
  width=0.025
  height=0.015
  length=0.075

  layer_block=block_location.layer
  diff_layer=cT_first.location.layer-layer_block
  orient_first=cT_first.location.orientation
  orient_block=block_location.orientation
  pos_first=cT_first.location.position
  pos_block=block_location.position

  transl=np.array([0.0,0.0,0.0])
  quat=pyquaternion.Quaternion(axis = [0, 1, 0], angle = 0.00)

  transl[1]=diff_layer*height

  if orient_first==orient_block:
    transl[2]=0.0
    if pos_first=="sx" and pos_block=="sx":
      transl[0]=0*width
    elif pos_first=="sx" and pos_block=="cx":
      transl[0]=1*width
    elif pos_first=="sx" and pos_block=="dx":
      transl[0]=2*width
    elif pos_first=="cx" and pos_block=="sx":
      transl[0]=-1*width  
    elif pos_first=="cx" and pos_block=="cx":
      transl[0]=0*width      
    elif pos_first=="cx" and pos_block=="dx":
      transl[0]=1*width   
    elif pos_first=="dx" and pos_block=="sx":
      transl[0]=-2*width
    elif pos_first=="dx" and pos_block=="cx":
      transl[0]=-1*width
    elif pos_first=="dx" and pos_block=="dx":
      transl[0]=0*width              

  elif orient_first=="sx" and orient_block=="dx":
    quat = pyquaternion.Quaternion(axis = [0, 1, 0], angle = -math.pi/2)
    if pos_first=="sx":
      transl[0]=2.5*width        
    elif pos_first=="cx":
      transl[0]=1.5*width       
    elif pos_first=="dx":
      transl[0]=0.5*width 

    if pos_block=="sx":
      transl[2]=0.5*width
    elif pos_block=="cx":
      transl[2]=1.5*width
    elif pos_block=="dx":
      transl[2]=2.5*width        

  elif orient_block=="dx" and orient_first=="sx":
    quat = pyquaternion.Quaternion(axis = [0, 1, 0], angle = math.pi/2)    
    if pos_first=="sx":
      transl[0]-0.5*width        
    elif pos_first=="cx":
      transl[0]=-1.5*width       
    elif pos_first=="dx":
      transl[0]=-2.5*width 
          
    if pos_block=="sx":
      transl[2]=2.5*width
    elif pos_block=="cx":
      transl[2]=1.5*width
    elif pos_block=="dx":
      transl[2]=0.5*width    

  print(transl)
  print("-"*30)

  p1Tp2_mat = toHomogeneousMatrix(transl)
  p1Tp2_mat[0:3, 0:3] = quat2mat(quat)

  print(p1Tp2_mat)
  print("-"*30)

  cT_homog = toHomogeneousMatrix(cT_first.pose.pose)
  cTtarget = cT_homog.dot(p1Tp2_mat)

  print(cTtarget)
  print("-"*30)

  cTtarget_pose = toPoseStampedMsg(cTtarget)
  # cTtarget_pose.header.stamp=rospy.Time.now()
  # cTtarget_pose.header.frame_id="camera_color_optical_frame"

  cTtarget_block=ReferenceBlock()
  cTtarget_block.pose=cTtarget_pose
  cTtarget_block.location.position=pos_block
  cTtarget_block.location.orientation=orient_block
  cTtarget_block.location.layer=layer_block

  return cTtarget_block
      
def First_service_handle(req):
  global search_top
  global search_bottom
  global cT_first
  rospy.loginfo("Service first init called")
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
    cT_first=copy(cTlayer1)
  else:
    ready.data=True
  return ready

def Block_service_handle(req):
  print("Blockestimate service called")
  print(req.block_choice)
  blockEstimate=blockEst_from_location(req.block_choice)
  print(blockEstimate)
  return BlockEstimationResponse(blockEstimate)

if __name__ == "__main__":
  rospy.init_node('Ros_server_motion')
  s=rospy.Service('/FirstLayerPose',FirstLayerPose,First_service_handle)
  rospy.loginfo("Start service First layer, waiting...")
  s=rospy.Service('/BlockEstimation',BlockEstimation,Block_service_handle)
  rospy.loginfo("Start service BlockEst, waiting...")  
  try:
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo('Shutting down...')