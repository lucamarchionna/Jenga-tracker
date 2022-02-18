#!/usr/bin/env python3

import rospy
from tracker_visp.srv import *
from tracker_visp.msg import *
from std_msgs.msg import String
from std_msgs.msg import Bool

# %%
import numpy as np
from warnings import catch_warnings
with catch_warnings():
    import quaternion

#!python
#cython: language_level=3

import torch
import cv2
import sys

import os, rospkg
rospack = rospkg.RosPack()
# %%
from Block_class import Block
from Group_finder_writer import Blocks_group
from LayerGroup import Layer_group

# %%
rosPath = rospack.get_path("tracker_visp")
yolact_path=os.path.join(rosPath, "yolact_cpu")
if not sys.path.__contains__(yolact_path):
    sys.path.append(yolact_path)
from yolact import Yolact
from data import set_cfg
from utils.augmentations import FastBaseTransform
from layers.output_utils import postprocess

# %%
def to_FirstLayerPoseRequest(found_top,found_bottom,search_bottom=False,rvec=None,tvec=None,position=None,layer=None):
  #From estimated pose to service message response
  found_top_msg=Bool()
  found_top_msg.data=False
  found_bottom_msg=Bool()
  found_bottom_msg.data=False  
  cTlayer1_msg=ReferenceBlock()
  # if (not search_bottom and found_top) or found_bottom:
  if found_top:
    tvec=np.squeeze(tvec.copy())
    rvec=np.squeeze(rvec.copy())
    found_top_msg.data=found_top
    found_bottom_msg.data=found_bottom
    cTlayer1_msg.location.position=position
    if rvec[1]<0:	#radians, "right face seen from camera"
      cTlayer1_msg.location.orientation="dx"
    else:
      cTlayer1_msg.location.orientation="sx"
    cTlayer1_msg.location.layer=layer
    #Name of reference frame of pose
    cTlayer1_msg.pose.header.frame_id="camera_color_optical_frame"
    cTlayer1_msg.pose.pose.position.x=tvec[0]
    cTlayer1_msg.pose.pose.position.y=tvec[1]
    cTlayer1_msg.pose.pose.position.z=tvec[2]
    rvec_quat=quaternion.from_rotation_vector(rvec)
    cTlayer1_msg.pose.pose.orientation.x=rvec_quat.x
    cTlayer1_msg.pose.pose.orientation.y=rvec_quat.y
    cTlayer1_msg.pose.pose.orientation.z=rvec_quat.z
    cTlayer1_msg.pose.pose.orientation.w=rvec_quat.w
  
  request=FirstLayerPoseRequest(found_top_msg,found_bottom_msg,cTlayer1_msg)

  return request

# %%
def to_PoseEstimationResponse(cao_path=None,rvec=None,tvec=None,position=None,layer=None):
  #From estimated pose to service message response
  initPose_msg=ReferenceBlock()
  cao_msg=String()
  if cao_path!="":
    tvec=np.squeeze(tvec.copy())
    rvec=np.squeeze(rvec.copy())
    cao_msg.data=cao_path
    initPose_msg.location.position=position
    if rvec[1]<0:	#radians, "right face seen from camera"
      initPose_msg.location.orientation="dx"
    else:
      initPose_msg.location.orientation="sx"
    initPose_msg.location.layer=layer
    #Name of reference frame of pose
    initPose_msg.pose.header.frame_id="camera_color_optical_frame"
    initPose_msg.pose.pose.position.x=tvec[0]
    initPose_msg.pose.pose.position.y=tvec[1]
    initPose_msg.pose.pose.position.z=tvec[2]
    rvec_quat=quaternion.from_rotation_vector(rvec)
    initPose_msg.pose.pose.orientation.x=rvec_quat.x
    initPose_msg.pose.pose.orientation.y=rvec_quat.y
    initPose_msg.pose.pose.orientation.z=rvec_quat.z
    initPose_msg.pose.pose.orientation.w=rvec_quat.w
  
  response=PoseEstimationResponse(cao_msg,initPose_msg)

  return response
  
def to_RestartFirstLayerResponse(found_top):
  found_top_msg=Bool()
  found_top_msg.data=found_top
  response=RestartFirstLayerResponse(found_top_msg)

  return response

# %%
class Yolact_full_service():
  def __init__(self):
    self.msg_image=None
    self.cam_width=640
    self.cam_height=480    
    self.width=480
    self.height=480
    self.pose_window_name="-POSE-:c:choose,esc:exit"   
    self.pose_imshow=np.zeros((self.width,self.height*2,3),dtype=np.uint8)
    self.init_window_name="-INIT-:c:choose,esc:exit"
    self.first_imshow=np.zeros((self.height,self.width*3,3),dtype=np.uint8)
    #Init stuff to enable POSE loop
    self.newImage=False
    self.keyFromWindow=0
    self.in_pose_service=False
    self.in_init_service=False

    self.rosPath=rosPath
    self.yolact_path=yolact_path

    self.cfg_name='yolact_resnet101_jenga_dataset_new_config'
    self.weights_path=os.path.join(self.yolact_path,'weights/yolact_resnet101_jenga_dataset_new_1199_180000.pth')

    ##### Setup #####
    if torch.cuda.is_available():
        torch.backends.cudnn.fastest = True
        torch.set_default_tensor_type('torch.cuda.FloatTensor')
        set_cfg(self.cfg_name)
        self.net = Yolact().cuda()
    else: #cpu only
        torch.set_default_tensor_type('torch.FloatTensor')
        set_cfg(self.cfg_name)
        self.net=Yolact().cpu()

    self.transform = FastBaseTransform()

    self.net.load_weights(self.weights_path)
    self.net.eval()

    self.sFirst=rospy.Service('/RestartFirstLayer',RestartFirstLayer,self.init_service_handle)
    self.sPose=rospy.Service('/PoseEstimation',PoseEstimation,self.pose_service_handle)
    self.first_layer_orient = rospy.Publisher('/init_orientation', String, queue_size=1)   


  # %%
  def compute_outputs(self, img, score_threshold):
    h, w, _ = img.shape
    tensor_img=torch.from_numpy(img)
    if torch.cuda.is_available():
      cuda_img=tensor_img.cuda().float()
      batch = self.transform(cuda_img.unsqueeze(0))
    else: #cpu only
      cpu_img=tensor_img.cpu().float()
      batch = self.transform(cpu_img.unsqueeze(0))

    with torch.no_grad():
      preds = self.net(batch)

    classes, scores, boxes, masks = postprocess(preds , w, h, score_threshold=score_threshold)

    classes=classes.cpu()
    scores=scores.cpu()
    boxes=boxes.cpu()
    masks=masks.cpu()

    return classes, scores, boxes, masks

# %%
  # Big client function
  def first_layer_detection(self,color_image,cam_mtx,cam_dist):

    # %%
    ### CROP IMAGE TO BE SQUARE
    hhh,www,_=color_image.shape
    width_offset=int(www-hhh)/2
    img=color_image[int(hhh/2)-240:int(hhh/2)+240,int(www/2)-240:int(www/2)+240]
    self.first_imshow=np.hstack((img,np.zeros((self.height,self.width,3),dtype=np.uint8),np.zeros((self.height,self.width,3),dtype=np.uint8)))    

    # %%
    # Detection
    classes, scores, boxes, masks = self.compute_outputs(img,0.3)
    print("Detected --> ",len(masks))

    # %%
    if len(masks)==0:
      rospy.loginfo("No blocks found")
      return to_FirstLayerPoseRequest(False,False,False)

    # %%
    totArea=0
    blocks_list=[]
    for idx in range(len(masks)):
      #Take masks
      maskcv2=masks[idx].numpy().round().astype(np.uint8)*255

      block=Block(classes[idx],scores[idx],boxes[idx],masks[idx],len(blocks_list))

      block.add_masked(img,maskcv2)

      #Find contours
      ret=block.find_contour(retr_list=cv2.RETR_LIST,chain_approx=cv2.CHAIN_APPROX_NONE)

      if ret:
        blocks_list.append(block)
        #Sum areas
        totArea+=block.area
      else:
        print("Skipped empty contour")

    if len(blocks_list)==0:
      rospy.loginfo("No blocks contour computed")
      return to_FirstLayerPoseRequest(False,False,False)
    else:
      avgArea=totArea/len(blocks_list)

    # %%
    ## Approximate and find corners
    for block in blocks_list:
      block.differentiate_approximate(0.04,avgArea,0.008,1.1)
      block.find_corners(5,subPix_eps=0.001,subPix_iters=100,harris_param=[2,3,0.04],harris_threshold=0.09)
      block.compute_slopes(min_slope_h=2)

    # %%
    ## Draw approximated masks
    img_all_masks=np.zeros(img.shape,dtype=np.uint8)
    for block in blocks_list:
      img_all_masks+=block.draw_masked_approx(img)

    # %%
    self.first_imshow=np.hstack((img,img_all_masks,np.zeros((self.height,self.width,3),dtype=np.uint8)))    

    ## Exit if less than 6 blocks found
    if len(blocks_list)<6:
      rospy.loginfo("Not enough blocks found, less than 6")  
      return to_FirstLayerPoseRequest(False,False)   
    else:
      ## Sort blocks_list for centroid height
      blocks_list_ordered=sorted(blocks_list,key=Block.get_centroid_height)

      ## Pick highest 3 and lowest 3 from sorted
      top3_blocks=blocks_list_ordered[:3]
      print(len(top3_blocks))
      bottom3_blocks=blocks_list_ordered[-3:]
      print(len(bottom3_blocks))

      ## Group the top 3 blocks, draw their masks
      top3_masks=np.zeros(img.shape,dtype=np.uint8)
      top_groups=[]
      for block in top3_blocks:
        top3_masks+=block.draw_masked_approx(img)
      for block in top3_blocks:
        if block.block_type=='front_face':
          top_group=Layer_group(top3_blocks,block.idx,img_all_masks)
          top_group.init_down(blocks_list_ordered)
          # print("TOP center: ",top_group.is_central())
          top_groups.append(top_group)

      self.first_imshow=np.hstack((img,img_all_masks,top3_masks))

    find_central=False
    for top_group in top_groups:
      if top_group.is_central():
        first_layer=top_group
        find_central=True
        print(top_group.is_central())

    if not find_central:
      return to_FirstLayerPoseRequest(False,False)

    rospy.loginfo("\nChoose first layer pressing 'c', exit pressing 'esc'\n")
    top_masks=first_layer.draw_masked_group(img)
    self.first_imshow=np.hstack((img,img_all_masks,top_masks))

    # %%
    # # Single block size and reference pose setup
    b_width=0.025
    b_height=0.015
    b_length=0.075
    #to move object reference frame to desired new pose:
    zend_quat_o=np.quaternion(np.cos(np.pi/4),np.sin(np.pi/4), 0, 0)
    zend_Rot_o=quaternion.as_rotation_matrix(zend_quat_o)
    zend_t_o=np.array([-b_width/2,b_height/2,0])
    zend_T_o =np.eye(4)
    zend_T_o[:3,:3]=zend_Rot_o
    zend_T_o[:3,3]=zend_t_o

    # %%
    first_layer.setup_object_frame(b_width,b_height,b_length,zend_T_o)
    rvec_first,tvec_first=first_layer.poseEstimate(width_offset,cam_mtx,cam_dist)

    #Draw frame axes on image
    img_big=np.zeros((self.cam_height,self.cam_width,3),dtype=np.uint8)
    img_big[:,80:560]=img_all_masks.copy()
    img_big=cv2.drawFrameAxes(img_big,cam_mtx,cam_dist,rvec_first,tvec_first,0.03,thickness=3)
    self.first_imshow=np.hstack((img,img_big[:,80:560],top_masks))

    first_layer_number=18

    # From pose estimation to service request message
    request = to_FirstLayerPoseRequest(True,True,False,rvec_first,tvec_first,"cx",first_layer_number)

    return request

# %%
  def init_service_handle(self,req):

    print("Init Service called")
    self.in_init_service=True

    # %%
    ## Convert messages into opencv formats
    msg_image=req.image
    cam_info=req.camInfo
    color_image = np.frombuffer(msg_image.data, dtype=np.uint8).reshape(msg_image.height, msg_image.width, -1).copy()
    color_image=cv2.cvtColor(color_image,cv2.COLOR_RGB2BGR)
    cam_mtx = np.array(cam_info.K).reshape([3, 3])
    cam_dist = np.array(cam_info.D)    

    # %% 
    request=to_FirstLayerPoseRequest(False,False)

    #wait until ready
    while(not rospy.is_shutdown()):
      #wait for service
      rospy.loginfo("Waiting for FirstLayerPose service")      
      rospy.wait_for_service('/FirstLayerPose')
      try:
        #Send to service, 
        rospy.loginfo("Calling FirstLayerPose service")          
        motion_service = rospy.ServiceProxy('/FirstLayerPose', FirstLayerPose)
        resp = motion_service(request)
        rospy.loginfo("Service FirstLayerPose called")  
        # If motion not ready, ask the server again
        if resp.ready:
          break
      except rospy.ServiceException as e:
        rospy.loginfo("Service FirstLayerPose call failed: %s"%e)
        continue
    
    ##
    request = yolact_object.first_layer_detection(color_image,cam_mtx,cam_dist)

    self.newImage=True # signal to the main that a new image is received
    while(self.keyFromWindow==0):
      # Wait until key is chosen in the main loop
      continue
    else:
      # get ready for next image 
      k=self.keyFromWindow
      self.keyFromWindow=0
      # exit if pressed 'esc'
      if (k==27):
        rospy.loginfo("SERVICE HALTED BY USER")
        self.in_init_service=False
        request=to_FirstLayerPoseRequest(False,False)
      elif (k==ord('c')):
        rospy.loginfo("CHOICE")
    
    #wait for service
    rospy.loginfo("Waiting for FirstLayerPose service")      
    rospy.wait_for_service('/FirstLayerPose')
    try:
      #Send to service, 
      rospy.loginfo("Calling FirstLayerPose service")          
      motion_service = rospy.ServiceProxy('/FirstLayerPose', FirstLayerPose)
      resp = motion_service(request)
      rospy.loginfo("Service FirstLayerPose called")                 
    except rospy.ServiceException as e:
      rospy.loginfo("Service FirstLayerPose call failed: %s"%e)
      self.in_init_service=False
      return to_RestartFirstLayerResponse(False)

    if request.found_top.data:
      print("Found orient: ",request.cTo.location.orientation)
      self.first_layer_orient.publish(request.cTo.location.orientation)
    else:
      print("Not found, take another image")
    
    self.in_init_service=False      
    return to_RestartFirstLayerResponse(request.found_top.data)
  # %%
  def pose_service_handle(self,req):

    print("PoseEstimation service called")
    self.in_pose_service=True
    # %%
    ## Convert messages into opencv formats
    msg_image=req.image
    cam_info=req.camInfo
    tvec_est=np.array([0,0,0],dtype=np.float)
    tvec_est[0]=req.cTo_est.pose.pose.position.x
    tvec_est[1]=req.cTo_est.pose.pose.position.y
    tvec_est[2]=req.cTo_est.pose.pose.position.z
    rvec_quat=quaternion.quaternion()
    rvec_quat.x=req.cTo_est.pose.pose.orientation.x
    rvec_quat.y=req.cTo_est.pose.pose.orientation.y
    rvec_quat.z=req.cTo_est.pose.pose.orientation.z
    rvec_quat.w=req.cTo_est.pose.pose.orientation.w
    rvec_est=quaternion.as_rotation_vector(rvec_quat)
    position=req.cTo_est.location.position
    orientation=req.cTo_est.location.orientation
    layer=req.cTo_est.location.layer

    color_image = np.frombuffer(msg_image.data, dtype=np.uint8).reshape(msg_image.height, msg_image.width, -1).copy()
    color_image=cv2.cvtColor(color_image,cv2.COLOR_RGB2BGR)
    cam_mtx = np.array(cam_info.K).reshape([3, 3])
    cam_dist = np.array(cam_info.D)

    # %%
    ### CROP IMAGE TO BE SQUARE
    hhh,www,_=color_image.shape
    width_offset=int(www-hhh)/2
    img=color_image[int(hhh/2)-240:int(hhh/2)+240,int(www/2)-240:int(www/2)+240]
    self.pose_imshow=np.hstack((img,np.zeros(img.shape,dtype=np.uint8)))

    # %%
    # Detection
    classes, scores, boxes, masks = self.compute_outputs(img,0.3)
    print("Detected --> ",len(masks))

    # %%
    if len(masks)==0:
      self.in_pose_service=False
      rospy.loginfo("No blocks found")
      return to_PoseEstimationResponse("")

    # %%
    totArea=0
    blocks_list=[]
    for idx in range(len(masks)):
      #Take masks
      maskcv2=masks[idx].numpy().round().astype(np.uint8)*255

      block=Block(classes[idx],scores[idx],boxes[idx],masks[idx],len(blocks_list))

      block.add_masked(img,maskcv2)

      #Find contours
      ret=block.find_contour(retr_list=cv2.RETR_LIST,chain_approx=cv2.CHAIN_APPROX_NONE)

      if ret:
        blocks_list.append(block)
        #Sum areas
        totArea+=block.area
      else:
        print("Skipped empty contour")

    if len(blocks_list)==0:
      rospy.loginfo("No blocks contour computed")
      self.in_pose_service=False
      return to_PoseEstimationResponse("")
    else:
      avgArea=totArea/len(blocks_list)

    # %%
    ## Approximate and find corners
    for block in blocks_list:
      block.differentiate_approximate(0.04,avgArea,0.008,1.1)
      block.find_corners(5,subPix_eps=0.001,subPix_iters=100,harris_param=[2,3,0.04],harris_threshold=0.09)
      block.compute_slopes(min_slope_h=2)

    # %%
    img_all_masks=np.zeros(img.shape,dtype=np.uint8)
    for block in blocks_list:
      img_all_masks+=block.draw_masked_approx(img)

    # %%
    if len(blocks_list)<0:
      rospy.loginfo("SERVICE HALTED BY USER")
      self.in_pose_service=False
      return to_PoseEstimationResponse("")
    else:
      ### LIST ALL GROUPS ###
      blocks_groups_list=[]
      for block in blocks_list:
        if block.block_type=='front_face':
          idx=block.idx
          blocks_group=Blocks_group(blocks_list,idx,img_all_masks)
          blocks_groups_list.append(blocks_group)

    # %%
    ## CHOICE FROM POLICY
    #####################

    #Draw estimated block pose on image
    img_big=np.zeros((480,640,3),dtype=np.uint8)
    img_big[:,80:560]=img_all_masks.copy()
    img_big=cv2.drawFrameAxes(img_big,cam_mtx,cam_dist,rvec_est,tvec_est,0.03,thickness=2)
    img_all_masks=img_big[:,80:560].copy()
    self.pose_imshow=np.hstack((img,img_all_masks))

    print("Search from policy")
    # print("position:",tvec_est)
    # print("orientation:",rvec_est)
    #Search block from estimate, projection
    policy_found=False
    for blocks_group in blocks_groups_list:
      if blocks_group.is_project3D_target(rvec_est,tvec_est,cam_mtx,cam_dist,width_offset):
        choice_group=blocks_group
        policy_found=True
    
    #No blocks found in projection
    if not policy_found:
      self.newImage=False # signal to the main that a new image is received
      img_zero=np.zeros(img_all_masks.shape,dtype=np.uint8)
      self.pose_imshow=np.hstack((img_all_masks,img_zero))
      self.keyFromWindow=0
      rospy.loginfo("No block found for policy from projection")
      self.in_pose_service=False
      return to_PoseEstimationResponse("")

    # Block found, draw and use its group
    rospy.loginfo("\nConfirm group pressing 'c', exit pressing 'esc'\n")
    masked_group=choice_group.draw_masked_group(img)
    # img_test_points=choice_group.get_drawn_search_img()
    self.newImage=True # signal to the main that a new image is received
    self.pose_imshow=np.hstack((img_all_masks,masked_group))
    # self.img_imshow=np.hstack((img_test_points,masked_group))
    while(self.keyFromWindow==0):
      # Wait until key is chosen in the main loop
      continue
    else:
      # get ready for next image 
      k=self.keyFromWindow
      self.keyFromWindow=0
      ##
      # exit if pressed 'esc'
      ## WARNING: RETURN HERE ##
      ##
      if (k==27):
        rospy.loginfo("SERVICE HALTED BY USER")
        self.pose_imshow=np.zeros((self.width,self.height*2,3),dtype=np.uint8)
        self.in_pose_service=False
        return to_PoseEstimationResponse("")
      if (k==ord('c')):
        chosen_blocks_group=choice_group    
    # %%
    # # Single block size and reference pose setup
    b_width=0.025
    b_height=0.015
    b_length=0.075

    #to move object reference frame to desired new pose:
    zend_quat_o=np.quaternion(np.cos(np.pi/4),np.sin(np.pi/4), 0, 0)
    zend_Rot_o=quaternion.as_rotation_matrix(zend_quat_o)
    zend_t_o=np.array([-b_width/2,b_height/2,0])
    zend_T_o =np.eye(4)
    zend_T_o[:3,:3]=zend_Rot_o
    zend_T_o[:3,3]=zend_t_o

    # %%
    # # Files writer for previous searched block
    center_cao_path=os.path.join(self.rosPath,"model/blocks_cao/jengaCenterZend.cao")
    right_cao_path=os.path.join(self.rosPath,"model/blocks_cao/jengaRightZend.cao")
    left_cao_path=os.path.join(self.rosPath,"model/blocks_cao/jengaLeftZend.cao")

    # %%
    chosen_blocks_group.setup_object_frame(b_width,b_height,b_length,zend_T_o)

    # %%
    cao_name=os.path.join(self.rosPath,"model/file_cao.cao")
    chosen_blocks_group.cao_file_write(cao_name,center_cao_path,right_cao_path,left_cao_path)

    # %%
    initPose_file_name=os.path.join(self.rosPath,"model/file_init.pos")
    tvec,rvec=chosen_blocks_group.initPose_file_write(width_offset,initPose_file_name,cam_mtx,cam_dist)
    #Draw frame axes on image
    img_big=np.zeros(color_image.shape,dtype=np.uint8)
    img_big[:,80:560]=masked_group.copy()
    img_big=cv2.drawFrameAxes(img_big,cam_mtx,cam_dist,rvec,tvec,0.04,thickness=2)
    self.pose_imshow=np.hstack((img_all_masks,img_big[:,80:560]))
    # %%
    rospy.loginfo("---\nSUCCESFULLY ENDED\n---")
    self.in_pose_service=False
    return to_PoseEstimationResponse(cao_name,rvec,tvec,position,layer)

# %%
if __name__ == "__main__":
  rospy.init_node('Yolact_server')
  yolact_object=Yolact_full_service()
  cv2.namedWindow(yolact_object.init_window_name)
  cv2.namedWindow(yolact_object.pose_window_name)
  try:
    while(not rospy.is_shutdown()):
      # now a key has been pressed
      # pass the key only if a new image has been received
      if yolact_object.in_init_service:
        kinit=0 #to enter in the loop
        # stay in the loop until q or c is pressed
        while (kinit!=ord('c') and kinit!=27 and not rospy.is_shutdown()):
          cv2.imshow(yolact_object.init_window_name,yolact_object.first_imshow) #imshow in the main, on the concurrent image
          kinit=cv2.waitKey(10) #red key in the image window        
        if (yolact_object.newImage and not rospy.is_shutdown()):
            yolact_object.keyFromWindow=kinit
            yolact_object.newImage=False  #return waiting for a new image
      elif yolact_object.in_pose_service:
        kpose=0 #to enter in the loop
        # stay in the loop until c or esc is pressed
        while (kpose!=ord('c') and kpose!=27 and not rospy.is_shutdown()):
          cv2.imshow(yolact_object.pose_window_name,yolact_object.pose_imshow) #imshow in the main, on the concurrent image
          kpose=cv2.waitKey(10) #red key in the image window        
        if (yolact_object.newImage and not rospy.is_shutdown()):
          yolact_object.keyFromWindow=kpose
          yolact_object.newImage=False  #return waiting for a new image
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo('Shutting down...')
    cv2.destroyAllWindows()
  finally:
    cv2.destroyAllWindows()