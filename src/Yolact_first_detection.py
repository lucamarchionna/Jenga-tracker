#!/usr/bin/env python3

import rospy
from tracker_visp.srv import FirstLayerPose
from tracker_visp.srv import FirstLayerPoseRequest
from tracker_visp.msg import ReferenceBlock
from tracker_visp.msg import location
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# %%
import pyrealsense2 as rs
import numpy as np
import time
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
def to_FirstLayerPoseRequest(found_top,found_bottom,rvec=None,tvec=None,position=None,layer=None):
  #From estimated pose to service message response
  found_top_msg=Bool()
  found_top_msg.data=False
  found_bottom_msg=Bool()
  found_bottom_msg.data=False  
  cTlayer1_msg=ReferenceBlock()
  if found_top or found_bottom:
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
class First_layer_client():
  def __init__(self):

    # Create a pipeline
    self.pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    # Intrinsics
    self.cam_width=640
    self.cam_height=480
    self.cam_mtx=np.array([[598.365,0,327.306],[0,600.842,245.576],[0,0,1]])
    self.cam_dist=np.array([[]])
    # cam_dist=np.array([[0.0344852993,0.793592898,0.00571190879,-0.00303585594,-3.21784069]])

    self.config = rs.config()
    self.config.enable_stream(rs.stream.color, self.cam_width, self.cam_height, rs.format.bgr8, 30) #color enabled

    # Start streaming
    self.profile=self.pipeline.start(self.config)
    rospy.sleep(1)

    #Squared images
    self.width=480
    self.height=480
    self.img_imshow=np.zeros((self.height,self.width*3,3),dtype=np.uint8)
    # self.top_show=np.zeros((self.height*2,self.width*2,3),dtype=np.uint8)
    # self.bottom_show=np.zeros((self.height*2,self.width*2,3),dtype=np.uint8)

    self.rosPath=rosPath
    self.yolact_path=yolact_path

    ## Net config
    ##### Setup #####
    self.cfg_name='yolact_resnet101_jenga_dataset_new_config'
    self.weights_path=os.path.join(self.yolact_path,'weights/yolact_resnet101_jenga_dataset_new_1199_180000.pth')
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

  # %%
  # Net usage
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
  def first_layer_detection(self,color_image,search_top,search_bottom):
    # %%
    img_name=os.path.join(self.yolact_path,"input_images",str(time.time())+".png")
    cv2.imwrite(img_name,color_image)

    # %%
    ### CROP IMAGE TO BE SQUARE
    hhh,www,_=color_image.shape
    width_offset=int(www-hhh)/2
    img=color_image[int(hhh/2)-240:int(hhh/2)+240,int(www/2)-240:int(www/2)+240]

    # %%
    # Detection
    classes, scores, boxes, masks = self.compute_outputs(img,0.3)
    print("Detected --> ",len(masks))

    # %%
    if len(masks)==0:
      rospy.loginfo("No blocks found")
      return to_FirstLayerPoseRequest(False,False)

    # %%
    totArea=0
    blocks_list=[]
    for idx in range(len(masks)):
      #Take masks
      maskcv2=masks[idx].numpy().round().astype(np.uint8)*255

      block=Block(classes[idx],scores[idx],boxes[idx],masks[idx],len(blocks_list))
      blocks_list.append(block)

      block.add_masked(img,maskcv2)

      #Find contours
      block.find_contour(retr_list=cv2.RETR_LIST,chain_approx=cv2.CHAIN_APPROX_NONE)

      #Sum areas
      totArea+=block.area

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
    self.img_imshow=np.zeros((self.height,self.width*3,3),dtype=np.uint8)

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
      # for block in top3_blocks:
      #   top3_masks+=block.draw_masked_approx(img)
      for block in top3_blocks:
        if block.block_type=='front_face':
          top_group=Layer_group(top3_blocks,block.idx,img_all_masks)
          top_group.init_up(blocks_list_ordered)
          # print("TOP center: ",top_group.is_central())
          top_groups.append(top_group)
      
      ## DONT Group the bottom 3 blocks, draw their masks
      bottom3_masks=np.zeros(img.shape,dtype=np.uint8)
      bottom_groups=[]
      # for block in bottom3_blocks:
      #   bottom3_masks+=block.draw_masked_approx(img)
      for block in bottom3_blocks:
        if block.block_type=='front_face':
          bottom_group=Layer_group(bottom3_blocks,block.idx,img_all_masks)
          bottom_group.init_down(blocks_list_ordered)
          # print("Bottom center: ",bottom_group.is_central())
          bottom_groups.append(bottom_group)
      self.img_imshow=np.hstack((img_all_masks,top3_masks,bottom3_masks))
    # %%
    ## Find top central and bottom central groups
    top_central_numbers=0
    for top_group in top_groups:
      if top_group.is_central():
        first_layer=top_group
        top_central_numbers+=1
    bottom_central_numbers=0
    for bottom_group in bottom_groups:
      if bottom_group.is_central():
        last_layer=bottom_group
        bottom_central_numbers+=1

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
    #Pose estimate of topmost, if top is still to be searched
    # CONTINUE only if top layer is full
    if top_central_numbers==1 and search_top:
      first_layer.print_idx()
      top3_masks=first_layer.draw_masked_group(img)
      first_layer.setup_object_frame(b_width,b_height,b_length,zend_T_o)

      rvec_first,tvec_first=first_layer.poseEstimate(width_offset,self.cam_mtx,self.cam_dist)

      #Draw frame axes on image
      img_big=np.zeros((self.cam_height,self.cam_width,3),dtype=np.uint8)
      img_big[:,80:560]=img_all_masks.copy()
      img_big=cv2.drawFrameAxes(img_big,self.cam_mtx,self.cam_dist,rvec_first,tvec_first,0.03,thickness=3)
      self.img_imshow=np.hstack((img_big[:,80:560],top3_masks,bottom3_masks))

      first_layer_number=18

      # From pose estimation to service request message
      request = to_FirstLayerPoseRequest(True,False,rvec_first,tvec_first,"cx",first_layer_number)

      return request
      # bl_orientation=request.cTlayer1.location.orientation

      # # From top to bottom, object frame 3D translations
      # ### NUMBER OF TOWER FLOORS##
      # ######################################
      # first_layer.top_to_bottom3D(bl_orientation,first_layer_number)

      # # Check and draw bottom translation projections 3d into 2d
      # img_check_bottom=first_layer.project3D_draw(img_big[:,80:560],rvec_first,tvec_first,self.cam_mtx,self.cam_dist,width_offset)
      # self.img_imshow=np.hstack((img_check_bottom,top3_masks,bottom3_masks))
    
      # # Projecting number of floors below to bottom, check if hits the bottom floor detected
      # # If yes, top and bottom are really detected
      # if (first_layer.project3D_toBottom(bl_orientation,bottom3_blocks,rvec_first,tvec_first,self.cam_mtx,self.cam_dist,width_offset)):
      #   return request
      # else:
      #   # Otherwise, not found
      #   rospy.loginfo("Projection to bottom failed")
      #   return to_FirstLayerPoseRequest(False,True,rvec_first,tvec_first,"cx",first_layer_number)
    
    elif bottom_central_numbers==1 and search_bottom:
      last_layer.print_idx()      
      bottom3_masks=last_layer.draw_masked_group(img)
      #Pose estimate of bottomost, if bottom is still to be searched and top is already searched
      # CONTINUE only if bottom layer is full
      last_layer.setup_object_frame(b_width,b_height,b_length,zend_T_o)

      rvec_last,tvec_last=last_layer.poseEstimate(width_offset,self.cam_mtx,self.cam_dist)

      #Draw frame axes on image
      img_big=np.zeros((self.cam_height,self.cam_width,3),dtype=np.uint8)
      img_big[:,80:560]=img_all_masks.copy()
      img_big=cv2.drawFrameAxes(img_big,self.cam_mtx,self.cam_dist,rvec_last,tvec_last,0.03,thickness=3)
      self.img_imshow=np.hstack((img_big[:,80:560],top3_masks,bottom3_masks))

      last_layer_number=18

      # From pose estimation to service request message
      request = to_FirstLayerPoseRequest(False,True,rvec_last,tvec_last,"cx",last_layer_number)

      return request

    if search_top:
      rospy.loginfo("Not full, top central layer")
    elif search_bottom:
      rospy.loginfo("Not full, bottom central layer")
    return to_FirstLayerPoseRequest(False,False)

# %%
if __name__ == "__main__":
  rospy.init_node('First_layer_service')
  # client=rospy.client('First_layer_detection',FirstLayerPose)
  yolact_object=First_layer_client()
  
  request=to_FirstLayerPoseRequest(False,False)
  search_top=True
  search_bottom=False

  cv2.namedWindow("top3,bottom3")
  cv2.namedWindow("Camera capture")
  try:
    # cycle until finding layer, after sending it
    while(not rospy.is_shutdown()):
      #wait for service
      rospy.loginfo("Waiting for service")      
      rospy.wait_for_service('/FirstLayerPose')
      try:
        #Send to service, 
        rospy.loginfo("Calling service")          
        motion_service = rospy.ServiceProxy('/FirstLayerPose', FirstLayerPose)
        resp = motion_service(request)
        rospy.loginfo("Service called")                 
      except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)      

      # If motion not ready, ask the server again
      if not resp.ready:
        continue

      # If last time sent a found both=true, exit loop
      if (not search_top and not search_bottom):
        break

      # k=-1
      # # stay until correct frame captured
      while (not rospy.is_shutdown()):
        try:
          # Get frameset of color
          frames = yolact_object.pipeline.wait_for_frames()
          color_frame = frames.get_color_frame()
          # Validate that frame is valid
          if not color_frame:
              continue
          else:
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow("Camera capture",color_image)
            cv2.waitKey(10)
            break
        except:
          print("Realsense exception")
          break
      
      # Detect first layer from current image
      if search_top:
        request = yolact_object.first_layer_detection(color_image,search_top,search_bottom)
        # keep searching if not found top layer
        if request.found_top.data:
          search_top=False
          search_bottom=True
      elif search_bottom:
        request = yolact_object.first_layer_detection(color_image,search_top,search_bottom)
        # keep searching if not found bottom layer
        if request.found_bottom.data:
          search_top=False
          search_bottom=False

      cv2.imshow("top3,bottom3",yolact_object.img_imshow) #imshow in the main, on the concurrent image
      # k=-1
      # while(k==-1 and not rospy.is_shutdown()):
      cv2.waitKey(5000) #red key in the image window

  except KeyboardInterrupt:
    rospy.loginfo('Shutting down...')
  finally:
    cv2.destroyAllWindows()
    yolact_object.pipeline.stop()