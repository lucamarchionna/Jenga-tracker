#!/usr/bin/env python3

import rospy
from tracker_visp.srv import FirstLayerPose
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
def pnp_to_firstLayerPose(rvec_o,tvec_o):
    tvec=np.squeeze(tvec_o.copy())
    rvec=np.squeeze(rvec_o.copy())
    found_msg=Bool()
    found_msg.data=True
    cTlayer1_msg=ReferenceBlock()
    cTlayer1_msg.location.position="cx"
    if rvec[1]<0:	#radians, "right face seen from camera"
      cTlayer1_msg.location.orientation="right"
    else:
      cTlayer1_msg.location.orientation="left"
    cTlayer1_msg.location.layer=1
    cTlayer1_msg.pose.header.frame_id="camera_color_optical_frame"
    cTlayer1_msg.pose.pose.position.x=tvec[0]
    cTlayer1_msg.pose.pose.position.y=tvec[1]
    cTlayer1_msg.pose.pose.position.z=tvec[2]
    rvec_quat=quaternion.from_rotation_vector(rvec)
    cTlayer1_msg.pose.pose.orientation.x=rvec_quat.x
    cTlayer1_msg.pose.pose.orientation.y=rvec_quat.y
    cTlayer1_msg.pose.pose.orientation.z=rvec_quat.z
    cTlayer1_msg.pose.pose.orientation.w=rvec_quat.w

    return found_msg,cTlayer1_msg

# %%
class First_layer_client():
  def __init__(self):

    # Create a pipeline
    self.pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    self.cam_width=640
    self.cam_height=480
    self.cam_mtx=np.array([[598.365,0,327.306],[0,600.842,245.576],[0,0,1]])
    self.cam_dist=np.array([[]])
    # cam_dist=np.array([[0.0344852993,0.793592898,0.00571190879,-0.00303585594,-3.21784069]])
    self.config = rs.config()
    self.config.enable_stream(rs.stream.color, self.cam_width, self.cam_height, rs.format.bgr8, 30) #color enabled

    # Start streaming
    self.profile=self.pipeline.start(self.config)

    self.width=480
    self.height=480
    self.img_imshow=np.zeros((self.height,self.width*3,3),dtype=np.uint8)
    # self.top_show=np.zeros((self.height*2,self.width*2,3),dtype=np.uint8)
    # self.bottom_show=np.zeros((self.height*2,self.width*2,3),dtype=np.uint8)

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
  def first_layer_detection(self,color_image):
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
      found_msg=Bool()
      found_msg.data=False
      cTlayer1_msg=ReferenceBlock() 
      return found_msg,cTlayer1_msg

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
    self.img_imshow=np.zeros((self.width,self.height*3,3),dtype=np.uint8)

    ## Exit if less than 6 blocks found
    if len(blocks_list)<6:
      rospy.loginfo("Not enough blocks found, less than 6")
      found_msg=Bool()
      found_msg.data=False
      cTlayer1_msg=ReferenceBlock() 
      return found_msg,cTlayer1_msg
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
          top_group=Layer_group(top3_blocks,block.idx,top3_masks)
          # print("TOP center: ",top_group.is_central())
          top_groups.append(top_group)
      
      ## Group the bottom 3 blocks, draw their masks
      bottom3_masks=np.zeros(img.shape,dtype=np.uint8)
      bottom_groups=[]
      for block in bottom3_blocks:
        bottom3_masks+=block.draw_masked_approx(img)
      for block in bottom3_blocks:
        if block.block_type=='front_face':
          bottom_group=Layer_group(bottom3_blocks,block.idx,bottom3_masks)
          # print("Bottom center: ",bottom_group.is_central())
          bottom_groups.append(bottom_group)
      self.img_imshow=np.hstack((img_all_masks,top3_masks,bottom3_masks))

      # # DRAW STUFF TO SHOW
      # top_img1=np.zeros((self.width,self.height,3),dtype=np.uint8)
      # top_img2=np.zeros((self.width,self.height,3),dtype=np.uint8)
      # if (len(top_groups)==2):
      #   top_img1=top_groups[0].draw_masked_group(img)
      #   top_test_points1=top_groups[0].get_drawn_search_img()
      #   top_img2=top_groups[1].draw_masked_group(img)
      #   top_test_points2=top_groups[1].get_drawn_search_img()
      #   stack1=np.hstack((top_test_points1,top_img1))
      #   stack2=np.hstack((top_test_points2,top_img2))
      #   self.top_show=np.vstack((stack1,stack2))
      # bottom_img1=np.zeros((self.width,self.height,3),dtype=np.uint8)
      # bottom_img2=np.zeros((self.width,self.height,3),dtype=np.uint8)
      # if (len(bottom_groups)==2):
      #   bottom_img1=bottom_groups[0].draw_masked_group(img)
      #   bottom_test_points1=bottom_groups[0].get_drawn_search_img()
      #   bottom_img2=bottom_groups[1].draw_masked_group(img)
      #   bottom_test_points2=bottom_groups[1].get_drawn_search_img()
      #   stack1=np.hstack((bottom_test_points1,bottom_img1))
      #   stack2=np.hstack((bottom_test_points2,bottom_img2))
      #   self.bottom_show=np.vstack((stack1,stack2))

    # %%
    ## Find top central and bottom central groups
    top_central_numbers=0
    for top_group in top_groups:
      if top_group.is_central():
        chosen_first_layer=top_group
        top_central_numbers+=1
    bottom_central_numbers=0
    for bottom_group in bottom_groups:
      if bottom_group.is_central():
        chosen_last_layer=bottom_group
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
    #Pose estimate of topmost
    if top_central_numbers==1 and bottom_central_numbers==1:
      chosen_first_layer.setup_object_frame(b_width,b_height,b_length,zend_T_o)
      # chosen_last_layer.setup_object_frame(b_width,b_height,b_length,zend_T_o)
      rvec_first,tvec_first=chosen_first_layer.poseEstimate(width_offset,self.cam_mtx,self.cam_dist)

      #Draw frame axes on image
      img_big=np.zeros((self.cam_height,self.cam_width,3),dtype=np.uint8)
      img_big[:,80:560]=img_all_masks.copy()
      img_big=cv2.drawFrameAxes(img_big,self.cam_mtx,self.cam_dist,rvec_first,tvec_first,0.03,thickness=3)
      self.img_imshow=np.hstack((img_big,top3_masks,bottom3_masks))

      # From pose estimation to service request message
      found_msg,cTlayer1_msg = pnp_to_firstLayerPose(rvec_first,tvec_first)

      # From top to bottom, object frame 3D translations
      bott,bottSx,bottCx,bottDx = chosen_first_layer.top_to_bottom3D(cTlayer1_msg.location.orientation,8)

      # Check and draw bottom translation projections 3d into 2d
      img_check_bottom=chosen_first_layer.project3D_draw(img_big[:,80:560],bott,bottSx,bottCx,bottDx,rvec_first,tvec_first,self.cam_mtx,self.cam_dist,width_offset)
      self.img_imshow=np.hstack((img_check_bottom,top3_masks,bottom3_masks))

      if (chosen_first_layer.project3D_toBottom(cTlayer1_msg.location.orientation,chosen_last_layer,bott,bottDx,bottSx,rvec_first,tvec_first,self.cam_mtx,self.cam_dist,width_offset)):
        return found_msg,cTlayer1_msg
      else:
        found_msg=Bool()
        found_msg.data=False
        cTlayer1_msg=ReferenceBlock()
        return found_msg,cTlayer1_msg
    # %%
    # ELSE; RETURN empty
    found_msg=Bool()
    found_msg.data=False
    cTlayer1_msg=ReferenceBlock()
    return found_msg,cTlayer1_msg

# %%
if __name__ == "__main__":
  rospy.init_node('First_layer_service')
  # client=rospy.client('First_layer_detection',FirstLayerPose)
  yolact_object=First_layer_client()
  
  found_msg=Bool()
  found_msg.data=False
  cTlayer1_msg=ReferenceBlock() 

  cv2.namedWindow("top3,bottom3")
  # cv2.namedWindow("TopGroups")
  # cv2.namedWindow("BottomGroups")
  cv2.namedWindow("Camera capture")
  try:
    # cycle until finding layer, after sending it
    while(not rospy.is_shutdown()):
      #wait for service
      rospy.wait_for_service('/FirstLayerPose')
      try:
        motion_service = rospy.ServiceProxy('/FirstLayerPose', FirstLayerPose)
        resp = motion_service(found_msg,cTlayer1_msg)
        print(resp)
      except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s"%e)

      if (found_msg.data):
        break

      k=-1
      # stay until correct frame captured
      while (k==-1 and not rospy.is_shutdown()):
        # Get frameset of color
        frames = yolact_object.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        # Validate that frame is valid
        if not color_frame:
            continue
        else:
          color_image = np.asanyarray(color_frame.get_data())
          cv2.imshow("Camera capture",color_image)
          k=cv2.waitKey(10)
      
      found_msg,cTlayer1_msg = yolact_object.first_layer_detection(color_image)

      cv2.imshow("top3,bottom3",yolact_object.img_imshow) #imshow in the main, on the concurrent image
      k=-1
      while(k==-1 and not rospy.is_shutdown()):
        k=cv2.waitKey(100) #red key in the image window

      # cv2.imshow("TopGroups",yolact_object.top_show) #imshow in the main, on the concurrent image
      # cv2.waitKey(0) #red key in the image window

      # cv2.imshow("BottomGroups",yolact_object.bottom_show) #imshow in the main, on the concurrent image
      # cv2.waitKey(0) #red key in the image window
  
  except KeyboardInterrupt:
    rospy.loginfo('Shutting down...')
  finally:
    cv2.destroyAllWindows()
    yolact_object.pipeline.stop()