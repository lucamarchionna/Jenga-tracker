#!/usr/bin/env python3

from numpy.core.fromnumeric import reshape
import rospy
from tracker_visp.srv import YolactInitializeCaoPose
from geometry_msgs.msg import Transform
from std_msgs.msg import String
#from cv_bridge import CvBridge

# %%
import numpy as np
import pyrealsense2 as rs
import time
from warnings import catch_warnings
with catch_warnings():
    import quaternion

#!python
#cython: language_level=3

import torch
import cv2
import sys
import os
import rospkg
# %%
from Block_class import Block
from Group_finder_writer import Blocks_group

# %%
rospack = rospkg.RosPack()
tracker_path = rospack.get_path('tracker_visp')
yolact_path = os.path.join(tracker_path, "yolact_cpu")

if not sys.path.__contains__(yolact_path):
    sys.path.append(yolact_path)
from yolact import Yolact
from data import set_cfg
from utils.augmentations import FastBaseTransform
from layers.output_utils import postprocess

# %%
#cfg_name='yolact_resnet101_jenga_dataset_config'
cfg_name='yolact_resnet101_jenga_dataset_new_config'
#weights_path=yolact_path+'/weights/yolact_resnet101_jenga_dataset_1916_230000.pth'
#weights_path=yolact_path+'weights/yolact_resnet101_jenga_dataset_3399_340000.pth'
#weights_path=yolact_path+'weights/yolact_resnet101_jenga_dataset_2099_210000.pth'
weights_path = os.path.join(yolact_path, "weights/yolact_resnet101_jenga_dataset_new_1199_180000.pth")

##### Setup #####
if torch.cuda.is_available():
  torch.backends.cudnn.fastest = True
  torch.set_default_tensor_type('torch.cuda.FloatTensor')
  set_cfg(cfg_name)
  net = Yolact().cuda()
else: #cpu only
  torch.set_default_tensor_type('torch.FloatTensor')
  set_cfg(cfg_name)
  net=Yolact().cpu()

transform = FastBaseTransform()

net.load_weights(weights_path)
net.eval()

# %%
def compute_outputs(img, net,transform, score_threshold):
  h, w, _ = img.shape
  tensor_img=torch.from_numpy(img)
  if torch.cuda.is_available():
    cuda_img=tensor_img.cuda().float()
    batch = transform(cuda_img.unsqueeze(0))
  else: #cpu only
    cpu_img=tensor_img.cpu().float()
    batch = transform(cpu_img.unsqueeze(0))

  with torch.no_grad():
    preds = net(batch)

  classes, scores, boxes, masks = postprocess(preds , w, h, score_threshold=score_threshold)

  classes=classes.cpu()
  scores=scores.cpu()
  boxes=boxes.cpu()
  masks=masks.cpu()

  return classes, scores, boxes, masks

# %%
def pose_service_handle(req):
  # %%
  ## Convert messages into opencv formats
  msg_image=req.image
  cam_info=req.camInfo
  # bridge = CvBridge()
  # color_image = bridge.imgmsg_to_cv2(msg_image, desired_encoding='passthrough')
  color_image = np.frombuffer(msg_image.data, dtype=np.uint8).reshape(msg_image.height, msg_image.width, -1).copy()
  color_image=cv2.cvtColor(color_image,cv2.COLOR_RGB2BGR)
  cam_mtx = np.array(cam_info.K).reshape([3, 3])
  cam_dist = np.array(cam_info.D)
  # print(cam_mtx)
  # print(cam_dist)
  # %%  
  img_name = yolact_path + "/input_images/" + str(time.time()) + ".png"
  cv2.imwrite(img_name,color_image)

  # %%
  ### CROP IMAGE TO BE SQUARE
  hhh,www,_=color_image.shape
  width_offset=int(www-hhh)/2
  img=color_image[int(hhh/2)-240:int(hhh/2)+240,int(www/2)-240:int(www/2)+240]

  # %%
  # Detection
  classes, scores, boxes, masks = compute_outputs(img,net,transform,0.5)
  print("Detected --> ",len(masks))

  # %%
  if len(masks)==0:
    exit(True)

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
  # img_all_masks=np.zeros(img.shape,dtype=np.uint8)
  # img_masks_idx=np.zeros(img.shape,dtype=np.uint8)
  # for block in blocks_list:
  #   img_all_masks+=block.draw_masked_single(img)
  #   img_masks_idx+=block.draw_masked_single(img)
  #   cv2.putText(img_masks_idx,str(block.idx),block.centroid,cv2.FONT_HERSHEY_COMPLEX,0.5,[0,0,255])

  # %%
  img_all_masks=np.zeros(img.shape,dtype=np.uint8)
  # img_masks_idx=np.zeros(img.shape,dtype=np.uint8)
  for block in blocks_list:
    img_all_masks+=block.draw_masked_approx(img)
    # img_masks_idx+=block.draw_masked_approx(img)
    # cv2.putText(img_masks_idx,str(block.idx),block.centroid,cv2.FONT_HERSHEY_COMPLEX,0.5,[0,0,255])

  #Save image
  cv2.imwrite(img_name[:-4]+'_ALL'+img_name[-4:],img_all_masks)

  # %%
  # for block in blocks_list:
  #     # DRAW STUFF    
  #     contouredAll_centroid_img=block.draw_contoursAll_centroid(img)
  #     maskedSingle_approx=block.draw_masked_approx(img)
  #     masked_corners=block.draw_corners(maskedSingle_approx)
      
  # %%
  if len(blocks_list)<0:
      exit(True)
  else:
      ### LIST ALL GROUPS ###
      blocks_groups_list=[]    
      for block in blocks_list:
          #idx=range(3,len(blocks_list)-4)    #find between not top and not bottom floors, if they are ordered
          if block.block_type=='front_face':
              idx=block.idx
              blocks_group=Blocks_group(blocks_list,idx,img)
              blocks_groups_list.append(blocks_group)

  # %%
  ## MANUAL GROUP CHOICE FROM LIST ##
  ## DRAW STUFF
  #SAVE STUFF

  selected_group=False
  while(not selected_group):
    print("\nChoose one group pressing 'c', pass pressing 'q'\n")
    cv2.namedWindow('Group')
    for random_block_group in blocks_groups_list:
        masked_group=random_block_group.draw_masked_group(img)
        img_test_points=random_block_group.get_drawn_search_img()

        k=0
        while (k!=ord('q') and k!=ord('c')):
            img_stack=np.hstack((img_test_points,masked_group))
            cv2.imshow('Group',img_stack)
            k=cv2.waitKey(10)
        if (k==ord('c') and not selected_group):
            chosen_blocks_group=random_block_group
            selected_group=True
            cv2.imwrite(img_name[:-3]+'-'+str(time.time())+img_name[-4:],img_stack)
        
    cv2.destroyAllWindows()

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
  center_cao_path = os.path.join(tracker_path, "model/blocks_cao/jengaCenterZend.cao")
  right_cao_path= os.path.join(tracker_path, "model/blocks_cao/jengaRightZend.cao")
  left_cao_path = os.path.join(tracker_path, "model/blocks_cao/jengaLeftZend.cao")


  # %%
  chosen_blocks_group.setup_object_frame(b_width,b_height,b_length,zend_T_o)

  # %%
  #cao_name="../model/jenga_"+str(time.time())+".cao"
  cao_name=os.path.join(tracker_path, "model/file_cao.cao")
  chosen_blocks_group.cao_file_write(cao_name,center_cao_path,right_cao_path,left_cao_path)

  # %%
  # img_plot=chosen_blocks_group.corners_print_draw(chosen_blocks_group.draw_masked_group(img))
  # cv2.imshow('frame',img_plot)

  # %%
  initPose_file_name= os.path.join(tracker_path, "model/file_init.pos")

  # CAMERA MATRICES
  # cam_mtx=np.array([[598.365,0,327.306],[0,600.842,245.576],[0,0,1]])
  # cam_dist=[]
  # cam_dist=np.array([[0.0344852993,0.793592898,0.00571190879,-0.00303585594,-3.21784069]])
  tvec,rvec=chosen_blocks_group.initPose_file_write(width_offset,initPose_file_name,cam_mtx,cam_dist)
  
  # %%
  tvec=np.squeeze(tvec)
  rvec=np.squeeze(rvec)
  cao_name_msg=String()
  cao_name_msg.data=cao_name
  init_pose_msg=Transform()
  init_pose_msg.translation.x=tvec[0]
  init_pose_msg.translation.y=tvec[1]
  init_pose_msg.translation.z=tvec[2]
  rvec_quat=quaternion.from_rotation_vector(rvec)
  init_pose_msg.rotation.x=rvec_quat.x
  init_pose_msg.rotation.y=rvec_quat.y
  init_pose_msg.rotation.z=rvec_quat.z
  init_pose_msg.rotation.w=rvec_quat.w
  print("---\nSUCCESFULLY ENDED\n---")
  return {'caoFilePath':cao_name_msg,'initPose':init_pose_msg}

# %%
if __name__ == "__main__":
  rospy.init_node('Yolact_server')
  s=rospy.Service('Pose_cao_initializer', YolactInitializeCaoPose, pose_service_handle)
  rospy.spin()