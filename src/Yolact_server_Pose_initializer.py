#!/usr/bin/env python3

import rospy
from tracker_visp.srv import *
from tracker_visp.msg import *
from std_msgs.msg import String
#from cv_bridge import CvBridge

# %%
import numpy as np
import time
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
  
  request=PoseEstimationResponse(cao_msg,initPose_msg)

  return request
  

# %%
class Yolact_pose_service():
  def __init__(self):
    self.msg_image=None
    self.width=480
    self.height=480
    self.img_imshow=np.zeros((self.width,self.height*2,3),dtype=np.uint8)
    #Init stuff to enable loop
    self.newImage=False
    self.keyFromWindow=0

    self.rosPath=rosPath
    self.yolact_path=yolact_path

    #cfg_name='yolact_resnet101_jenga_dataset_config'
    self.cfg_name='yolact_resnet101_jenga_dataset_new_config'
    #weights_path=yolact_path+'/weights/yolact_resnet101_jenga_dataset_1916_230000.pth'
    #weights_path=yolact_path+'weights/yolact_resnet101_jenga_dataset_3399_340000.pth'
    #weights_path=yolact_path+'weights/yolact_resnet101_jenga_dataset_2099_210000.pth'
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

    self.s=rospy.Service('/PoseEstimation',PoseEstimation,self.pose_service_handle)

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
  def pose_service_handle(self,req):

    print("Service called")
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

    color_image = np.frombuffer(msg_image.data, dtype=np.uint8).reshape(msg_image.height, msg_image.width, -1).copy()
    color_image=cv2.cvtColor(color_image,cv2.COLOR_RGB2BGR)
    cam_mtx = np.array(cam_info.K).reshape([3, 3])
    cam_dist = np.array(cam_info.D)
    # %%
    img_name=os.path.join(self.yolact_path,"input_images",str(time.time())+".png")
    # print(img_name)
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
      return to_PoseEstimationResponse("")

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
    # cv2.imwrite(img_name[:-4]+'_ALL'+img_name[-4:],img_all_masks)

    # %%
    # for block in blocks_list:
    #     # DRAW STUFF
    #     contouredAll_centroid_img=block.draw_contoursAll_centroid(img)
    #     maskedSingle_approx=block.draw_masked_approx(img)
    #     masked_corners=block.draw_corners(maskedSingle_approx)

    # %%
    if len(blocks_list)<0:
      rospy.loginfo("SERVICE HALTED BY USER")
      return to_PoseEstimationResponse("")
    else:
      ### LIST ALL GROUPS ###
      blocks_groups_list=[]
      for block in blocks_list:
        #idx=range(3,len(blocks_list)-4)    #find between not top and not bottom floors, if they are ordered
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
      self.newImage=True # signal to the main that a new image is received
      img_zero=np.zeros(img_all_masks.shape,dtype=np.uint8)
      self.img_imshow=np.hstack((img_all_masks,img_zero))
      self.keyFromWindow=0
      rospy.loginfo("No block found for policy from projection")
      return to_PoseEstimationResponse("")

    # Block found, draw and use its group
    rospy.loginfo("\nConfirm group pressing 'c', exit pressing 'esc'\n")
    masked_group=choice_group.draw_masked_group(img)
    # img_test_points=choice_group.get_drawn_search_img()
    self.newImage=True # signal to the main that a new image is received
    self.img_imshow=np.hstack((img_all_masks,masked_group))
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
        self.img_imshow=np.zeros((self.width,self.height*2,3),dtype=np.uint8)
        return to_PoseEstimationResponse("")
      if (k==ord('c')):
        chosen_blocks_group=choice_group
        cv2.imwrite(img_name[:-3]+'-'+str(time.time())+img_name[-4:],self.img_imshow) 
    # %%
    # selected_group=False
    # chosen_img=np.zeros((self.width,self.height*2,3),dtype=np.uint8)
    # rospy.loginfo("\nChoose one group pressing 'c', pass pressing 'q', exit pressing 'esc'\n")
    # while(not selected_group):
    #   for random_block_group in blocks_groups_list:
    #     masked_group=random_block_group.draw_masked_group(img)
    #     img_test_points=random_block_group.get_drawn_search_img()
    #     self.newImage=True # signal to the main that a new image is received
    #     # self.img_imshow=np.hstack((img_all_masks,masked_group))
    #     self.img_imshow=np.hstack((img_test_points,masked_group))
    #     # print("Cases:",random_block_group.caseA,random_block_group.caseB,random_block_group.caseC,random_block_group.caseD)
    #     while(self.keyFromWindow==0):
    #       # Wait until key is chosen in the main loop
    #       continue
    #     else:
    #       # rospy.loginfo("Received key: "+chr(self.keyFromWindow))
    #       # get ready for next image 
    #       k=self.keyFromWindow
    #       self.keyFromWindow=0
    #       ##
    #       # exit if pressed 'esc'
    #       ## WARNING: RETURN HERE ##
    #       ##
    #       if (k==27 and not selected_group):
    #         rospy.loginfo("SERVICE HALTED BY USER")
    #         self.img_imshow=np.zeros((self.width,self.height*2,3),dtype=np.uint8)
    #         return to_PoseEstimationResponse("")
    #       if (k==ord('c') and not selected_group):
    #         chosen_blocks_group=random_block_group
    #         selected_group=True
    #         chosen_img=self.img_imshow
    #         cv2.imwrite(img_name[:-3]+'-'+str(time.time())+img_name[-4:],self.img_imshow)

    # self.img_imshow=chosen_img.copy()      
    
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

    # %%
    rospy.loginfo("---\nSUCCESFULLY ENDED\n---")
    return to_PoseEstimationResponse(cao_name,rvec,tvec,"",0)

# %%
if __name__ == "__main__":
  rospy.init_node('Yolact_server')
  yolact_object=Yolact_pose_service()
  cv2.namedWindow("c:choose,esc:exit")
  try:
    while(not rospy.is_shutdown()):
      k=0 #to enter in the loop
      # stay in the loop until q or c is pressed
      while (k!=ord('c') and k!=27 and not rospy.is_shutdown()):
        cv2.imshow("c:choose,esc:exit",yolact_object.img_imshow) #imshow in the main, on the concurrent image
        k=cv2.waitKey(10) #red key in the image window
      # now a key has been pressed
      # pass the key only if a new image has been received
      if (yolact_object.newImage and not rospy.is_shutdown()):
        yolact_object.keyFromWindow=k
        yolact_object.newImage=False  #return waiting for a new image
    rospy.spin()
  except KeyboardInterrupt:
    rospy.loginfo('Shutting down...')
    cv2.destroyAllWindows()
  finally:
    cv2.destroyAllWindows()