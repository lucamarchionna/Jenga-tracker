#!/usr/bin/env python3

import numpy as np
import cv2
import quaternion

from Block_class import Block

class Blocks_group():

    def __init__(self,blocks_list,startIdx,img):
        self.startIdx=startIdx
        self.upIdx=-1
        self.downIdx=-1
        self.rightIdx=[-1,-1]
        self.leftIdx=[-1,-1]
        self.up1Idx=-1
        self.up1left0Idx=-1
        self.up1right0Idx=-1
        self.up1left1Idx=-1
        self.up1right1Idx=-1

        self.img_cross=img.copy()

        # Starting block object
        #self.start_block=blocks_list[self.startIdx]
        try:
            self.start_block=next((block for block in blocks_list if block.idx==self.startIdx))
        except StopIteration:
            print("Error: block with index: ",self.startIdx," not found in blocks list given")
            return

        # Void block objects
        self.up_block=Block(-1,[],[],[],-1)
        self.down_block=Block(-1,[],[],[],-1)
        self.right0_block=Block(-1,[],[],[],-1)
        self.right1_block=Block(-1,[],[],[],-1)
        self.left0_block=Block(-1,[],[],[],-1)
        self.left1_block=Block(-1,[],[],[],-1)
        self.up1_block=Block(-1,[],[],[],-1)
        self.up1left0_block=Block(-1,[],[],[],-1)
        self.up1right0_block=Block(-1,[],[],[],-1)
        self.up1left1_block=Block(-1,[],[],[],-1)
        self.up1right1_block=Block(-1,[],[],[],-1)

        if self.start_block.block_type!='front_face':
            print("Not a front face block")
            return
            
        test_up,test_down,test_right0,test_right1,test_left0,test_left1,test_upleft0,test_downleft0,\
            test_upright1,test_downright1,test_upright0,test_downright0,test_upleft1,test_downleft1,\
            test_up1,test_up1left0,test_up1right0,test_up1left1,test_up1right1=self.start_block.compute_test_points()
        self.img_cross=self.start_block.draw_test_points(self.img_cross)

        #TEST ALL
        #only first found, mutually exclusive
        for block_test in blocks_list:
            if block_test.idx!=self.startIdx:
                #Test up
                if self.upIdx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_up,False)==1):
                    self.upIdx=block_test.idx
                    self.up_block=block_test
                #Test up1
                if self.up1Idx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_up1,False)==1):
                    self.up1Idx=block_test.idx
                    self.up1_block=block_test
                #Test up1left0
                if  self.up1left0Idx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_up1left0,False)==1):
                    self.up1left0Idx=block_test.idx
                    self.up1left0_block=block_test
                #Test up1right0
                if self.up1right0Idx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_up1right0,False)==1):
                    self.up1right0Idx=block_test.idx
                    self.up1right0_block=block_test
                #Test down
                elif self.downIdx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_down,False)==1):
                    self.downIdx=block_test.idx
                    self.down_block=block_test
                #Test right0
                elif self.rightIdx[0]==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_right0,False)==1):
                    self.rightIdx[0]=block_test.idx
                    self.right0_block=block_test
                    #Then test right1
                    #only if right0 is front
                    if self.right0_block.block_type=='front_face':
                        _,_,test_right1,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_=self.right0_block.compute_test_points()
                        self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_right1,self.right0_block.centroid,cv2color=[0,0,255])
                        for block_test1 in blocks_list:
                            if block_test1.idx!=self.startIdx and block_test1.idx!=self.rightIdx[0]:  #to be sure
                                if self.rightIdx[1]==-1 and (cv2.pointPolygonTest(block_test1.contour_max,test_right1,False)==1):
                                    self.rightIdx[1]=block_test1.idx
                                    self.right1_block=block_test1
                #Test left0
                elif self.leftIdx[0]==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_left0,False)==1):
                    self.leftIdx[0]=block_test.idx
                    self.left0_block=block_test
                    #Then test left1
                    #only if left0 is front
                    if self.left0_block.block_type=='front_face':
                        _,_,_,_,test_left1,_,_,_,_,_,_,_,_,_,_,_,_,_,_=self.left0_block.compute_test_points()
                        self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_left1,self.left0_block.centroid,cv2color=[0,255,0])
                        for block_test1 in blocks_list:
                            if block_test1.idx!=self.startIdx and block_test1.idx!=self.leftIdx[0]:  #to be sure
                                if self.leftIdx[1]==-1 and (cv2.pointPolygonTest(block_test1.contour_max,test_left1,False)==1):
                                    self.leftIdx[1]=block_test1.idx
                                    self.left1_block=block_test1

        # Search for far block, if central is missing                                
        if self.rightIdx[0]==-1 and self.leftIdx[0]==-1:
            self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_right1,self.start_block.centroid,cv2color=[0,0,125])
            self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_left1,self.start_block.centroid,cv2color=[0,125,0])
            for block_test in blocks_list:
                if block_test.idx!=self.startIdx and block_test.idx!=self.upIdx and block_test.idx!=self.downIdx:  #to be sure
                    if self.rightIdx[1]==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_right1,False)==1):
                        self.rightIdx[1]=block_test.idx
                        self.right1_block=block_test
                    elif self.leftIdx[1]==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_left1,False)==1):
                        self.leftIdx[1]=block_test.idx
                        self.left1_block=block_test
        
        # Search how 2 horizontal blocks are placed
        self.caseA=False
        self.caseD=False
        self.caseB=False
        self.caseC=False
        self.caseH=False
        self.caseE=False
        self.caseG=False
        self.caseF=False

        #type1: 
        if self.rightIdx[0]!=-1 and self.rightIdx[1]==-1 and self.leftIdx[0]==-1:
            self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_upleft0,self.start_block.centroid,cv2color=[255,0,0])
            self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_downleft0,self.start_block.centroid,cv2color=[67,0,0])
            self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_upright1,self.start_block.centroid,cv2color=[0,0,255])
            self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_downright1,self.start_block.centroid,cv2color=[0,0,67])
            # search case A
            self.caseA=True
            if self.upIdx!=-1:
                for block_test in blocks_list:
                    if block_test.idx!=self.startIdx and (cv2.pointPolygonTest(block_test.contour_max,test_upleft0,False)==1):
                        self.caseA=False  
            if self.downIdx!=-1:
                for block_test in blocks_list:
                    if block_test.idx!=self.startIdx and (cv2.pointPolygonTest(block_test.contour_max,test_downleft0,False)==1):
                        self.caseA=False
            if not self.caseA:
                # search case D
                self.caseD=True
                if self.upIdx!=-1:
                    for block_test in blocks_list:
                        if block_test.idx!=self.startIdx and block_test.idx!=self.rightIdx[0]:
                            if (cv2.pointPolygonTest(block_test.contour_max,test_upright1,False)==1):
                                self.caseD=False  
                if self.downIdx!=-1:
                    for block_test in blocks_list:
                        if block_test.idx!=self.startIdx and block_test.idx!=self.rightIdx[0]:
                            if (cv2.pointPolygonTest(block_test.contour_max,test_downright1,False)==1):
                                self.caseD=False
                if not self.caseD:
                    # search case B
                    self.caseB=False
                    for block_test in blocks_list:
                        if block_test.idx==self.startIdx and (cv2.pointPolygonTest(block_test.contour_max,test_left0,False)==1):
                            self.caseB=True
                    if not self.caseB:
                        # sure to be case C
                        self.caseC=True
        
        elif self.leftIdx[0]!=-1 and self.leftIdx[1]==-1 and self.rightIdx[0]==-1:
            #type2: 
            self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_upright0,self.start_block.centroid,cv2color=[255,0,0])
            self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_downright0,self.start_block.centroid,cv2color=[67,0,0])
            self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_upleft1,self.start_block.centroid,cv2color=[0,0,255])
            self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_downleft1,self.start_block.centroid,cv2color=[0,0,67])
            # search case H
            self.caseH=True
            if self.upIdx!=-1:
                for block_test in blocks_list:
                    if block_test.idx!=self.startIdx and (cv2.pointPolygonTest(block_test.contour_max,test_upright0,False)==1):
                        self.caseH=False  
            if self.downIdx!=-1:
                for block_test in blocks_list:
                    if block_test.idx!=self.startIdx and (cv2.pointPolygonTest(block_test.contour_max,test_downright0,False)==1):
                        self.caseH=False
            if not self.caseH:
                # search case E
                self.caseE=True
                if self.upIdx!=-1:
                    for block_test in blocks_list:
                        if block_test.idx!=self.startIdx and block_test.idx!=self.leftIdx[0]:
                            if (cv2.pointPolygonTest(block_test.contour_max,test_upleft1,False)==1):
                                self.caseE=False  
                if self.downIdx!=-1:
                    for block_test in blocks_list:
                        if block_test.idx!=self.startIdx and block_test.idx!=self.leftIdx[0]:
                            if (cv2.pointPolygonTest(block_test.contour_max,test_downleft1,False)==1):
                                self.caseE=False
                if not self.caseE:
                    # search case G
                    self.caseG=False
                    for block_test in blocks_list:
                        if block_test.idx==self.startIdx and (cv2.pointPolygonTest(block_test.contour_max,test_right0,False)==1):
                            self.caseG=True
                    if not self.caseF:
                        # sure to be case F
                        self.caseF=True
        
        if self.caseA or (self.rightIdx[0]!=-1 and self.rightIdx[1]!=-1):
            if self.up1right0Idx!=-1:
                #Then test up1right1
                #only if up1right0 is front
                if self.up1right0_block.block_type=='front_face':
                    _,_,test1_up1right0,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_=self.up1right0_block.compute_test_points()
                    # self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test1_up1right0,self.up1right0_block.centroid,cv2color=[0,255,0])
                    for block_test1 in blocks_list:
                        if self.up1right1Idx==-1 and (cv2.pointPolygonTest(block_test1.contour_max,test1_up1right0,False)==1):
                            self.up1right1Idx=block_test1.idx
                            self.up1right1_block=block_test1
            else:
                #Then test up1right1
                #only if up1 is front
                if self.up1Idx!=-1 and self.up1_block.block_type=='front_face':
                    _,_,_,test1_up1right1,_,_,_,_,_,_,_,_,_,_,_,_,_,_,_=self.up1_block.compute_test_points()
                    # self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test1_up1right1,self.up1_block.centroid,cv2color=[0,255,0])
                    for block_test1 in blocks_list:
                        if self.up1right1Idx==-1 and (cv2.pointPolygonTest(block_test1.contour_max,test1_up1right1,False)==1):
                            self.up1right1Idx=block_test1.idx
                            self.up1right1_block=block_test1

        if self.caseH or (self.leftIdx[0]!=-1 and self.leftIdx[1]!=-1):
            if self.up1left0Idx!=-1:
                #Then test up1left1
                #only if up1left0 is front
                if self.up1Idx!=-1 and self.up1left0_block.block_type=='front_face':
                    _,_,_,_,test1_up1left1,_,_,_,_,_,_,_,_,_,_,_,_,_,_=self.up1left0_block.compute_test_points()
                    # self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test1_up1left1,self.up1left0_block.centroid,cv2color=[0,255,0])
                    for block_test1 in blocks_list:
                        if self.up1left1Idx==-1 and (cv2.pointPolygonTest(block_test1.contour_max,test1_up1left1,False)==1):
                            self.up1left1Idx=block_test1.idx
                            self.up1left1_block=block_test1
            else:
                #Then test up1left1
                #only if up1 is front
                if self.up1Idx!=-1 and self.up1_block.block_type=='front_face':
                    _,_,_,_,_,test1_up1left1,_,_,_,_,_,_,_,_,_,_,_,_,_=self.up1_block.compute_test_points()
                    # self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test1_up1left1,self.up1_block.centroid,cv2color=[0,255,0])
                    for block_test1 in blocks_list:
                        if self.up1left1Idx==-1 and (cv2.pointPolygonTest(block_test1.contour_max,test1_up1left1,False)==1):
                            self.up1left1Idx=block_test1.idx
                            self.up1left1_block=block_test1

    def print_idx(self):
        #Print indices
        print("Index target: ",self.startIdx)
        print("Index up: ",self.upIdx)
        print("Index down: ",self.downIdx)
        print("Index right 0: ",self.rightIdx[0])
        print("Index right 1: ",self.rightIdx[1])
        print("Index left 0: ",self.leftIdx[0])
        print("Index left 1: ",self.leftIdx[1])

    def get_drawn_search_img(self):
        return self.img_cross.copy()

    def get_blocks_group(self):
        return self.start_block,self.up_block,self.down_block,self.right0_block,self.right1_block,self.left0_block,self.left1_block

    def get_blocks_idxs(self):
        return self.startIdx,self.upIdx,self.downIdx,self.rightIdx,self.leftIdx

    def draw_masked_group(self,img):
        self.masked_group=img.copy()
        if self.startIdx!=-1:
            self.masked_group=self.start_block.draw_masked_approx(img)
        if self.upIdx!=-1:
            self.masked_group+=self.up_block.draw_masked_approx(img)
        if self.downIdx!=-1:
            self.masked_group+=self.down_block.draw_masked_approx(img)
        if self.rightIdx[0]!=-1:
            self.masked_group+=self.right0_block.draw_masked_approx(img)
        if self.rightIdx[1]!=-1:
            self.masked_group+=self.right1_block.draw_masked_approx(img)
        if self.leftIdx[0]!=-1:
            self.masked_group+=self.left0_block.draw_masked_approx(img)
        if self.leftIdx[1]!=-1:
            self.masked_group+=self.left1_block.draw_masked_approx(img)
        if self.up1Idx!=-1:
            self.masked_group+=self.up1_block.draw_masked_approx(img)
        if self.up1left0Idx!=-1:
            self.masked_group+=self.up1left0_block.draw_masked_approx(img)
        if self.up1right0Idx!=-1:
            self.masked_group+=self.up1right0_block.draw_masked_approx(img)    
        if self.up1left1Idx!=-1:
            self.masked_group+=self.up1left1_block.draw_masked_approx(img)
        if self.up1right1Idx!=-1:
            self.masked_group+=self.up1right1_block.draw_masked_approx(img)                                  

        if self.startIdx!=-1:
            self.masked_group=self.start_block.draw_corners(self.masked_group)
            cv2.circle(self.masked_group,self.start_block.centroid,3,[0,255,0],thickness=-1)
        if self.upIdx!=-1:
            self.masked_group=self.up_block.draw_corners(self.masked_group)
        if self.downIdx!=-1:
            self.masked_group=self.down_block.draw_corners(self.masked_group)
        if self.rightIdx[0]!=-1:
            self.masked_group=self.right0_block.draw_corners(self.masked_group)
        if self.rightIdx[1]!=-1:
            self.masked_group=self.right1_block.draw_corners(self.masked_group)
        if self.leftIdx[0]!=-1:
            self.masked_group=self.left0_block.draw_corners(self.masked_group)
        if self.leftIdx[1]!=-1:
            self.masked_group=self.left1_block.draw_corners(self.masked_group)

        return self.masked_group

    def is_project3D_target(self,rvec,tvec,cam_mtx,cam_dist,width_offset=80):
        #TODO: consider if projection is slightly outside all
        #TODO: use of position,orientation,(layer) to better filter the finding
        target_pt=np.zeros(3,dtype=np.float)
        proj_target,_=cv2.projectPoints(target_pt,rvec,tvec,cam_mtx,cam_dist)
        proj_target=np.squeeze(proj_target).astype(np.float)-np.array([width_offset,0],dtype=np.float)

        return (cv2.pointPolygonTest(self.start_block.contour_max,proj_target,False)==1)

    def setup_object_frame(self,b_width,b_height,b_length,target_T_o=np.eye(4)):
        self.b_width=b_width
        self.b_height=b_height
        self.b_length=b_length
        
        #assuming a reference in corner 0 of object, with x,y,z axes aligned to w,l,h dimensions
        o_Rot=np.eye(3)
        o_t=np.array([0.,0.,0.])
        o_T=np.eye(4)
        o_T[:3,:3]=o_Rot
        o_T[:3,3]=o_t
        o_move_target=np.array([0.,0.,0.])
        o_move_right0=np.array([b_width,0.,0.])
        o_move_right1=np.array([2*b_width,0.,0.])
        o_move_left0=np.array([-b_width,0.,0.])
        o_move_left1=np.array([-2*b_width,0.,0.])
        o_move_upleft=np.array([0.,b_width,b_height])
        # o_move_upcenter=np.array([-b_width,b_width,b_height])
        # o_move_upright=np.array([-2*b_width,b_width,b_height])
        o_move_downleft=np.array([0.,b_width,-b_height])
        # o_move_downcenter=np.array([-b_width,b_width,-b_height])
        # o_move_downright=np.array([-2*b_width,b_width,-b_height])
        o_move_up1=np.array([0,0,2*b_height])
        o_move_up1right0=np.array([b_width,0,2*b_height])
        o_move_up1left0=np.array([-b_width,0,2*b_height])
        o_move_up1right1=np.array([2*b_width,0,2*b_height])
        o_move_up1left1=np.array([-2*b_width,0,2*b_height])        
        o_quat_updown=np.quaternion(np.cos(-np.pi/4),0, 0, np.sin(-np.pi/4))
        o_Rot_updown=quaternion.as_rotation_matrix(o_quat_updown)
        o_T_upleft=np.eye(4)
        o_T_upleft[:3,:3]=o_Rot_updown
        o_T_upleft[:3,3]=o_move_upleft
        o_T_downleft=np.eye(4)
        o_T_downleft[:3,:3]=o_Rot_updown
        o_T_downleft[:3,3]=o_move_downleft

        o_corner0=np.array([0.,0.,0.])
        o_corner1=np.array([0.,0.,b_height])
        o_corner2=np.array([b_width,0.,b_height])
        o_corner3=np.array([b_width,0.,0.])

        #new frame
        self.target_T_o=target_T_o
        target_Rot_o=target_T_o[:3,:3]
        target_quat_o=quaternion.from_rotation_matrix(target_Rot_o)
        target_t_o=target_T_o[:3,3]

        def homogeneus_T_product(hom_T,vector):
            hom_vector=np.zeros(4)
            hom_vector[:3]=vector
            hom_vector[3]=1
            hom_product=hom_T@hom_vector
            return hom_product[:3]

        #new translations in new frame
        self.target_move_target=o_move_target.copy()
        self.target_move_right0=target_Rot_o@o_move_right0
        self.target_move_right1=target_Rot_o@o_move_right1
        self.target_move_left0=target_Rot_o@o_move_left0
        self.target_move_left1=target_Rot_o@o_move_left1
        self.target_move_up1=target_Rot_o@o_move_up1
        self.target_move_up1right0=target_Rot_o@o_move_up1right0
        self.target_move_up1left0=target_Rot_o@o_move_up1left0
        self.target_move_up1right1=target_Rot_o@o_move_up1right1
        self.target_move_up1left1=target_Rot_o@o_move_up1left1        
        self.target_move_upleft=homogeneus_T_product(self.target_T_o,homogeneus_T_product(o_T_upleft,-target_Rot_o.T@target_t_o))
        self.target_move_upcenter=homogeneus_T_product(self.target_T_o,homogeneus_T_product(o_T_upleft,-target_Rot_o.T@target_t_o))+self.target_move_left0
        self.target_move_upright=homogeneus_T_product(self.target_T_o,homogeneus_T_product(o_T_upleft,-target_Rot_o.T@target_t_o))+self.target_move_left1
        self.target_move_downleft=homogeneus_T_product(self.target_T_o,homogeneus_T_product(o_T_downleft,-target_Rot_o.T@target_t_o))
        self.target_move_downcenter=homogeneus_T_product(self.target_T_o,homogeneus_T_product(o_T_downleft,-target_Rot_o.T@target_t_o))+self.target_move_left0
        self.target_move_downright=homogeneus_T_product(self.target_T_o,homogeneus_T_product(o_T_downleft,-target_Rot_o.T@target_t_o))+self.target_move_left1
        #new rotations in new frame
        #target_quat_updown=quaternion.from_rotation_matrix(target_Rot_o@o_Rot_updown@target_Rot_o.T)
        target_quat_updown=target_quat_o*o_quat_updown*target_quat_o.conjugate()
        self.target_RotVec_updown=quaternion.as_rotation_vector(target_quat_updown)
        self.target_RotVec_target=np.array([0.,0.,0.])

        #new target corners in new frame
        self.target_corner0=homogeneus_T_product(self.target_T_o,o_corner0)
        self.target_corner1=homogeneus_T_product(self.target_T_o,o_corner1)
        self.target_corner2=homogeneus_T_product(self.target_T_o,o_corner2)
        self.target_corner3=homogeneus_T_product(self.target_T_o,o_corner3)
    
    def cao_file_write(self,cao_name,center_cao_path,right_cao_path,left_cao_path):
        self.cao_name=cao_name
        self.center_cao_path=center_cao_path
        self.right_cao_path=right_cao_path
        self.left_cao_path=left_cao_path

        def t_to_string(translation):
            return "t=["+str(round(translation[0],4))+"; "+str(round(translation[1],4))+"; "+str(round(translation[2],4))+"]"

        def tu_to_string(rotVect):
            return "tu=["+str(round(rotVect[0],4))+"; "+str(round(rotVect[1],4))+"; "+str(round(rotVect[2],4))+"]"
        
        def write_cao_line(block_name,translation,rotation):
            return 'load("'+block_name+'", '+t_to_string(translation)+', '+tu_to_string(rotation)+')\n'

        lines_to_write=[]
        with open(cao_name,"w") as file_cao:
            lines_to_write.append("V1\n")

            #block at center of horizontal
            if self.rightIdx[0]!=-1 and self.leftIdx[0]!=-1:
                lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_target,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_right0,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_left0,self.target_RotVec_target))
                if self.upIdx!=-1:   #keep same T not depending on target block on right or left side; front and rear faces depends on it though
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_upcenter,self.target_RotVec_updown))
                if self.downIdx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_downcenter,self.target_RotVec_updown))
                if self.up1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_up1,self.target_RotVec_target))
                if self.up1left0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_up1left0,self.target_RotVec_target))
                if self.up1right0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_up1right0,self.target_RotVec_target))                    

            #block at left of horizontal
            elif self.rightIdx[0]!=-1 and self.rightIdx[1]!=-1:
                lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_target,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_right0,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_right1,self.target_RotVec_target))
                if self.upIdx!=-1:   #keep same T not depending on target block on right or left side; front and rear faces depends on it though
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_upleft,self.target_RotVec_updown))
                if self.downIdx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_downleft,self.target_RotVec_updown))
                if self.up1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_up1,self.target_RotVec_target))                                
                if self.up1right0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_up1right0,self.target_RotVec_target))                    
                if self.up1right1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_up1right1,self.target_RotVec_target))                    

            #block at right of horizontal
            elif self.leftIdx[0]!=-1 and self.leftIdx[1]!=-1:
                lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_target,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_left0,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_left1,self.target_RotVec_target))
                if self.upIdx!=-1:   #keep same T not depending on target block on right or left side; front and rear faces depends on it though
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_upright,self.target_RotVec_updown))
                if self.downIdx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_downright,self.target_RotVec_updown))    
                if self.up1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_up1,self.target_RotVec_target))                                
                if self.up1left0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_up1left0,self.target_RotVec_target))                    
                if self.up1left1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_up1left1,self.target_RotVec_target))                    

            #missing central block, target at left of horizontal
            elif self.rightIdx[0]==-1 and self.rightIdx[1]!=-1 and self.leftIdx[0]==-1:
                lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_target,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_right1,self.target_RotVec_target))
                if self.upIdx!=-1:   #keep same T not depending on target block on right or left side; front and rear faces depends on it though
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_upleft,self.target_RotVec_updown))
                if self.downIdx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_downleft,self.target_RotVec_updown))
                if self.up1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_up1,self.target_RotVec_target))                                
                if self.up1right0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_up1right0,self.target_RotVec_target))                    
                if self.up1right1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_up1right1,self.target_RotVec_target))                    

            #missing central block, target at right of horizontal
            elif self.rightIdx[0]==-1 and self.leftIdx[0]==-1 and self.leftIdx[1]!=-1:
                lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_target,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_left1,self.target_RotVec_target))
                if self.upIdx!=-1:   #keep same T not depending on target block on right or left side; front and rear faces depends on it though
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_upright,self.target_RotVec_updown))
                if self.downIdx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_downright,self.target_RotVec_updown))    
                if self.up1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_up1,self.target_RotVec_target))                                
                if self.up1left0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_up1left0,self.target_RotVec_target))                    
                if self.up1left1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_up1left1,self.target_RotVec_target))                    

            ## Missing blocks conditions: 2 horizontal blocks
            # target external left, the other at center
            elif self.caseA or self.caseB:                
                lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_target,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_right0,self.target_RotVec_target))
                if self.upIdx!=-1:   #keep same T not depending on target block on right or left side; front and rear faces depends on it though
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_upleft,self.target_RotVec_updown))
                if self.downIdx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_downleft,self.target_RotVec_updown))
                if self.up1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_up1,self.target_RotVec_target))                                
                if self.up1right0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_up1right0,self.target_RotVec_target))                    
                if self.up1right1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_up1right1,self.target_RotVec_target))                                
            # target center, the other external left
            elif self.caseC or self.caseD:
                lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_target,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_right0,self.target_RotVec_target))
                if self.upIdx!=-1:   #keep same T not depending on target block on right or left side; front and rear faces depends on it though
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_upcenter,self.target_RotVec_updown))
                if self.downIdx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_downcenter,self.target_RotVec_updown))
                if self.up1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_up1,self.target_RotVec_target))
                if self.up1left0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_up1left0,self.target_RotVec_target))
                if self.up1right0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_up1right0,self.target_RotVec_target))                    
            
            ## Missing blocks conditions: 2 horizontal blocks
            # target center, the other external left
            elif self.caseE or self.caseF:                
                lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_target,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_left0,self.target_RotVec_target))
                if self.upIdx!=-1:   #keep same T not depending on target block on right or left side; front and rear faces depends on it though
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_upcenter,self.target_RotVec_updown))
                if self.downIdx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_downcenter,self.target_RotVec_updown))
                if self.up1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_up1,self.target_RotVec_target))
                if self.up1left0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_up1left0,self.target_RotVec_target))
                if self.up1right0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_up1right0,self.target_RotVec_target))                    

            # target external right, the other at center
            elif self.caseG or self.caseH:
                lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_target,self.target_RotVec_target))
                lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_left0,self.target_RotVec_target))
                if self.upIdx!=-1:   #keep same T not depending on target block on right or left side; front and rear faces depends on it though
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_upright,self.target_RotVec_updown))
                if self.downIdx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_downright,self.target_RotVec_updown))
                if self.up1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.right_cao_path,self.target_move_up1,self.target_RotVec_target))                                
                if self.up1left0Idx!=-1:
                    lines_to_write.append(write_cao_line(self.center_cao_path,self.target_move_up1left0,self.target_RotVec_target))                    
                if self.up1left1Idx!=-1:
                    lines_to_write.append(write_cao_line(self.left_cao_path,self.target_move_up1left1,self.target_RotVec_target))                    

            #TERMINATE FILE; WRITE IN IT THE LINES
            for line in lines_to_write:
                file_cao.write(line)
            file_cao.write("###############################################################\n# 3D Points\n0\n# 3D Lines\n0 # Number of lines\n# Faces from 3D lines\n0 # Number of faces\n# Faces from 3D points\n0\n# 3D cylinders\n0 # Number of cylinders\n# 3D circles\n0 # Number of circles")

    def corners_print_draw(self,img):
        numb_img=img.copy()
        i=0
        if self.startIdx!=-1:
            for corner in self.start_block.corners:
                cv2.putText(numb_img,str(i),corner,cv2.FONT_HERSHEY_PLAIN,1,[0,255,0])
                print(str(i)+" : ",corner)
                i=i+1
            print("-"*20)
        if self.upIdx!=-1:
            for corner in self.up_block.corners:
                cv2.putText(numb_img,str(i),corner,cv2.FONT_HERSHEY_PLAIN,1,[0,255,0])
                print(str(i)+" : ",corner)
                i=i+1
            print("-"*20)
        if self.downIdx!=-1:
            for corner in self.down_block.corners:
                cv2.putText(numb_img,str(i),corner,cv2.FONT_HERSHEY_PLAIN,1,[0,255,0])
                print(str(i)+" : ",corner)
                i=i+1
            print("-"*20)
        if self.rightIdx[0]!=-1:
            for corner in self.right0_block.corners:
                cv2.putText(numb_img,str(i),corner,cv2.FONT_HERSHEY_PLAIN,1,[0,255,0])
                print(str(i)+" : ",corner)
                i=i+1
            print("-"*20)
        if self.rightIdx[1]!=-1:
            for corner in self.right1_block.corners:
                cv2.putText(numb_img,str(i),corner,cv2.FONT_HERSHEY_PLAIN,1,[0,255,0])
                print(str(i)+" : ",corner)
                i=i+1
            print("-"*20)
        if self.leftIdx[0]!=-1:
            for corner in self.left0_block.corners:
                cv2.putText(numb_img,str(i),corner,cv2.FONT_HERSHEY_PLAIN,1,[0,255,0])
                print(str(i)+" : ",corner)
                i=i+1
            print("-"*20)
        if self.leftIdx[1]!=-1:
            for corner in self.left1_block.corners:
                cv2.putText(numb_img,str(i),corner,cv2.FONT_HERSHEY_PLAIN,1,[0,255,0])
                print(str(i)+" : ",corner)
                i=i+1
            print("-"*20)

        return numb_img

    def initPose_file_write(self,width_offset,initPose_name,cam_mtx,cam_dist):
        self.init_objp=[]
        self.init_impt=[]

        def ordered_objp(target_move):
            ordered_objp=[self.target_corner0,self.target_corner1,self.target_corner2,self.target_corner3]
            ordered_objp_moved=[]
            for objp in ordered_objp:
                ordered_objp_moved.append(objp+target_move)
            return ordered_objp_moved

        if self.startIdx!=-1:
            ordered_corners=self.start_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_target))

        # if self.upIdx!=-1:
        #     ordered_corners=self.up_block.ordered_corners()
        #     if len(ordered_corners)>0:
        #         self.init_impt.append(ordered_corners)
        # if self.downIdx!=-1:
        #     ordered_corners=self.down_block.ordered_corners()
        #     if len(ordered_corners)>0:
        #         self.init_impt.append(ordered_corners)

        if self.rightIdx[0]!=-1:
            ordered_corners=self.right0_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_right0))
        if self.rightIdx[1]!=-1:
            ordered_corners=self.right1_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_right1))       
        if self.leftIdx[0]!=-1:
            ordered_corners=self.left0_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_left0))
        if self.leftIdx[1]!=-1:
            ordered_corners=self.left1_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_left1))
        if self.up1Idx!=-1 and self.up1_block.block_type=="front_face":
            ordered_corners=self.up1_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_up1))
        if self.up1left0Idx!=-1 and self.up1left0_block.block_type=="front_face":
            ordered_corners=self.up1left0_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_up1left0))
        if self.up1left1Idx!=-1 and self.up1left1_block.block_type=="front_face":
            ordered_corners=self.up1left1Idx.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_up1left1))       
        if self.up1right0Idx!=-1 and self.up1right0_block.block_type=="front_face":
            ordered_corners=self.up1right0_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_up1right0))
        if self.up1right1Idx!=-1 and self.up1right1_block.block_type=="front_face":
            ordered_corners=self.up1right1_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_up1right1))

        self.init_impt2=[]
        for impt_corners_list in self.init_impt:
            for impt_corner in impt_corners_list:
                #Add offset coming from making image squared
                impt_corner[0]+=width_offset
                self.init_impt2.append(impt_corner)
        init_impt=np.array(self.init_impt2,dtype=np.float)
        self.init_objp2=[]
        for obj_corners_list in self.init_objp:
            for obj_corner in obj_corners_list:
                self.init_objp2.append(obj_corner)
        init_objp=np.array(self.init_objp2)

        initialGuess=False
        iters=100
        reprError=8.0
        confidence=0.99
        ret, rvecs, tvecs, inliers = cv2.solvePnPRansac(init_objp,init_impt,cam_mtx,cam_dist,\
            useExtrinsicGuess=initialGuess,iterationsCount=iters,reprojectionError=reprError,confidence=confidence)
        
        with open(initPose_name,"w") as file_initPose:
            for t_cord in tvecs:
                file_initPose.write(str(t_cord[0])+"\n")
            for r_cord in rvecs:
                file_initPose.write(str(r_cord[0])+"\n")
        
        return tvecs,rvecs

if __name__=='__main__':
    print("Must be used as a class")