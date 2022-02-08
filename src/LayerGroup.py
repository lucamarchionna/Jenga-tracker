#!/usr/bin/env python3

from cv2 import projectPoints
import numpy as np
import cv2
import quaternion

from Block_class import Block

class Layer_group():

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
        self.down1Idx=-1
        self.down1left0Idx=-1
        self.down1right0Idx=-1
        self.down1left1Idx=-1
        self.down1right1Idx=-1        

        self.img_cross=img.copy()

        try:
            self.start_block=next((block for block in blocks_list if block.idx==self.startIdx))
        except StopIteration:
            print("Error: block with index: ",self.startIdx," not found in blocks list given")
            return

        # Void block objects
        self.right0_block=Block(-1,[],[],[],-1)
        self.right1_block=Block(-1,[],[],[],-1)
        self.left0_block=Block(-1,[],[],[],-1)
        self.left1_block=Block(-1,[],[],[],-1)
        self.down_block=Block(-1,[],[],[],-1)
        self.down1left0_block=Block(-1,[],[],[],-1)
        self.down1right0_block=Block(-1,[],[],[],-1)
        self.down1left1_block=Block(-1,[],[],[],-1)
        self.down1right1_block=Block(-1,[],[],[],-1)        

        if self.start_block.block_type!='front_face':
            print("Not a front face block")
            return
            
    def init_down(self,blocks_list):

        test_down,test_right0,test_right1,test_left0,test_left1,\
            test_down1,test_down1left0,test_down1right0=self.start_block.compute_down_test_points()
        self.img_cross=self.start_block.draw_layer_test_points(self.img_cross)

        #TEST Right left
        #only first found, mutually exclusive
        for block_test in blocks_list:
            if block_test.idx!=self.startIdx:
                # #Test down
                # if self.downIdx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_down,False)==1):
                #     self.downIdx=block_test.idx
                #     self.down_block=block_test
                #Test right0
                if self.rightIdx[0]==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_right0,False)==1):
                    self.rightIdx[0]=block_test.idx
                    self.right0_block=block_test
                    # #Then test right1
                    # #only if right0 is front
                    # if self.right0_block.block_type=='front_face':
                    #     _,test_right1,_,_,_,_,_,_=self.right0_block.compute_down_test_points()
                    #     self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_right1,self.right0_block.centroid,cv2color=[0,0,255])
                    #     for block_test1 in blocks_list:
                    #         if block_test1.idx!=self.startIdx and block_test1.idx!=self.rightIdx[0]:  #to be sure
                    #             if self.rightIdx[1]==-1 and (cv2.pointPolygonTest(block_test1.contour_max,test_right1,False)==1):
                    #                 self.rightIdx[1]=block_test1.idx
                    #                 self.right1_block=block_test1
                #Test left0
                elif self.leftIdx[0]==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_left0,False)==1):
                    self.leftIdx[0]=block_test.idx
                    self.left0_block=block_test
                    # #Then test left1
                    # #only if left0 is front
                    # if self.left0_block.block_type=='front_face':
                    #     _,_,_,test_left1,_,_,_,_=self.left0_block.compute_down_test_points()
                    #     self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_left1,self.left0_block.centroid,cv2color=[0,255,0])
                    #     for block_test1 in blocks_list:
                    #         if block_test1.idx!=self.startIdx and block_test1.idx!=self.leftIdx[0]:  #to be sure
                    #             if self.leftIdx[1]==-1 and (cv2.pointPolygonTest(block_test1.contour_max,test_left1,False)==1):
                    #                 self.leftIdx[1]=block_test1.idx
                    #                 self.left1_block=block_test1                
                #Test down1
                elif self.down1Idx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_down1,False)==1):
                    self.down1Idx=block_test.idx
                    self.down1_block=block_test
                #Test down1left0
                elif  self.down1left0Idx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_down1left0,False)==1):
                    self.down1left0Idx=block_test.idx
                    self.down1left0_block=block_test
                #Test down1right0
                elif self.down1right0Idx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_down1right0,False)==1):
                    self.down1right0Idx=block_test.idx
                    self.down1right0_block=block_test

    def init_up(self,blocks_list):

        test_up,test_right0,test_right1,test_left0,test_left1,\
            test_up1,test_up1left0,test_up1right0=self.start_block.compute_up_test_points()
        self.img_cross=self.start_block.draw_layer_test_points(self.img_cross)

        #TEST Right left
        #only first found, mutually exclusive
        for block_test in blocks_list:
            if block_test.idx!=self.startIdx:
                # #Test up
                # if self.upIdx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_up,False)==1):
                #     self.upIdx=block_test.idx
                #     self.up_block=block_test
                #Test right0
                if self.rightIdx[0]==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_right0,False)==1):
                    self.rightIdx[0]=block_test.idx
                    self.right0_block=block_test
                    # #Then test right1
                    # #only if right0 is front
                    # if self.right0_block.block_type=='front_face':
                    #     _,test_right1,_,_,_,_,_,_=self.right0_block.compute_up_test_points()
                    #     self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_right1,self.right0_block.centroid,cv2color=[0,0,255])
                    #     for block_test1 in blocks_list:
                    #         if block_test1.idx!=self.startIdx and block_test1.idx!=self.rightIdx[0]:  #to be sure
                    #             if self.rightIdx[1]==-1 and (cv2.pointPolygonTest(block_test1.contour_max,test_right1,False)==1):
                    #                 self.rightIdx[1]=block_test1.idx
                    #                 self.right1_block=block_test1
                #Test left0
                elif self.leftIdx[0]==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_left0,False)==1):
                    self.leftIdx[0]=block_test.idx
                    self.left0_block=block_test
                    # #Then test left1
                    # #only if left0 is front
                    # if self.left0_block.block_type=='front_face':
                    #     _,_,_,test_left1,_,_,_,_=self.left0_block.compute_up_test_points()
                    #     self.img_cross=self.start_block.draw_additional_test_point(self.img_cross,test_left1,self.left0_block.centroid,cv2color=[0,255,0])
                    #     for block_test1 in blocks_list:
                    #         if block_test1.idx!=self.startIdx and block_test1.idx!=self.leftIdx[0]:  #to be sure
                    #             if self.leftIdx[1]==-1 and (cv2.pointPolygonTest(block_test1.contour_max,test_left1,False)==1):
                    #                 self.leftIdx[1]=block_test1.idx
                    #                 self.left1_block=block_test1                    
                #Test up1
                elif self.up1Idx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_up1,False)==1):
                    self.up1Idx=block_test.idx
                    self.up1_block=block_test
                #Test up1left0
                elif  self.up1left0Idx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_up1left0,False)==1):
                    self.up1left0Idx=block_test.idx
                    self.up1left0_block=block_test
                #Test up1right0
                elif self.up1right0Idx==-1 and (cv2.pointPolygonTest(block_test.contour_max,test_up1right0,False)==1):
                    self.up1right0Idx=block_test.idx
                    self.up1right0_block=block_test
    
    def is_central(self):
        #block at center of horizontal
        return (self.rightIdx[0]!=-1 and self.leftIdx[0]!=-1)

    def print_idx(self):
        #Print indices
        print("Index target: ",self.startIdx)
        print("Index right 0: ",self.rightIdx[0])
        print("Index left 0: ",self.leftIdx[0])
        print("Index up: ",self.up1Idx)
        print("Index up1 right 0: ",self.up1right0Idx)
        print("Index up1 left 0: ",self.up1left0Idx)
        print("Index down: ",self.down1Idx)
        print("Index down1 right 0: ",self.down1right0Idx)
        print("Index down1 left 0: ",self.down1left0Idx)

    def get_drawn_search_img(self):
        return self.img_cross.copy()

    # def get_blocks_group(self):
    #     return self.start_block,self.up_block,self.down_block,self.right0_block,self.right1_block,self.left0_block,self.left1_block

    # def get_blocks_idxs(self):
    #     return self.startIdx,self.upIdx,self.downIdx,self.rightIdx,self.leftIdx

    def draw_masked_group(self,img):
        self.masked_group=img.copy()
        if self.startIdx!=-1:
            self.masked_group=self.start_block.draw_masked_approx(img)
        if self.rightIdx[0]!=-1:
            self.masked_group+=self.right0_block.draw_masked_approx(img)
        if self.rightIdx[1]!=-1:
            self.masked_group+=self.right1_block.draw_masked_approx(img)
        if self.leftIdx[0]!=-1:
            self.masked_group+=self.left0_block.draw_masked_approx(img)
        if self.leftIdx[1]!=-1:
            self.masked_group+=self.left1_block.draw_masked_approx(img)                               
        if self.up1Idx!=-1 and self.up1_block.block_type=="front_face":
            self.masked_group=self.start_block.draw_masked_approx(img)
        if self.up1right0Idx!=-1 and self.up1right0_block.block_type=="front_face":
            self.masked_group+=self.up1right0_block.draw_masked_approx(img)
        if self.up1left0Idx!=-1 and self.up1left0_block.block_type=="front_face":
            self.masked_group+=self.up1left0_block.draw_masked_approx(img)
        if self.down1Idx!=-1 and self.down1_block.block_type=="front_face":
            self.masked_group=self.start_block.draw_masked_approx(img)
        if self.down1right0Idx!=-1 and self.down1right0_block.block_type=="front_face":
            self.masked_group+=self.down1right0_block.draw_masked_approx(img)
        if self.down1left0Idx!=-1 and self.down1left0_block.block_type=="front_face":
            self.masked_group+=self.down1left0_block.draw_masked_approx(img)            

        if self.startIdx!=-1:
            self.masked_group=self.start_block.draw_corners(self.masked_group)
            cv2.circle(self.masked_group,self.start_block.centroid,3,[0,255,0],thickness=-1)
        if self.rightIdx[0]!=-1:
            self.masked_group=self.right0_block.draw_corners(self.masked_group)
        if self.rightIdx[1]!=-1:
            self.masked_group=self.right1_block.draw_corners(self.masked_group)
        if self.leftIdx[0]!=-1:
            self.masked_group=self.left0_block.draw_corners(self.masked_group)
        if self.leftIdx[1]!=-1:
            self.masked_group=self.left1_block.draw_corners(self.masked_group)

        return self.masked_group

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
        o_move_down1=np.array([0,0,-2*b_height])
        o_move_down1right0=np.array([b_width,0,-2*b_height])
        o_move_down1left0=np.array([-b_width,0,-2*b_height])
        o_move_up1=np.array([0,0,2*b_height])
        o_move_up1right0=np.array([b_width,0,2*b_height])
        o_move_up1left0=np.array([-b_width,0,2*b_height])        

        o_corner0=np.array([0.,0.,0.])
        o_corner1=np.array([0.,0.,b_height])
        o_corner2=np.array([b_width,0.,b_height])
        o_corner3=np.array([b_width,0.,0.])

        #new frame
        self.target_T_o=target_T_o
        target_Rot_o=target_T_o[:3,:3]
        # target_quat_o=quaternion.from_rotation_matrix(target_Rot_o)
        # target_t_o=target_T_o[:3,3]

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
        self.target_move_down1=target_Rot_o@o_move_down1
        self.target_move_down1right0=target_Rot_o@o_move_down1right0
        self.target_move_down1left0=target_Rot_o@o_move_down1left0        
        self.target_move_up1=target_Rot_o@o_move_up1
        self.target_move_up1right0=target_Rot_o@o_move_up1right0
        self.target_move_up1left0=target_Rot_o@o_move_up1left0     

        #new target corners in new frame
        self.target_corner0=homogeneus_T_product(self.target_T_o,o_corner0)
        self.target_corner1=homogeneus_T_product(self.target_T_o,o_corner1)
        self.target_corner2=homogeneus_T_product(self.target_T_o,o_corner2)
        self.target_corner3=homogeneus_T_product(self.target_T_o,o_corner3)
    
    def corners_print_draw(self,img):
        numb_img=img.copy()
        i=0
        if self.startIdx!=-1:
            for corner in self.start_block.corners:
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

    def poseEstimate(self,width_offset,cam_mtx,cam_dist):
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
        if self.down1Idx!=-1 and self.down1_block.block_type=="front_face":
            ordered_corners=self.down1_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_down1))
        if self.down1left0Idx!=-1 and self.down1left0_block.block_type=="front_face":
            ordered_corners=self.down1left0_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_down1left0))
        if self.down1right0Idx!=-1 and self.down1right0_block.block_type=="front_face":
            ordered_corners=self.down1right0_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_down1right0))
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
        if self.up1right0Idx!=-1 and self.up1right0_block.block_type=="front_face":
            ordered_corners=self.up1right0_block.ordered_corners()
            if len(ordered_corners)>0:
                self.init_impt.append(ordered_corners)
                self.init_objp.append(ordered_objp(self.target_move_up1right0))                

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
        
        return rvecs,tvecs

    def top_to_bottom3D(self,orientation,tower_height=18):
        self.bott=np.array([0,self.b_height*tower_height,0])
        if orientation=="sx":
            self.bottSx=np.array([self.b_width*1.5,self.b_height*tower_height,self.b_width*0.5])
            self.bottCx=np.array([self.b_width*1.5,self.b_height*tower_height,self.b_width*1.5])
            self.bottDx=np.array([self.b_width*1.5,self.b_height*tower_height,self.b_width*2.5])
        elif orientation=="dx":
            self.bottSx=np.array([-self.b_width*1.5,self.b_height*tower_height,self.b_width*2.5])
            self.bottCx=np.array([-self.b_width*1.5,self.b_height*tower_height,self.b_width*1.5])
            self.bottDx=np.array([-self.b_width*1.5,self.b_height*tower_height,self.b_width*0.5])
    
    def project3D_draw(self,img,rvec,tvec,cam_mtx,cam_dist,width_offset=80):
        img_draw=img.copy()
        proj_bott,_=cv2.projectPoints(self.bott,rvec,tvec,cam_mtx,cam_dist)
        proj_bottSx,_=cv2.projectPoints(self.bottSx,rvec,tvec,cam_mtx,cam_dist)
        proj_bottCx,_=cv2.projectPoints(self.bottCx,rvec,tvec,cam_mtx,cam_dist)
        proj_bottDx,_=cv2.projectPoints(self.bottDx,rvec,tvec,cam_mtx,cam_dist)
        proj_bott=np.squeeze(proj_bott).astype(np.int)-np.array([width_offset,0],dtype=np.int)
        proj_bottSx=np.squeeze(proj_bottSx).astype(np.int)-np.array([width_offset,0],dtype=np.int)
        proj_bottCx=np.squeeze(proj_bottCx).astype(np.int)-np.array([width_offset,0],dtype=np.int)
        proj_bottDx=np.squeeze(proj_bottDx).astype(np.int)-np.array([width_offset,0],dtype=np.int)

        cv2.line(img_draw,self.start_block.centroid,proj_bott,[255,255,255])
        cv2.line(img_draw,self.start_block.centroid,proj_bottSx,[255,0,0])
        cv2.line(img_draw,self.start_block.centroid,proj_bottCx,[0,255,0])
        cv2.line(img_draw,self.start_block.centroid,proj_bottDx,[0,0,255])

        return img_draw
    
    def project3D_toBottom(self,orientation,bottom3_blocks,rvec,tvec,cam_mtx,cam_dist,width_offset=80):
        proj_bott,_=cv2.projectPoints(self.bott,rvec,tvec,cam_mtx,cam_dist)
        proj_bott=np.squeeze(proj_bott).astype(np.float)-np.array([width_offset,0],dtype=np.float)

        for block_test in bottom3_blocks:
            if orientation=="dx" and block_test.block_type!='front_face':
                proj_bottDx,_=cv2.projectPoints(self.bottDx,rvec,tvec,cam_mtx,cam_dist)
                proj_bottDx=np.squeeze(proj_bottDx).astype(np.float)-np.array([width_offset,0],dtype=np.float)
                if (cv2.pointPolygonTest(block_test.contour_max,proj_bott,True)>-20) or \
                    (cv2.pointPolygonTest(block_test.contour_max,proj_bottDx,True)>-20):
                    #found bottom layer
                    return True
            elif orientation=="sx" and block_test.block_type!='front_face':
                proj_bottSx,_=cv2.projectPoints(self.bottSx,rvec,tvec,cam_mtx,cam_dist)
                proj_bottSx=np.squeeze(proj_bottSx).astype(np.float)-np.array([width_offset,0],dtype=np.float)
                if (cv2.pointPolygonTest(block_test.contour_max,proj_bott,True)>-20) or \
                    (cv2.pointPolygonTest(block_test.contour_max,proj_bottSx,True)>-20):
                    #found bottom layer
                    return True
        return False
        
if __name__=='__main__':
    print("Must be used as a class")
