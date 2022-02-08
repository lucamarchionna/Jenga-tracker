#!/usr/bin/env python3

import numpy as np
import cv2

class Block():
    def __init__(self,classId,score,box,mask,idx):
        self.classId=classId
        self.score=score
        self.box=box
        self.mask=mask
        self.idx=idx

    def add_masked(self,img,maskcv2):
        self.img=img.copy()
        self.maskcv2=maskcv2
        self.masked=cv2.bitwise_and(self.img,self.img,mask=self.maskcv2)
    
    def find_contour(self,retr_list=cv2.RETR_LIST,chain_approx=cv2.CHAIN_APPROX_NONE):
        try:
            self.contours_all, _ = cv2.findContours(self.maskcv2,retr_list,chain_approx)
        except ValueError:
            _, self.contours_all, _ = cv2.findContours(self.maskcv2,retr_list,chain_approx)
        self.contour_max=max(self.contours_all,key=cv2.contourArea)
        self.area=cv2.contourArea(self.contour_max)
        moments = cv2.moments(self.contour_max)
        self.centroid=(int(moments['m10']/moments['m00']),int(moments['m01']/moments['m00']))
        self.mask_single=np.zeros(self.maskcv2.shape,dtype=np.uint8)
        #cv2.fillConvexPoly(self.mask_single,self.contour_max,255)
        cv2.drawContours(self.mask_single,[self.contour_max],-1,255,thickness=-1)
    
    def draw_contoursAll_centroid(self,img):
        temp=img.copy()
        cv2.drawContours(temp,self.contours_all,-1,[0,0,255],thickness=2)
        cv2.circle(temp,self.centroid,3,[0,255,0],thickness=-1)
        return temp

    def draw_contourMax_centroid(self,img):
        temp=img.copy()
        cv2.drawContours(temp,self.contour_max,-1,[0,0,255],thickness=2)
        cv2.circle(temp,self.centroid,3,[0,255,0],thickness=-1)
        return temp

    def draw_masked_approx(self,img):
        temp=img.copy()
        masked_single=cv2.bitwise_and(temp,temp,mask=self.mask_approx_single)
        return masked_single

    def draw_masked_single(self,img):
        temp=img.copy()
        masked_single=cv2.bitwise_and(temp,temp,mask=self.mask_single)
        return masked_single
    
    def draw_corners(self,img):
        temp=img.copy()
        if self.block_type=='front_face':
            for corner in self.corners:
                cv2.circle(temp,(corner[0],corner[1]),5,[0,0,255])
        else: #more than four corners or wrong shape overall
            for corner in self.corners:
                cv2.circle(temp,(corner[0],corner[1]),5,[255,0,0]) 
        return temp

    def differentiate_approximate(self,epsi_percent_front,avgArea,epsi_percent_side=0.001,scale_avgArea=1):
        self.epsi_front=epsi_percent_front*cv2.arcLength(self.contour_max,True)
        self.epsi_side=epsi_percent_side*cv2.arcLength(self.contour_max,True)
        self.mask_approx_single=np.zeros(self.maskcv2.shape,dtype=np.uint8)
        if (len(cv2.approxPolyDP(self.contour_max,self.epsi_front,True))==4 and cv2.contourArea(self.contour_max)<scale_avgArea*avgArea):
            self.block_type='front_face'
            self.contour_approx = cv2.approxPolyDP(self.contour_max,self.epsi_front,True) #approximate contour to few lesser precision
            cv2.fillConvexPoly(self.mask_approx_single,self.contour_approx,255)
        else:
            self.block_type='side_face'
            #self.contour_approx=self.contour_max #no approximations for now
            self.contour_approx = cv2.approxPolyDP(self.contour_max,self.epsi_side,True) #approximate contour to way lesser precision
            #cv2.fillPoly(self.mask_approx_single,self.contour_approx,255) #FILL CONVEX PLOY HAS A UGLY EFFECT ON SIDE BLOCKS
            cv2.drawContours(self.mask_approx_single,[self.contour_approx],-1,255,thickness=-1)
    
    def find_corners(self,subPix_size,subPix_eps=0.001,subPix_iters=100,harris_param=[2,3,0.04],harris_threshold=0.1):
        #Refine corners subpixel, from harris centroids (depends on harris quality) or from polynomial approximation
        # This is done on original mask, not approximated, to keep full info
        subPix_criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, subPix_iters, subPix_eps)
        if self.block_type=='front_face':
            self.corners=np.int0(cv2.cornerSubPix(self.mask_single,np.float32(self.contour_approx[:,0]),(subPix_size,subPix_size),(-1,-1),subPix_criteria))
        else:
            harris_corners=cv2.cornerHarris(self.mask_approx_single,harris_param[0],harris_param[1],harris_param[2])
            _, harris_corners = cv2.threshold(harris_corners,harris_threshold*harris_corners.max(),255,0) #0.1 to keep (stronger) corners, decrease to increase nb of corners
            harris_corners=harris_corners.astype(np.uint8)
            _, _, _, centroids = cv2.connectedComponentsWithStats(harris_corners)
            centroids=centroids[1:] #REMOVE THE INFO ABOUT BACKGROUND
            self.corners=np.int0(cv2.cornerSubPix(self.mask_single,np.float32(centroids),(subPix_size,subPix_size),(-1,-1),subPix_criteria))
    
    def ordered_corners(self,min_slope_h=2):
        '''Require find_corners first'''
        if self.block_type=='front_face':
            xc=self.corners[:,0]
            yc=self.corners[:,1]

            slope1=slope2=slope3=slope4=1000.0
            if (xc[1]-xc[0])!=0:
                slope1 = (yc[1]-yc[0])/(xc[1]-xc[0])
            if (xc[2]-xc[1])!=0:
                slope2 = (yc[2]-yc[1])/(xc[2]-xc[1])
            if (xc[3]-xc[2])!=0:
                slope3 = (yc[3]-yc[2])/(xc[3]-xc[2])
            if (xc[0]-xc[3])!=0:
                slope4 = (yc[0]-yc[3])/(xc[0]-xc[3])

            #corners are counterclockwise, starting from higher (smaller y)
            if abs(slope1)<min_slope_h and abs(slope3)<min_slope_h: #corners[0] is up right
                ordered_corners=[self.corners[2],self.corners[1],self.corners[0],self.corners[3]]
            else: # #corners[0] is up left, same as previous but points become 0->1, 1->2, 2->3, 3->0
                ordered_corners=[self.corners[1],self.corners[0],self.corners[3],self.corners[2]]
            return ordered_corners
        else:
            return []

    def compute_slopes(self,min_slope_h=2):
        '''Require find_corners first'''
        self.versor_H=(0,0)
        self.versor_V=(0,0)
        self.avgLength_H=0.0
        self.avgLength_V=0.0
        if self.block_type=='front_face':
            xc=self.corners[:,0]
            yc=self.corners[:,1]

            slope1=slope2=slope3=slope4=1000.0
            if (xc[1]-xc[0])!=0:
                slope1 = (yc[1]-yc[0])/(xc[1]-xc[0])
            if (xc[2]-xc[1])!=0:
                slope2 = (yc[2]-yc[1])/(xc[2]-xc[1])
            if (xc[3]-xc[2])!=0:
                slope3 = (yc[3]-yc[2])/(xc[3]-xc[2])
            if (xc[0]-xc[3])!=0:
                slope4 = (yc[0]-yc[3])/(xc[0]-xc[3])

            #corners are counterclockwise, starting from higher (smaller y)
            if abs(slope1)<min_slope_h and abs(slope3)<min_slope_h: #corners[0] is up right
                avgPoint1_H=[(xc[1]+xc[2])/2.0,(yc[1]+yc[2])/2.0]
                avgPoint2_H=[(xc[0]+xc[3])/2.0,(yc[0]+yc[3])/2.0]
                vx_H, vy_H, _, _ = cv2.fitLine(np.vstack((avgPoint1_H,avgPoint2_H)), cv2.DIST_L2, 0, 0.01, 0.01)
                #self.avgSlope_H=vy_H/vx_H
                self.versor_H=(vx_H[0],vy_H[0])
                avgPoint1_V=[(xc[2]+xc[3])/2.0,(yc[2]+yc[3])/2.0]
                avgPoint2_V=[(xc[0]+xc[1])/2.0,(yc[0]+yc[1])/2.0]
                vx_V, vy_V, _, _ = cv2.fitLine(np.vstack((avgPoint1_V,avgPoint2_V)), cv2.DIST_L2, 0, 0.01, 0.01)
                #self.avgSlope_V=vy_V/vx_V
                self.versor_V=(vx_V[0],vy_V[0])
                self.avgLength_H=(np.linalg.norm(self.corners[0]-self.corners[1])+np.linalg.norm(self.corners[2]-self.corners[3]))/2.0
                self.avgLength_V=(np.linalg.norm(self.corners[1]-self.corners[2])+np.linalg.norm(self.corners[3]-self.corners[0]))/2.0
            else:   # #corners[0] is up left, same as previous but points become 0->1, 1->2, 2->3, 3->0
                avgPoint1_H=[(xc[0]+xc[1])/2.0,(yc[0]+yc[1])/2.0]
                avgPoint2_H=[(xc[2]+xc[3])/2.0,(yc[2]+yc[3])/2.0]
                vx_H, vy_H, _, _ = cv2.fitLine(np.vstack((avgPoint1_H,avgPoint2_H)), cv2.DIST_L2, 0, 0.01, 0.01)
                #self.avgSlope_H=vy_H/vx_H
                self.versor_H=(vx_H[0],vy_H[0])
                avgPoint1_V=[(xc[1]+xc[2])/2.0,(yc[1]+yc[2])/2.0]
                avgPoint2_V=[(xc[0]+xc[3])/2.0,(yc[0]+yc[3])/2.0]
                vx_V, vy_V, _, _ = cv2.fitLine(np.vstack((avgPoint1_V,avgPoint2_V)), cv2.DIST_L2, 0, 0.01, 0.01)
                #self.avgSlope_V=vy_V/vx_V
                self.versor_V=(vx_V[0],vy_V[0])
                self.avgLength_H=(np.linalg.norm(self.corners[1]-self.corners[2])+np.linalg.norm(self.corners[3]-self.corners[0]))/2.0
                self.avgLength_V=(np.linalg.norm(self.corners[0]-self.corners[1])+np.linalg.norm(self.corners[2]-self.corners[3]))/2.0
        return self.versor_H, self.versor_V,self.avgLength_H,self.avgLength_V
        
    def draw_test_points(self,img,min_slope_h=2):
        '''Requires slopes and compute_test_points first'''
        #self.compute_slopes(min_slope_h)
        #self.compute_test_points(min_slope_h)
        ######################
        temp=img.copy()
        if self.block_type=='front_face':
            cv2.circle(temp,self.centroid,5,[255,255,0],thickness=-1)
            cv2.line(temp,self.centroid,self.test_up,[255,255,255],thickness=2)
            cv2.line(temp,self.centroid,self.test_down,[255,0,0],thickness=2)
            cv2.line(temp,self.centroid,self.test_up1,[255,255,255],thickness=1)
            cv2.line(temp,self.centroid,self.test_up1left0,[255,255,0],thickness=1)
            cv2.line(temp,self.centroid,self.test_up1right0,[255,0,255],thickness=1)
            cv2.line(temp,self.centroid,self.test_up1left1,[180,180,180],thickness=1)
            cv2.line(temp,self.centroid,self.test_up1right1,[180,0,180],thickness=1)
            cv2.line(temp,self.centroid,self.test_right0,[0,0,255],thickness=2)
            cv2.line(temp,self.centroid,self.test_left0,[0,255,0],thickness=2)
        return temp
    
    def draw_layer_test_points(self,img,min_slope_h=2):
        '''Requires slopes and compute_test_points first'''
        #self.compute_slopes(min_slope_h)
        #self.compute_test_points(min_slope_h)
        ######################
        temp=img.copy()
        if self.block_type=='front_face':
            cv2.circle(temp,self.centroid,5,[255,255,0],thickness=-1)
            cv2.line(temp,self.centroid,self.test_right0,[0,0,255],thickness=2)
            cv2.line(temp,self.centroid,self.test_left0,[0,255,0],thickness=2)
        return temp
    
    def draw_additional_test_point(self,img,test_point,centroid,cv2color=[255,255,255]):
        temp=img.copy()
        if self.block_type=='front_face' and centroid!=(0,0):
            cv2.line(temp,centroid,test_point,cv2color,thickness=1)
        return temp

    def compute_test_points(self,min_slope_h=2):
        '''Requires compute_slopes first'''
        #self.compute_slopes(min_slope_h)
        #####################
        self.test_up=(0,0)
        self.test_down=(0,0)
        self.test_right0=(0,0)
        self.test_right1=(0,0)
        self.test_left0=(0,0)
        self.test_left1=(0,0)
        self.test_upleft=(0,0)
        self.test_downleft=(0,0)
        self.test_upright1=(0,0)
        self.test_downright1=(0,0)
        self.test_up1left0=(0,0)
        self.test_up1left1=(0,0)
        self.test_up1right0=(0,0)        
        self.test_up1right1=(0,0)        
        if self.block_type=='front_face':
            if self.versor_V[1]>0:  #versor going down
                self.test_up=(int(self.centroid[0]-self.avgLength_V*self.versor_V[0]),int(self.centroid[1]-self.avgLength_V*self.versor_V[1]))
                self.test_down=(int(self.centroid[0]+self.avgLength_V*self.versor_V[0]),int(self.centroid[1]+self.avgLength_V*self.versor_V[1]))
                self.test_up1=(int(self.centroid[0]-2*self.avgLength_V*self.versor_V[0]),int(self.centroid[1]-2*self.avgLength_V*self.versor_V[1]))
            else: 
                self.test_up=(int(self.centroid[0]+self.avgLength_V*self.versor_V[0]),int(self.centroid[1]+self.avgLength_V*self.versor_V[1]))
                self.test_down=(int(self.centroid[0]-self.avgLength_V*self.versor_V[0]),int(self.centroid[1]-self.avgLength_V*self.versor_V[1]))
                self.test_up1=(int(self.centroid[0]+2*self.avgLength_V*self.versor_V[0]),int(self.centroid[1]+2*self.avgLength_V*self.versor_V[1]))
            self.test_right0=(int(self.centroid[0]+self.avgLength_H*self.versor_H[0]),int(self.centroid[1]+self.avgLength_H*self.versor_H[1]))
            self.test_right1=(int(self.centroid[0]+2*self.avgLength_H*self.versor_H[0]),int(self.centroid[1]+2*self.avgLength_H*self.versor_H[1]))
            self.test_left0=(int(self.centroid[0]-self.avgLength_H*self.versor_H[0]),int(self.centroid[1]-self.avgLength_H*self.versor_H[1]))
            self.test_left1=(int(self.centroid[0]-2*self.avgLength_H*self.versor_H[0]),int(self.centroid[1]-2*self.avgLength_H*self.versor_H[1]))
            self.test_upleft0=(int(self.test_up[0]-self.avgLength_H*self.versor_H[0]),int(self.test_up[1]-self.avgLength_H*self.versor_H[1]))
            self.test_downleft0=(int(self.test_down[0]-self.avgLength_H*self.versor_H[0]),int(self.test_down[1]-self.avgLength_H*self.versor_H[1]))
            self.test_upright1=(int(self.test_up[0]+2*self.avgLength_H*self.versor_H[0]),int(self.test_up[1]+2*self.avgLength_H*self.versor_H[1]))
            self.test_downright1=(int(self.test_down[0]+2*self.avgLength_H*self.versor_H[0]),int(self.test_down[1]+2*self.avgLength_H*self.versor_H[1]))
            self.test_upright0=(int(self.test_up[0]+self.avgLength_H*self.versor_H[0]),int(self.test_up[1]+self.avgLength_H*self.versor_H[1]))
            self.test_downright0=(int(self.test_down[0]+self.avgLength_H*self.versor_H[0]),int(self.test_down[1]+self.avgLength_H*self.versor_H[1]))
            self.test_upleft1=(int(self.test_up[0]-2*self.avgLength_H*self.versor_H[0]),int(self.test_up[1]-2*self.avgLength_H*self.versor_H[1]))
            self.test_downleft1=(int(self.test_down[0]-2*self.avgLength_H*self.versor_H[0]),int(self.test_down[1]-2*self.avgLength_H*self.versor_H[1]))
            self.test_up1left0=(int(self.test_up1[0]-self.avgLength_H*self.versor_H[0]),int(self.test_up1[1]-self.avgLength_H*self.versor_H[1]))
            self.test_up1right0=(int(self.test_up1[0]+self.avgLength_H*self.versor_H[0]),int(self.test_up1[1]+self.avgLength_H*self.versor_H[1]))
            self.test_up1left1=(int(self.test_up1[0]-2*self.avgLength_H*self.versor_H[0]),int(self.test_up1[1]-2*self.avgLength_H*self.versor_H[1]))
            self.test_up1right1=(int(self.test_up1[0]+2*self.avgLength_H*self.versor_H[0]),int(self.test_up1[1]+2*self.avgLength_H*self.versor_H[1]))
        return self.test_up,self.test_down,self.test_right0,self.test_right1,self.test_left0,self.test_left1,\
            self.test_upleft0,self.test_downleft0,self.test_upright1,self.test_downright1,self.test_upright0,\
            self.test_downright0,self.test_upleft1,self.test_downleft1,\
            self.test_up1,self.test_up1left0,self.test_up1right0,self.test_up1left1,self.test_up1right1

    def compute_down_test_points(self,min_slope_h=2):
        '''Requires compute_slopes first'''
        #self.compute_slopes(min_slope_h)
        #####################
        self.test_down=(0,0)
        self.test_right0=(0,0)
        self.test_right1=(0,0)
        self.test_left0=(0,0)
        self.test_left1=(0,0)
        self.test_down1left0=(0,0)        
        self.test_down1right0=(0,0)              
        if self.block_type=='front_face':
            if self.versor_V[1]>0:  #versor going down
                self.test_up=(int(self.centroid[0]-self.avgLength_V*self.versor_V[0]),int(self.centroid[1]-self.avgLength_V*self.versor_V[1]))
                self.test_down=(int(self.centroid[0]+self.avgLength_V*self.versor_V[0]),int(self.centroid[1]+self.avgLength_V*self.versor_V[1]))
                self.test_down1=(int(self.centroid[0]+2*self.avgLength_V*self.versor_V[0]),int(self.centroid[1]+2*self.avgLength_V*self.versor_V[1]))
            else: 
                self.test_up=(int(self.centroid[0]+self.avgLength_V*self.versor_V[0]),int(self.centroid[1]+self.avgLength_V*self.versor_V[1]))
                self.test_down=(int(self.centroid[0]-self.avgLength_V*self.versor_V[0]),int(self.centroid[1]-self.avgLength_V*self.versor_V[1]))
                self.test_down1=(int(self.centroid[0]-2*self.avgLength_V*self.versor_V[0]),int(self.centroid[1]-2*self.avgLength_V*self.versor_V[1]))
            self.test_right0=(int(self.centroid[0]+self.avgLength_H*self.versor_H[0]),int(self.centroid[1]+self.avgLength_H*self.versor_H[1]))
            self.test_right1=(int(self.centroid[0]+2*self.avgLength_H*self.versor_H[0]),int(self.centroid[1]+2*self.avgLength_H*self.versor_H[1]))
            self.test_left0=(int(self.centroid[0]-self.avgLength_H*self.versor_H[0]),int(self.centroid[1]-self.avgLength_H*self.versor_H[1]))
            self.test_left1=(int(self.centroid[0]-2*self.avgLength_H*self.versor_H[0]),int(self.centroid[1]-2*self.avgLength_H*self.versor_H[1]))
            self.test_down1left0=(int(self.test_down1[0]-self.avgLength_H*self.versor_H[0]),int(self.test_down1[1]-self.avgLength_H*self.versor_H[1]))
            self.test_down1right0=(int(self.test_down1[0]+self.avgLength_H*self.versor_H[0]),int(self.test_down1[1]+self.avgLength_H*self.versor_H[1]))
        return self.test_down,self.test_right0,self.test_right1,self.test_left0,self.test_left1,\
            self.test_down1,self.test_down1left0,self.test_down1right0

    def get_centroid_height(self):
        return self.centroid[1]


if __name__=='__main__':
    print('Must be used as class only')
