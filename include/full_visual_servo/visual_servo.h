#include <iostream>
#include <ros/ros.h>

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/vision/vpKeyPoint.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/vs/vpServo.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

// TEMP
#include <visp3/io/vpVideoWriter.h>
#include <visp3/core/vpImageDraw.h>
#include <visp3/core/vpFont.h>
// TEMP

#include "tf2_msgs/TFMessage.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include <boost/bind.hpp>

#include "tracker_visp/YolactInitializeCaoPose.h"
#include "tracker_visp/ForceBasedDecision.h"
#include "tracker_visp/location.h"
#include "tracker_visp/ReferenceBlock.h"

#include <tracker_visp/angle_velocity.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <ros/package.h>
#include <boost/filesystem.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <edo_core_msgs/JointControlArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>


using namespace std;

class visual_servoing 
{
    public:
        visual_servoing(ros::NodeHandle& nh);
        //~visual_servoing(); //destroy when class ends
        // ~visual_servoing(); //destroy when class ends
        int init_matrices();
        void init_servo();
        void stopping_criteria();
        void update(const ros::TimerEvent& e);
        void estimationCallback(const geometry_msgs::Pose& tracker_pose_P);
        void init_parameters();
        void learning_process();
        void reinit_vs();
        void detection_process();
        void learningCallback(const std_msgs::Bool::ConstPtr& msg);
        void forceCallback(const std_msgs::BoolConstPtr& retract_call);
        double toBlocktransl(tracker_visp::location block);
        

        vpHomogeneousMatrix homogeneousTransformation(string link1, string link2);
        geometry_msgs::TransformStamped toMoveit(vpHomogeneousMatrix data, string header, string child_frame);
        geometry_msgs::PoseStamped insertPosData(vpHomogeneousMatrix data);
        geometry_msgs::TwistStamped insertData(vpColVector data);
        geometry_msgs::TwistStamped nullVelocity();
        vpAdaptiveGain lambda(double gain_at_zero, double gain_at_infinity, double slope_at_zero);
        
   

        
    private:
        ros::NodeHandle node_handle;
        //ros::Rate loop_rate;

        ros::Subscriber subEstimationPose, subForce; 
        ros::Publisher velocityInput;
        ros::Publisher startingPos;
        
        ros::Subscriber subLearning; 
        ros::Publisher trackerEstimation;
        ros::Publisher servoPub;
        ros::ServiceClient client, client_force;
        string tracker_path, opt_config, opt_model, opt_init, opt_init_pos, opt_learning_data, opt_keypoint_config; 

        static const string PLANNING_GROUP; 
        moveit::planning_interface::MoveGroupInterface move_group_interface(string PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tf2_ros::Buffer tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform transform;
        ros::Time t;
        rs2::config config;
        vpDisplayOpenCV d1, d2, d3, d4;
        tracker_visp::YolactInitializeCaoPose srv;


        vpHomogeneousMatrix cMo, cTo, eeTc, eeTc1, wTee, depth_M_color, wTc, wTc1, eeTcam, baseTee, eeTtarget, baseTtarget, targetTcam, cam_desTtarget, camTtarget, cdTc, offset, cdTtarget, camTee, camTbase, bTee;

        geometry_msgs::PoseStamped initialGuestPos;
        geometry_msgs::Pose lastPoseReceived;
        vpTranslationVector t_off;
        vpRxyzVector rxyz(vpRotationMatrix);
        vpRotationMatrix R_off;
        vpTranslationVector trans_vec;
        double translX, translX_policy{0};
        double translY;
        double translZ;

        tracker_visp::ForceBasedDecision srv_force;
        tracker_visp::location block_choice, new_block;

        vpServo task;
        vpFeatureTranslation s;
        vpFeatureTranslation s_star;
        vpFeatureThetaU s_tu;
        vpFeatureThetaU s_tu_star;
        bool add_features_once{true};
        vpPoint point, point_des2;
        vpFeaturePoint ps, point_des;
        vpFeaturePoint p[4], pd[4];
        vector<vpPoint> points;
        vpTranslationVector t_coord, t_des;
        vpFeaturePoint s_x, s_x_star;
        
        vpImage<unsigned char> Iint;
        vpImage<unsigned char> Iext;
        
        moveit::planning_interface::MoveGroupInterface move_group{"edo"};

        geometry_msgs::TwistStamped velocityData;
        double error;
        //positionbased_vs::InitialGuess service;
        double threshold, threshold_pose{0.10};
        double vitesse;
        double rapport;
        bool block_axis{false}, take_cTo{true}, retract{false}, go_to_service{true};
        float signPoseReceived{1.0};
        vpColVector v_ee(unsigned int n), omega_ee(unsigned int n), v_cam, v(unsigned int n), e1, proj_e1;


        edo_core_msgs::JointControlArray jnt_ctrl;
        //const vpException &e;
        
        
        ros::Publisher pub, lastPose;
        double opt_learn, opt_auto_init, opt_proj_error_threshold{25.0}, opt_setGoodME_thresh{0.4};
        int opt_disp_visibility{0}, width{640}, height{480}, fps{30};
        bool opt_display_projection_error{false}, opt_display_features{false}, opt_display_model{true}, opt_yolact_init{true}, opt_pose_init{true}, learn_position{true}, rotated{false}, f_max{false}, run_completed{false};
        vpRotationMatrix cdRo;
        vpKeyPoint keypoint;
        vpMbGenericTracker *tracker;
        vector<int> trackerTypes;
        vpVideoWriter writer;


        vpRealSense2 realsense;
        vpImage<vpRGBa> I_color, ILearned, I_depth_color; 
        vpImage<unsigned char> I_gray;
        vpImage<unsigned char> I_depth_gray;
        vpImage<unsigned char> IMatching;
        vpImage<uint16_t> I_depth_raw;

        vpCameraParameters cam_color, cam_depth;

        unsigned int _posx{100}, _posy{50};

                
};
