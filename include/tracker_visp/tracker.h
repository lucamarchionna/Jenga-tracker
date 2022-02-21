#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tracker_visp/angle_velocity.h>

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/vision/vpKeyPoint.h>

// TEMP
#include <visp3/io/vpVideoWriter.h>
#include <visp3/core/vpImageDraw.h>
#include <visp3/core/vpFont.h>
// TEMP
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "tf2_msgs/TFMessage.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include <boost/bind.hpp>
#include "std_msgs/String.h"
#include <std_msgs/UInt16.h>
#include "tracker_visp/YolactInitializeCaoPose.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <ros/package.h>
#include <boost/filesystem.hpp>

using namespace std;

class tracking 
{
    public:
        tracking(ros::NodeHandle& nh);
        //~tracking(); //destroy when class ends
        vpHomogeneousMatrix homogeneousTransformation(string link1, string link2);
        geometry_msgs::TransformStamped toMoveit(vpHomogeneousMatrix data, string header, string child_frame);

        void init_parameters();
        void learning_process();
        void detection_process();
        void learningCallback(const std_msgs::Bool::ConstPtr& msg);
        //void estimationCallback(const geometry_msgs::Pose& tracker_pose_P);
        //void learningCallback(const std_msgs::Bool::ConstPtr& msg);

        
    private:
        ros::NodeHandle node_handle;
        //ros::Rate loop_rate;

        ros::Subscriber subLearning; 
        ros::Publisher trackerEstimation;
        ros::Publisher servoPub;
        ros::ServiceClient client;
        string tracker_path, opt_config, opt_model, opt_init, opt_init_pos, opt_learning_data, opt_keypoint_config; 

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tf2_ros::Buffer tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        tf::StampedTransform transform;
        ros::Time t;
        rs2::config config;
        vpDisplayOpenCV d1, d2, d3, d4;
        tracker_visp::YolactInitializeCaoPose srv;

        static const string PLANNING_GROUP; 
        moveit::planning_interface::MoveGroupInterface move_group_interface(string PLANNING_GROUP);
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit::planning_interface::MoveGroupInterface move_group{"edo"};

        vpHomogeneousMatrix cMo, cTo, eeTc, eeTc1, wTee, depth_M_color, wTc, wTc1, bTee;

        double opt_learn, opt_auto_init, opt_proj_error_threshold{25.0}, opt_setGoodME_thresh{0.4};
        int opt_disp_visibility{0}, width{640}, height{480}, fps{30};
        bool opt_display_projection_error{false}, opt_display_features{true}, opt_display_model{true}, opt_yolact_init{true}, opt_pose_init{true}, learn_position{true}, rotated{false};
        vpRotationMatrix cdRo;
        vpKeyPoint keypoint;
        vpMbGenericTracker *tracker;
        std::vector<int> trackerTypes;
        vpVideoWriter writer;


        vpRealSense2 realsense;
        vpImage<vpRGBa> I_color, ILearned, I_depth_color; 
        vpImage<unsigned char> I_gray;
        vpImage<unsigned char> I_depth_gray;
        vpImage<unsigned char> IMatching;
        vpImage<uint16_t> I_depth_raw;

        vpCameraParameters cam_color, cam_depth;

        unsigned int _posx{100}, _posy{50};

        
        ros::Publisher pub;
        
};
