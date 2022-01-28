#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#include <ros/ros.h>
#include "tracker_visp/YolactInitializeCaoPose.h"
#include <geometry_msgs/Transform.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

int main(int argc, char *argv[])
{
	std::string opt_image_name = "";

  ros::init(argc, argv, "cpp_client");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(1); 
  spinner.start();
  ros::Rate loop_rate(10);
  ros::ServiceClient client = node.serviceClient<tracker_visp::YolactInitializeCaoPose>("/Pose_cao_initializer");
	// ros::Publisher pub = node.advertise<sensor_msgs::Image>("/client_image",1);
	// ros::Publisher pub2 = node.advertise<sensor_msgs::CameraInfo>("/client_camInfo",1);
	// ros::Publisher pub3 = node.advertise<geometry_msgs::Transform>("/client_pose",1);

	node.param<std::string>("image_name",opt_image_name,"../model/prova_client.png");

	std::string model_cao = "";
	std::string tracker_config = "../trackers/jenga_tracker_params.xml";
	vpHomogeneousMatrix cMo;
	vpMbGenericTracker tracker(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
	tracker.loadConfigFile(tracker_config);
	
  // [Realsense camera configuration]
  vpRealSense2 realsense;
  int width = 640, height = 480;
  int fps = 30;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);

  try {
    realsense.open(config);
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
    std::cout << "Check if the Realsense camera is connected..." << std::endl;
    return EXIT_SUCCESS;
  }
  catch (const rs2::error &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
    std::cout << "Check if the Realsense camera is connected..." << std::endl;
    return EXIT_SUCCESS;
  }

  vpCameraParameters cam = realsense.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
	tracker.setCameraParameters(cam);

	vpImage<vpRGBa> I_color(height, width);
	vpDisplayOpenCV d1;
	d1.init(I_color, 100, 50, "Color stream");

	// [Acquire stream and initialize displays]
	bool terminated = false;
/*
  while (!terminated) {
		realsense.acquire(I_color,NULL);

    vpDisplay::display(I_color);
    vpDisplay::displayText(I_color, 20, 20, "Click when ready.", vpColor::red);
    vpDisplay::flush(I_color);

    if (vpDisplay::getClick(I_color, false)){
			sensor_msgs::Image sensor_image = visp_bridge::toSensorMsgsImage(clientvpImage);
			sensor_msgs::CameraInfo sensor_camInfo = visp_bridge::toSensorMsgsCameraInfo(cam,width,height);
			// Send image and cam par to service, where Python node responds with cao_file and pose
			ROS_INFO("Subscribing to service...");
			ros::service::waitForService("/Pose_cao_initializer");
			tracker_visp::YolactInitializeCaoPose srv;
			srv.request.image = sensor_image;
			srv.request.camInfo = sensor_camInfo;
			ROS_INFO("Starting call to service..");
			if (client.call(srv))
			{
				ROS_INFO("Response from service..");
				model_cao=srv.response.caoFilePath.data;
				// Print model name
				std::cout << model_cao << std::endl;
				geometry_msgs::Transform initPose=srv.response.initPose;
				cMo=visp_bridge::toVispHomogeneousMatrix(initPose);
				// Print pose
				// std::cout << cMo << std::endl;
				std::cout << initPose.translation << std::endl << initPose.rotation << std::endl;
				tracker.loadModel(model_cao);
				ROS_INFO("Model loaded");
				try{
					tracker.initFromPose(I_color,cMo);
					ROS_INFO("Initialized with pose");
				} catch (const vpException &e) {
					std::cout << "Tracker exception: " << e.getStringMessage() << std::endl;
				}
			}
			else{
				ROS_ERROR("Cannot receive response from server");
				return EXIT_FAILURE;
			}
      terminated=true;
		}
	
	try{
		tracker.track(I_color);
	} catch (const vpException &e) {
			std::cout << "Tracker exception: " << e.getStringMessage() << std::endl;
	}

	// Keep ros node alive
	ros::spinOnce();
	loop_rate.sleep();
*/
  return EXIT_SUCCESS;

}