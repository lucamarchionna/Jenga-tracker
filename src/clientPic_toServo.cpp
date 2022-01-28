#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpXmlParserCamera.h>

#include <ros/ros.h>
#include "tracker_visp/YolactInitializeCaoPose.h"
#include <geometry_msgs/Transform.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
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
  ros::Publisher servoPub = node.advertise<std_msgs::UInt16>("/servo",1);

	node.param<std::string>("image_name",opt_image_name,"../model/prova_client.png");

	vpXmlParserCamera cameraParser;
	vpCameraParameters cam;
	cameraParser.parse(cam,"../trackers/camRS_vispCalib.xml","Realsense",vpCameraParameters::perspectiveProjWithoutDistortion);
	int width = 640, height = 480;

	vpImage<vpRGBa> clientvpImage;
	try{
		vpImageIo::read(clientvpImage,opt_image_name);
	} catch (const vpException &e){
		std::cout << e.getMessage() << std::endl;
	}

	std::string model_cao = "";
	vpHomogeneousMatrix cMo;

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
		geometry_msgs::Transform initPose=srv.response.initPose;
		cMo=visp_bridge::toVispHomogeneousMatrix(initPose);	//object pose cMo
		
		// SERVO SEND ANGLE
		double angle = cMo.getThetaUVector().getTheta();	//from object pose cMo
		std_msgs::UInt16 angle_to_servo;
		// std::cout << "Theta: \n" << angle << std::endl;
		if (angle<1){	//radians, "right face seen from camera"
			angle_to_servo.data = 90;	//degrees, final angle
			servoPub.publish(angle_to_servo);
		}
		else if (angle>=1){	//radians, "left face seen from camera"
			angle_to_servo.data = 0;	//degrees, final angle
			servoPub.publish(angle_to_servo);
		}
	}
	else{
		ROS_ERROR("Cannot receive response from server");
		return EXIT_FAILURE;
	}

	// Keep ros node alive
	ros::spinOnce();
	loop_rate.sleep();

  return EXIT_SUCCESS;
}