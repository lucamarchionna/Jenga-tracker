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
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>

int main(int argc, char *argv[])
{
	std::string opt_image_name = "";

  ros::init(argc, argv, "realsense_client");
  ros::NodeHandle node("~");
	// ros::Rate loop_rate(10);
  ros::ServiceClient client = node.serviceClient<tracker_visp::YolactInitializeCaoPose>("/YolactInitializeCaoPose");


  std::string model_cao = "";
  vpHomogeneousMatrix cMo;

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

	vpImage<vpRGBa> I_color(height, width);
	vpDisplayOpenCV d1;
  d1.init(I_color, 100, 50, "Color stream");

// 	bool terminated = false;
//   bool found_top = false;
//   vpMouseButton::vpMouseButtonType button;
//   while (!terminated && !found_top && node.ok()) {
//     try{
//       realsense.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, NULL, NULL);
//     }
//     catch (const rs2::error &e){
//         std::cout << "Catch an exception: " << e.what() << std::endl;
//         return;
//     }

//     vpImageConvert::createDepthHistogram(I_depth_raw, I_depth_gray);
//     vpImageConvert::convert(I_depth_gray,I_depth_color);
//     mapOfImages["Camera1"] = &I_color;
//     mapOfImages["Camera2"] = &I_depth_color;
//     vpDisplay::display(I_color);
//     vpDisplay::displayText(I_color, 20, 20, "Left click: start init. Righ:quit", vpColor::red);
//     vpDisplay::flush(I_color);
    
//     if (vpDisplay::getClick(I_color, button, false)) {
//       if(button == vpMouseButton::button1){
//         sensor_msgs::Image sensor_image = visp_bridge::toSensorMsgsImage(I_color);
//         sensor_msgs::CameraInfo sensor_camInfo = visp_bridge::toSensorMsgsCameraInfo(cam_color,width,height);
//         // Send image and cam par to service init
//         ROS_INFO("Subscribing to service...");
//         ros::service::waitForService("/FirstInitialization"); 
//         srv_init.request.image = sensor_image;
//         srv_init.request.camInfo = sensor_camInfo;
//         ROS_INFO("Starting call to service..");
//         if (client_init.call(srv_init))
//         {
//           ROS_INFO("Response from service..");
//           found_top=srv_init.response.found_top.data;

//           if (found_top){
//             cout << "Found top init" << endl; 
//           }
//           else {
//             ROS_INFO("Image discarded");
//           }
//         }
//         else{
//           ROS_ERROR("Cannot receive response from server");
//         }
//       }
//       else if (button == vpMouseButton::button3){
//         terminated=true;
//       }
//     }
//   }

	// [Acquire stream and initialize displays]
	bool terminated = false;
	vpMouseButton::vpMouseButtonType button;
	model_cao="";
  	while (!terminated && (model_cao=="") && node.ok()) {
		try{
			realsense.acquire(I_color,NULL);
		}
		catch (const rs2::error &e){
    		std::cout << "Catch an exception: " << e.what() << std::endl;
		    return EXIT_FAILURE;
		}

		vpDisplay::display(I_color);
		vpDisplay::displayText(I_color, 20, 20, "Click when ready.", vpColor::red);
		vpDisplay::flush(I_color);
		
		if (vpDisplay::getClick(I_color, button, false)) {
			if(button == vpMouseButton::button1){
				sensor_msgs::Image sensor_image = visp_bridge::toSensorMsgsImage(I_color);
				sensor_msgs::CameraInfo sensor_camInfo = visp_bridge::toSensorMsgsCameraInfo(cam,width,height);
				// Send image and cam par to service, where Python node responds with cao_file and pose
				ROS_INFO("Subscribing to service...");
				ros::service::waitForService("/Pose_cao_initializer",1000);
				tracker_visp::YolactInitializeCaoPose srv;
				srv.request.image = sensor_image;
				srv.request.camInfo = sensor_camInfo;
				ROS_INFO("Starting call to service..");
				if (client.call(srv))
				{
					ROS_INFO("Response from service..");
					model_cao=srv.response.caoFilePath.data;
					if (model_cao!=""){
						geometry_msgs::Pose initPose=srv.response.initPose.pose.pose;
						cMo=visp_bridge::toVispHomogeneousMatrix(initPose);	//object pose cMo
					}
					else {
						ROS_INFO("Image discarded");
					}
				}
				else{
					ROS_ERROR("Cannot receive response from server");
				}
			}
			else if (button == vpMouseButton::button3){
				terminated=true;
			}
		}
	}

	// Keep ros node alive
	// ros::spinOnce();
	// loop_rate.sleep();

  return EXIT_SUCCESS;
}