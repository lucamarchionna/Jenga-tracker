#include <iostream>
#include <visp3/core/vpConfig.h>
// #include <visp3/core/vpDisplay.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpHomogeneousMatrix.h>
// #include <visp3/gui/vpDisplayOpenCV.h>
// #include <visp3/sensor/vpRealSense2.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#include <ros/ros.h>
#include "tracker_visp/YolactInitializeCaoPose.h"
// #include "tracker_visp/PoseEstimation.h"
#include "tracker_visp/ReferenceBlock.h"
#include <geometry_msgs/Pose.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include <ros/package.h>

int main(int argc, char *argv[])
{
	std::string opt_image_name = "";
	std::string tracker_path = ros::package::getPath("tracker_visp");

  	ros::init(argc, argv, "cpp_sendPic");
  	ros::NodeHandle node("~");
	ros::Rate loop_rate(10);
  	// ros::ServiceClient client = node.serviceClient<tracker_visp::PoseEstimation>("/PoseEstimation");
	ros::ServiceClient client = node.serviceClient<tracker_visp::YolactInitializeCaoPose>("/YolactInitializeCaoPose");

	node.param<std::string>("image_name",opt_image_name,"prova_client.png");

	vpXmlParserCamera cameraParser;
	vpCameraParameters cam;
	cameraParser.parse(cam,tracker_path+"/trackers/camRS_vispCalib.xml","Realsense",vpCameraParameters::perspectiveProjWithoutDistortion);
	int width = 640, height = 480;

	opt_image_name = tracker_path + "/model/" + opt_image_name;
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
	// tracker_visp::ReferenceBlock cTo_est;
	// vpPoseVector init_pos;
	// std::fstream finit;
	// finit.open(tracker_path+"/model/file_init.pos", std::ios::in);
	// for (unsigned int i = 0; i < 6; i += 1) {
    // 	finit >> init_pos[i];
   	// }
	// vpHomogeneousMatrix vpcTo_est;
   	// vpcTo_est.buildFrom(init_pos);
	// cTo_est.pose.pose = visp_bridge::toGeometryMsgsPose(vpcTo_est);

	// Send image and cam par to service, where Python node responds with cao_file and pose
	ROS_INFO("Subscribing to service...");
  	ros::service::waitForService("/YolactInitializeCaoPose");
	// tracker_visp::PoseEstimation srv;
	tracker_visp::YolactInitializeCaoPose srv;
	srv.request.image = sensor_image;
	srv.request.camInfo = sensor_camInfo;
	// srv.request.cTo_est = cTo_est;
	ROS_INFO("Starting call to service..");
	if (client.call(srv))
	{
		ROS_INFO("Response from service..");
		model_cao=srv.response.caoFilePath.data;
		if (model_cao!=""){
			ROS_INFO("Image accepted by user");
			geometry_msgs::Pose initPose=srv.response.initPose.pose.pose;
			cMo=visp_bridge::toVispHomogeneousMatrix(initPose);	//object pose cMo
        }
		else {
			ROS_INFO("Image discarded");
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