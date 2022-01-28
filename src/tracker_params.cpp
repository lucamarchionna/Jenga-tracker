#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/vision/vpKeyPoint.h>

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
#include "std_msgs/String.h"
#include "tracker_visp/YolactInitializeCaoPose.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <iostream>
#include <ros/package.h>
#include <boost/filesystem.hpp>


using namespace std;

bool opt_learn = true;
bool opt_auto_init = false;

bool learn_position = false;

vpHomogeneousMatrix homogeneousTransformation(string link1, string link3) 
{

  vpHomogeneousMatrix homogeneousMatrix;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  tf::StampedTransform transform;
  ros::Time t = ros::Time(0);
  
  try {
    transformStamped = tfBuffer.lookupTransform(link1, link3, t, ros::Duration(3));
    vpHomogeneousMatrix homogeneousMatrix = visp_bridge::toVispHomogeneousMatrix(transformStamped.transform);
    return homogeneousMatrix;
  }

  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    //ros::Duration(1.0).sleep();
  
  }
  return homogeneousMatrix;
}

// Convert vpHomogeneus to TFMessage
geometry_msgs::TransformStamped toMoveit(vpHomogeneousMatrix data, std::string header, std::string child_frame) {
  geometry_msgs::TransformStamped target;

  target.header.stamp=ros::Time::now();
  target.header.frame_id=header;
  target.child_frame_id=child_frame;
  target.transform=visp_bridge::toGeometryMsgsTransform(data); 

  return target;
}

void learningCallback(const std_msgs::Bool::ConstPtr& msg)
{ 
  opt_learn = false;
  learn_position = false;

  if (!opt_auto_init) {
    opt_learn = msg->data;
  }

  if (opt_learn) {
    learn_position = true;
  }

}



int main(int argc, char *argv[])
{
  // Object pose matrix in camera
  vpHomogeneousMatrix cMo, cTo, eeTc, eeTc1, wTee;
  static vpHomogeneousMatrix wTc, wTc1;
  ros::init(argc, argv, "tracker_params");
  ros::NodeHandle node("~");
  ros::AsyncSpinner spinner(1); 
  spinner.start();
  
  ros::Rate loop_rate(20);
  ros::Subscriber sub = node.subscribe("learning_phase", 1, learningCallback);
  ros::ServiceClient client = node.serviceClient<tracker_visp::YolactInitializeCaoPose>("Pose_cao_initializer");
  ros::Publisher trackerEstimation = node.advertise<geometry_msgs::Pose>("/pose_estimation", 1);

  
  std::string tracker_path = ros::package::getPath("tracker_visp");
  std::string opt_config = tracker_path + "/trackers/jenga_tracker_params.xml";
  std::string opt_model = tracker_path + "/model/file_cao.cao";
  std::string opt_init = tracker_path + "/model/jenga_single.init";
  std::string opt_init_pos= tracker_path + "/model/file_init.pos";
  std::string opt_learning_data = tracker_path + "/learning/1/data-learned.bin";
  std::string opt_keypoint_config = tracker_path + "/learning/keypoint_config.xml";
  double opt_proj_error_threshold = 25;
  double opt_setGoodME_thresh = 0.4;
  int opt_disp_visibility = 0;
  bool opt_display_projection_error = false;
  bool opt_display_features = false;
  bool opt_display_model = true;
  bool opt_yolact_init = false;
  bool opt_pose_init = false;

  //node.param<std::string>("learning_data", opt_learning_data, tracker_path + "/learning/1/data-learned.bin");
  node.param("proj_error_threshold",opt_proj_error_threshold,25.0);
  node.param("goodME_thresh",opt_setGoodME_thresh,0.4);
  node.param("display_features",opt_display_features,false);
  node.param("display_model",opt_display_model,true);
  node.param("display_visibility",opt_disp_visibility,0);
  node.param("display_proj_error",opt_display_projection_error,false);
  node.param("learn",opt_learn,false);
  node.param("auto_init",opt_auto_init,false);
  node.param("yolact_init",opt_yolact_init,false);
  node.param("pose_init",opt_pose_init,true);

  std::cout << "Tracker options: " << std::endl;
  std::cout << "  Proj. error : " << opt_proj_error_threshold << std::endl;
  std::cout << "  Display proj. error: " << opt_display_projection_error << std::endl;
  std::cout << "Config files: " << std::endl;
  std::cout << "  Model : " << "\"" << opt_model << "\"" << std::endl;
  std::cout << "  Init file   : " << "\"" << opt_init << "\"" << std::endl;
  std::cout << "  Init pos file   : " << "\"" << opt_init_pos << "\"" << std::endl;
  std::cout << "Yolact options   : " << std::endl;
  std::cout << "  Yolact init   : " << opt_yolact_init << std::endl;
  std::cout << "Learning options   : " << std::endl;
  std::cout << "  Learn       : " << opt_learn << std::endl;
  std::cout << "  Auto init   : " << opt_auto_init << std::endl;
  std::cout << "  Learning data: " << opt_learning_data << std::endl;

  // [Realsense camera configuration]
  vpRealSense2 realsense;
  int width = 640, height = 480;
  int fps = 30;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

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

  vpCameraParameters cam_color = realsense.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
  vpCameraParameters cam_depth = realsense.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithoutDistortion);
  std::cout << "cam_color\n " << cam_color << std::endl;
  std::cout << "cam_depth\n " << cam_depth << std::endl;

  // [Configuration of display]
  vpImage<vpRGBa> I_color(height, width);
  vpImage<unsigned char> I_gray(height, width);
  vpImage<unsigned char> I_depth_gray(height, width);
  vpImage<vpRGBa> I_depth_color(height, width);
  vpImage<uint16_t> I_depth_raw(height, width);

  unsigned int _posx = 100, _posy = 50;

  vpDisplayOpenCV d1, d2;
  d1.init(I_color, _posx, _posy, "Color stream");
  d2.init(I_depth_color, _posx, _posy + I_gray.getHeight()+10, "Depth stream");

  vpDisplayOpenCV d3;
  //! [Display (set to Opencv)]

  vpImage<vpRGBa> ILearned(I_color.getHeight()*2,I_color.getWidth()*2);
  if (opt_learn){
    d3.init(ILearned, _posx+I_color.getWidth()*2+10, _posy, "Learned-images");
  }

  // [Acquire stream and initialize displays]
  while (true) {
    realsense.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, NULL, NULL);

    vpDisplay::display(I_color);
    vpDisplay::displayText(I_color, 20, 20, "Click when ready.", vpColor::red);
    vpDisplay::flush(I_color);
    if (vpDisplay::getClick(I_color, false))
      break;

    vpImageConvert::createDepthHistogram(I_depth_raw, I_depth_gray);
    vpImageConvert::convert(I_depth_gray,I_depth_color);
    vpDisplay::display(I_depth_color);
    vpDisplay::displayText(I_depth_color, 20, 20, "Click when ready.", vpColor::red);
    vpDisplay::flush(I_depth_color);
    if (vpDisplay::getClick(I_depth_color, false))
      break;
  }

  // [Map of images to use tracker with depth]
  vpHomogeneousMatrix depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
  std::cout << "depth_M_color\n " << depth_M_color << std::endl;
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformations;
  std::map<std::string, const vpImage<vpRGBa> *> mapOfImages;
  std::map<std::string, std::string> mapOfInitFiles;
  std::map<std::string, std::string> mapOfInitPoses;
  std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
  std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraPoses;
  std::vector<vpColVector> pointcloud;

  mapOfCameraTransformations["Camera2"] = depth_M_color;
  mapOfImages["Camera1"] = &I_color;
  mapOfImages["Camera2"] = &I_depth_color;
  mapOfInitFiles["Camera1"] = opt_init;
  mapOfInitPoses["Camera1"] = opt_init_pos;

  // [Load all info in the tracker]
  // Define tracker types
  std::vector<int> trackerTypes;
  trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
  trackerTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);
  vpMbGenericTracker tracker(trackerTypes);

  tracker.loadConfigFile(opt_config, opt_config);
  tracker.loadModel(opt_model, opt_model);
  tracker.setCameraTransformationMatrix(mapOfCameraTransformations);
  tracker.setCameraParameters(cam_color, cam_depth);

  // Set keypoints type from code if not from file
  std::string detectorName = "FAST";
  std::string extractorName = "ORB";
  std::string matcherName = "BruteForce-Hamming";
  vpKeyPoint keypoint;
  if (opt_learn || opt_auto_init) {
    // Load keypoints config
    if (vpIoTools::checkFilename(opt_keypoint_config)){
      keypoint.loadConfigFile(opt_keypoint_config);
    }
    else if (opt_learn || opt_auto_init) {
      keypoint.setDetector(detectorName);
      keypoint.setExtractor(extractorName);
      keypoint.setMatcher(matcherName);
    }
  }
  // Directory to store learning data
  if (opt_learn && !opt_auto_init)
    vpIoTools::makeDirectory(vpIoTools::getParent(opt_learning_data));

  // Load auto learn data, or initialize with clicks
  vpImage<unsigned char> IMatching;
  if (opt_auto_init) {
    // Preparation for automatic initialization
    if (!vpIoTools::checkFilename(opt_learning_data)) {
      std::cout << "Cannot enable auto detection. Learning file \"" << opt_learning_data << "\" doesn't exist" << std::endl;
      return EXIT_FAILURE;
    }
    keypoint.loadLearningData(opt_learning_data, true);
    vpImageConvert::convert(I_color,I_gray);
    keypoint.createImageMatching(I_gray, IMatching);
    d3.setDownScalingFactor(vpDisplay::SCALE_AUTO);
    d3.init(IMatching,_posx+I_gray.getWidth()*2+10, _posy,"Detection-from-learned");
  } 
  else if (opt_yolact_init) {
    // Send image and cam par to service, where Python node responds with cao_file and pose
    tracker_visp::YolactInitializeCaoPose srv;
    srv.request.image = visp_bridge::toSensorMsgsImage(I_color);
    srv.request.camInfo = visp_bridge::toSensorMsgsCameraInfo(cam_color,width,height);
    std::cout << "Starting call to service.." << std::endl;
    if (client.call(srv))
    {
      std::cout << "Response from service.." << std::endl;
      opt_model=srv.response.caoFilePath.data;
      geometry_msgs::Transform initPose=srv.response.initPose;
      cMo=visp_bridge::toVispHomogeneousMatrix(initPose);
      tracker.loadModel(opt_model, opt_model);
      mapOfCameraPoses["Camera1"] = cMo;
      mapOfCameraPoses["Camera2"] = depth_M_color *cMo;
      tracker.initFromPose(mapOfImages,mapOfCameraPoses);
    }
    else{
      std::cout << "Cannot receive response from server" << std::endl;
      return EXIT_FAILURE;
    }
  }
  else {
    if (opt_pose_init){
      // // Initialize from pose file .pos
      tracker.initFromPose(mapOfImages,mapOfInitPoses);
    }
    else {
      // // Initialize with clicks from .init
      tracker.initClick(mapOfImages, mapOfInitFiles, true);
    }
  }

  // Display and error
  tracker.setDisplayFeatures(opt_display_features);
  tracker.setProjectionErrorComputation(true);
  tracker.setProjectionErrorDisplay(opt_display_projection_error);

  // Force good moving edges ratio threshold
  tracker.setGoodMovingEdgesRatioThreshold(opt_setGoodME_thresh);  //default=0.4

  // Remove faces with name "occluded" from the tracking, a priori
  tracker.setUseEdgeTracking("occluded",false);
  tracker.setUseKltTracking("occluded",false);
  tracker.setUseDepthDenseTracking("occluded",false);

  // Before the loop
  bool run_auto_init = false;
  if (opt_auto_init) {
    run_auto_init = true;
  }
  std::vector<double> times_vec;
  std::vector<double> train_t_vec;
  std::vector<double> detect_t_vec;

  vpVideoWriter writer;
  //Initialize the writer.
  if (opt_learn && opt_display_features){
    vpIoTools::makeDirectory(vpIoTools::getParent(opt_learning_data).append("_cross"));
    writer.setFileName(vpIoTools::getParent(opt_learning_data).append("_cross/image%04d.jpeg"));
    writer.open(I_color);
  }

  try {
    
    //To be able to display keypoints matching with test-detection-rs2
    int learn_id = 1;
    unsigned int learn_cpt = 0;
  
    double loop_t = 0;
    
    bool quit = false;
    static tf2_ros::TransformBroadcaster br;

    // Define camera's RF for girst initialization
    wTc = homogeneousTransformation("world", "camera_color_optical_frame");
    vpTranslationVector t1; t1 << 0.0,0, 0;
    vpQuaternionVector q_1; q_1 << 0, 0, 0, 1;
    
    eeTc1.buildFrom(t1, q_1);
    wTc1 = wTc*eeTc1.inverse();


    
    
    
    //! [LOOP]
    //! -----------------------------------------------------------------------------------------------
    while (!quit ) {
      
      double t = vpTime::measureTimeMs();
      bool tracking_failed = false;
      double elapsedTime = 0;

      // Acquire images, visualize and update tracker input data
      realsense.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, &pointcloud, NULL, NULL);
      vpDisplay::display(I_color);
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth_gray);
      vpImageConvert::convert(I_depth_gray, I_depth_color);
      vpDisplay::display(I_depth_color);

      mapOfImages["Camera1"] = &I_color;
      mapOfPointclouds["Camera2"] = &pointcloud;
      mapOfWidths["Camera2"] = width;
      mapOfHeights["Camera2"] = height;

      // Run auto initialization from learned data
      if (run_auto_init) {
        tracking_failed = false;
        double error;
        double detect_t= vpTime::measureTimeMs();
        vpImageConvert::convert(I_color, I_gray);
        keypoint.insertImageMatching(I_gray, IMatching);
        // DETECTION
        if (keypoint.matchPoint(I_gray, cam_color, cMo, error, elapsedTime)) {
          std::cout << "Auto init succeed in elapsed time: " << elapsedTime << " ms" << std::endl;
          // Show matchings on learned images
          vpDisplay::display(IMatching);
          keypoint.displayMatching(I_gray, IMatching);
          std::stringstream ss;
          ss << "Number of matchings: " << keypoint.getMatchedPointNumber();
          vpDisplay::displayText(IMatching, 20, 20, ss.str(), vpColor::red);
          ss.str("");
          ss  << "Number of references: " << keypoint.getReferencePointNumber();
          vpDisplay::displayText(IMatching, 60, 20, ss.str(), vpColor::red);
          vpDisplay::flush(IMatching);
          mapOfCameraPoses["Camera1"] = cMo;
          mapOfCameraPoses["Camera2"] = depth_M_color *cMo;
          // Initialize from detection
          tracker.initFromPose(mapOfImages, mapOfCameraPoses);
          std::cout << "Detection succeed in elapsed time: " << vpTime::measureTimeMs() - detect_t << " ms" << std::endl;
          detect_t_vec.push_back(vpTime::measureTimeMs() - detect_t);
        } 
        else {
          vpDisplay::flush(I_color);
          vpDisplay::flush(I_depth_color);
          //Keep searching
          continue;
        }
      }
      else if (tracking_failed) {
        // Manual init
        tracking_failed = false;
        tracker.initClick(mapOfImages, mapOfInitFiles, true);
      }

      // Run the tracker
      try {
        if (run_auto_init) {
          // Turn display features off just after auto init to not display wrong moving-edge if the tracker fails
          tracker.setDisplayFeatures(false);
          run_auto_init = false;
        }
        tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
      } catch (const vpException &e) {
        std::cout << "Tracker exception: " << e.getStringMessage() << std::endl;
        tracking_failed = true;
        if (opt_auto_init) {
          //std::cout << "Tracker needs to restart (tracking exception)" << std::endl;
          run_auto_init = true;
        }
      }


      // Define camera's RF
      
      vpTranslationVector t_cam; t_cam << 0.54, 0.08, 0.4;
      vpQuaternionVector q_cam, q_unit; q_cam << 0, 0, 0.7071068, 0.7071068;
      /*
      eeTc.buildFrom(t_cam, q_cam);
      geometry_msgs::TransformStamped camera_RF = toMoveit(eeTc, "world", "object");
      br.sendTransform(camera_RF);
      */
      //geometry_msgs::TransformStamped camera_1 = toMoveit(wTc1, "world", "camera_1");
      //br.sendTransform(camera_1);

      // Get and publish object pose
      cMo = tracker.getPose();
      //cTo = cMo;
      //q_unit << 0, 0, 0, 1;
      //cTo.insert(q_unit); //only for simulation
      geometry_msgs::TransformStamped pose_target = toMoveit(cMo, "camera_color_optical_frame" , "handeye_target");
      br.sendTransform(pose_target);

      
      geometry_msgs::Pose cTo_P;
      cTo_P = visp_bridge::toGeometryMsgsPose(cMo); 
      trackerEstimation.publish(cTo_P);
      

      // Check tracking errors
      double proj_error = 0;
      if (tracker.getTrackerType() & vpMbGenericTracker::EDGE_TRACKER) {
        // Check tracking errors
        proj_error = tracker.getProjectionError();
      }

      if (proj_error > opt_proj_error_threshold) {
        std::cout << "Tracker needs to restart (projection error detected: " << proj_error << ")" << std::endl;
        if (opt_auto_init) {
            run_auto_init = true;
          }
        tracking_failed = true;
      }

      // Display tracking results
      if (!tracking_failed) {
        // Turn display features on
        if (opt_display_features){
          tracker.setDisplayFeatures(opt_display_features);
          {
            std::stringstream ss;
            ss << "Nb features: " << tracker.getError().size();
            vpDisplay::displayText(I_color, I_color.getHeight() - 50, 20, ss.str(), vpColor::red);
          }
          {
            std::stringstream ss;
            ss << "Features: edges " << tracker.getNbFeaturesEdge()
              << ", klt " << tracker.getNbFeaturesKlt()
              << ", depth " << tracker.getNbFeaturesDepthDense();
            vpDisplay::displayText(I_color, I_color.getHeight() - 30, 20, ss.str(), vpColor::red);
          }
        }

        // Display model
        if (opt_display_model)
          tracker.display(I_color, I_depth_color, cMo, depth_M_color*cMo, cam_color, cam_depth, vpColor::green, 1);
        
        // Display frames
        vpDisplay::displayFrame(I_color, cMo, cam_color, 0.05, vpColor::none, 3);
        vpDisplay::displayFrame(I_depth_color, depth_M_color*cMo, cam_depth, 0.05, vpColor::none, 3);
        // Display estimated pose in [m] and [deg]
        // { 
        //   vpPoseVector pose(cMo);
        //   std::stringstream ss;
        //   ss << "Translation: " << std::setprecision(5) << pose[0] << " " << pose[1] << " " << pose[2] << " [m]";
        //   vpDisplay::displayText(I_color, I_color.getHeight() - 100, 20, ss.str(), vpColor::black);
        //   ss.str(""); // erase ss
        //   ss << "Rotation tu: " << std::setprecision(4) << vpMath::deg(pose[3]) << " " << vpMath::deg(pose[4]) << " " << vpMath::deg(pose[5]) << " [deg]";
        //   vpDisplay::displayText(I_color, I_color.getHeight() - 80, 20, ss.str(), vpColor::black);
        // }
      }

      vpMouseButton::vpMouseButtonType button;
      
      // if (opt_learn)
        //vpDisplay::displayText(I_color, 35, 20, "Left click: learn  Right click: quit", vpColor::red);
      // else if (opt_auto_init)
        //vpDisplay::displayText(I_color, 35, 20, "Left click: auto_init  Right click: quit", vpColor::red);
      // else
        //vpDisplay::displayText(I_color, 35, 20, "Right click: quit", vpColor::red);
      vpDisplay::flush(I_color);

      //vpDisplay::displayText(I_depth_color, 20, 20, ss.str(), vpColor::red);
      //vpDisplay::displayText(I_depth_color, 40, 20, "Click to quit", vpColor::red);
      vpDisplay::flush(I_depth_color);

      if (vpDisplay::getClick(I_color, button, false)) {
        if (button == vpMouseButton::button3) {
          quit = true;
        } else if (button == vpMouseButton::button1 && opt_learn) {
          learn_position = true;
        } else if (button == vpMouseButton::button1 && opt_auto_init && !opt_learn) {
          run_auto_init = true;
        }
      }
      if (vpDisplay::getClick(I_depth_color, false)) {
        quit = true;
      }

      // Loop time without training
      loop_t = vpTime::measureTimeMs() - t;
      times_vec.push_back(loop_t);
      // std::stringstream ss;
      // ss << "Loop time: " << loop_t << " ms";
      // vpDisplay::displayText(I_color, 20, 20, ss.str(), vpColor::red);

      if (learn_position) {
        double train_t = vpTime::measureTimeMs();

        learn_cpt ++;
        // Detect keypoints on the current image
        std::vector<cv::KeyPoint> trainKeyPoints;
        keypoint.detect(I_color, trainKeyPoints);

        // Keep only keypoints on the object
        std::vector<vpPolygon> polygons;
        std::vector<std::vector<vpPoint> > roisPt;
        std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces();
        polygons = pair.first;
        roisPt = pair.second;

        // Compute the 3D coordinates
        std::vector<cv::Point3f> points3f;
        vpKeyPoint::compute3DForPointsInPolygons(cMo, cam_color, trainKeyPoints, polygons, roisPt, points3f);

        // Build the reference keypoints
        keypoint.buildReference(I_color, trainKeyPoints, points3f, true, learn_id++);

        if(opt_display_features){
          writer.close();
          // Display learned data
          for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
            vpDisplay::displayCross(I_color, (int)it->pt.y, (int)it->pt.x, 10, vpColor::yellow, 3);
            vpImageDraw::drawCross(I_color,vpImagePoint((int)it->pt.y, (int)it->pt.x), 10, vpColor::yellow, 2);
            vpFont font(20);
            std::stringstream ss;
            ss << "Number: " << trainKeyPoints.size();
            font.drawText(I_color, ss.str(), vpImagePoint(35,20), vpColor::red);
          }
          writer.saveFrame(I_color);
          
        }

        // Display saved learned image
        if (((learn_cpt-1)%4)==0){
          ILearned.insert(I_color,vpImagePoint(0,0));
        }
        else if (((learn_cpt-1)%4)==1){
          ILearned.insert(I_color,vpImagePoint(0,I_color.getWidth()));
        }
        else if (((learn_cpt-1)%4)==2){
          ILearned.insert(I_color,vpImagePoint(I_color.getHeight(),0)); 
        }
        else if (((learn_cpt-1)%4)==3){
          ILearned.insert(I_color,vpImagePoint(I_color.getHeight(),I_color.getWidth())); 
        }
        vpDisplay::display(ILearned);
        vpDisplay::flush(ILearned);

        learn_position = false;
        std::cout << "Data learned" << std::endl;

        train_t_vec.push_back(vpTime::measureTimeMs() - train_t);
      }



    }
    //! -----------------------------------------------------------------------------------------------
    //! [END OF LOOP]

    // Terminate learning phase, save all on exit
    if (opt_learn) {
      
      if (learn_cpt>0){
        std::cout << "Save learning from " << learn_cpt << " images in file: " << opt_learning_data << std::endl;
        keypoint.saveLearningData(opt_learning_data, true, true);
      }
    }

    if (!times_vec.empty()) {
    std::cout << "\nProcessing time, Mean: " << vpMath::getMean(times_vec) << " ms ; Median: " << vpMath::getMedian(times_vec)
              << " ; Std: " << vpMath::getStdev(times_vec) << " ms" << std::endl;
    }
    if (!train_t_vec.empty()) {
    std::cout << "\nTrain:\nProcessing time, Mean: " << vpMath::getMean(train_t_vec) << " ms ; Median: " << vpMath::getMedian(train_t_vec)
              << " ; Std: " << vpMath::getStdev(train_t_vec) << " ms" << std::endl;
    }
    if (!detect_t_vec.empty()) {
    std::cout << "\nDetection:\nProcessing time, Mean: " << vpMath::getMean(detect_t_vec) << " ms ; Median: " << vpMath::getMedian(detect_t_vec)
              << " ; Std: " << vpMath::getStdev(detect_t_vec) << " ms" << std::endl;
    }

    // Keep ros node alive
    ros::spinOnce();
    loop_rate.sleep(); 
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
