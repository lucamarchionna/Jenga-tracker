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

#include "tf2_msgs/TFMessage.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Transform.h>
#include <visp_bridge/3dpose.h>
#include <visp_bridge/image.h>
#include <visp_bridge/camera.h>
#include <boost/bind.hpp>
#include "std_msgs/String.h"
#include "tracker_visp/YolactInitializeCaoPose.h"

bool opt_learn = true;
bool opt_auto_init = false;
bool git = true;
bool learn_position = false;

// Convert vpHomogeneus to TFMessage
tf2_msgs::TFMessage toMoveit(vpHomogeneousMatrix data) {
  tf2_msgs::TFMessage pose_tracker;
  geometry_msgs::TransformStamped target;

  target.header.stamp=ros::Time::now();
  target.header.frame_id="object";
  target.child_frame_id="world";
  target.transform=visp_bridge::toGeometryMsgsTransform(data); 
  pose_tracker.transforms.push_back(target);

  return pose_tracker;
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
  ros::init(argc, argv, "traker_client");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(1); 
  spinner.start();
  ros::Rate loop_rate(10);
  ros::Publisher pub = node.advertise<tf2_msgs::TFMessage>("/tf", 1);
  ros::Subscriber sub = node.subscribe("learning_phase", 1, learningCallback);
  ros::ServiceClient client = node.serviceClient<tracker_visp::YolactInitializeCaoPose>("Pose_cao_initializer");

  std::string opt_config = "../trackers/jenga_tracker_params.xml";
  std::string opt_model = "";
  std::string opt_init = "";
  //std::string opt_init_pos="../model/file_init.pos";
  std::string opt_learning_data = "../learning/0/data-learned.bin";
  std::string opt_keypoint_config = "../learning/keypoint_config.xml";
  
  double opt_proj_error_threshold = 25;
  double opt_setGoodME_thresh = 0.4;
  int opt_disp_visibility = 0;

  bool opt_display_projection_error = false;
  bool opt_display_features = false;

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

  vpCameraParameters cam_color = realsense.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
  vpCameraParameters cam_depth = realsense.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithoutDistortion);

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
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformations;
  std::map<std::string, const vpImage<vpRGBa> *> mapOfImages;
  std::map<std::string, std::string> mapOfInitFiles;
  std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
  std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraPoses;
  std::vector<vpColVector> pointcloud;

  // [Load all info in the tracker]
  // Define tracker types
  std::vector<int> trackerTypes;
  trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
  trackerTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);
  vpMbGenericTracker tracker(trackerTypes);

  //Map of cameras to use both types
  tracker.loadConfigFile(opt_config, opt_config);
  //tracker.loadModel(opt_model, opt_model);
  std::cout << "Sensor internal depth_M_color: \n" << depth_M_color << std::endl;
  mapOfCameraTransformations["Camera2"] = depth_M_color;
  tracker.setCameraTransformationMatrix(mapOfCameraTransformations);
  mapOfImages["Camera1"] = &I_color;
  mapOfImages["Camera2"] = &I_depth_color;
  mapOfInitFiles["Camera1"] = opt_init;
  //mapOfInitPoses["Camera1"] = opt_init_pos;
  tracker.setCameraParameters(cam_color, cam_depth);

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

  // Object pose matrix in camera
  vpHomogeneousMatrix cMo;

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
  else {
    // // Initialize with clicks from .init
    //tracker.initClick(mapOfImages, mapOfInitFiles, true);
    // // Initialize from pose file .pos
    // tracker.initFromPose(mapOfImages,mapOfInitPos);
    // // Initialize from neural network
    tracker_visp::YolactInitializeCaoPose srv;
    srv.request.image = visp_bridge::toSensorMsgsImage(I_color);
    srv.request.camInfo = visp_bridge::toSensorMsgsCameraInfo(cam_color,width,height);
    if (client.call(srv))
    {
      opt_model=srv.response.caoFilePath.data;
      geometry_msgs::Transform initPose=srv.response.initPose;
      cMo=visp_bridge::toVispHomogeneousMatrix(initPose);
      tracker.loadModel(opt_model, opt_model);
      mapOfCameraPoses["Camera1"] = cMo;
      mapOfCameraPoses["Camera2"] = depth_M_color *cMo;
      tracker.initFromPose(mapOfImages,mapOfCameraPoses);
    }
  }

  // Before the loop
  bool run_auto_init = false;
  if (opt_auto_init) {
    run_auto_init = true;
  }
  std::vector<double> times_vec;

  try {
    //To be able to display keypoints matching with test-detection-rs2
    int learn_id = 1;
    unsigned int learn_cpt = 0;
  
    double loop_t = 0;
    
    bool quit = false;
    
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
        vpImageConvert::convert(I_color, I_gray);
        keypoint.insertImageMatching(I_gray, IMatching);
        // DETECTION
        if (keypoint.matchPoint(I_gray, cam_color, cMo, error, elapsedTime)) {
          std::cout << "Auto init succeed in elapsed time: " << elapsedTime << " ms" << std::endl;
          // Show matchings on learned images
          vpDisplay::display(IMatching);
          keypoint.displayMatching(I_gray, IMatching);
          vpDisplay::flush(IMatching);
          mapOfCameraPoses["Camera1"] = cMo;
          mapOfCameraPoses["Camera2"] = depth_M_color *cMo;
          // Initialize from detection
          tracker.initFromPose(mapOfImages, mapOfCameraPoses);
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

      // Get and publish object pose
      cMo = tracker.getPose();
      tf2_msgs::TFMessage pose_target = toMoveit(cMo);
      pub.publish(pose_target);

      // Check tracking errors
      double proj_error = 0;
      if (tracker.getTrackerType() & vpMbGenericTracker::EDGE_TRACKER) {
        // Check tracking errors
        proj_error = tracker.getProjectionError();
      }

      if (proj_error > opt_proj_error_threshold) {
        //std::cout << "Tracker needs to restart (projection error detected: " << proj_error << ")" << std::endl;
        if (opt_auto_init) {
            run_auto_init = true;
          }
        tracking_failed = true;
      }

      // Display tracking results
      if (!tracking_failed) {
        // Turn display features on
        tracker.setDisplayFeatures(opt_display_features);

        // Display frames
        tracker.display(I_color, I_depth_color, cMo, depth_M_color*cMo, cam_color, cam_depth, vpColor::green, 3);
        vpDisplay::displayFrame(I_color, cMo, cam_color, 0.05, vpColor::none, 3);
        vpDisplay::displayFrame(I_depth_color, depth_M_color*cMo, cam_depth, 0.05, vpColor::none, 3);
        // { // Display estimated pose in [m] and [deg]
        //   vpPoseVector pose(cMo);
        //   std::stringstream ss;
        //   ss << "Translation: " << std::setprecision(5) << pose[0] << " " << pose[1] << " " << pose[2] << " [m]";
        //   vpDisplay::displayText(I_color, I_color.getHeight() - 100, 20, ss.str(), vpColor::black);
        //   ss.str(""); // erase ss
        //   ss << "Rotation tu: " << std::setprecision(4) << vpMath::deg(pose[3]) << " " << vpMath::deg(pose[4]) << " " << vpMath::deg(pose[5]) << " [deg]";
        //   vpDisplay::displayText(I_color, I_color.getHeight() - 80, 20, ss.str(), vpColor::black);
        // }
        // {
        //   std::stringstream ss;
        //   ss << "Nb features: " << tracker.getError().size();
        //   vpDisplay::displayText(I_color, I_color.getHeight() - 50, 20, ss.str(), vpColor::red);
        // }
        // {
        //   std::stringstream ss;
        //   ss << "Features: edges " << tracker.getNbFeaturesEdge()
        //      << ", klt " << tracker.getNbFeaturesKlt()
        //      << ", depth " << tracker.getNbFeaturesDepthDense();
        //   vpDisplay::displayText(I_color, I_color.getHeight() - 30, 20, ss.str(), vpColor::red);
        // }
      }

      std::stringstream ss;
      ss << "Loop time: " << loop_t << " ms";

      vpMouseButton::vpMouseButtonType button;
      
      vpDisplay::displayText(I_color, 20, 20, ss.str(), vpColor::red);
      if (opt_learn)
        vpDisplay::displayText(I_color, 35, 20, "Left click: learn  Right click: quit", vpColor::red);
      else if (opt_auto_init)
        vpDisplay::displayText(I_color, 35, 20, "Left click: auto_init  Right click: quit", vpColor::red);
      else
        vpDisplay::displayText(I_color, 35, 20, "Right click: quit", vpColor::red);
      vpDisplay::flush(I_color);

      vpDisplay::displayText(I_depth_color, 20, 20, ss.str(), vpColor::red);
      vpDisplay::displayText(I_depth_color, 40, 20, "Click to quit", vpColor::red);
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

      if (learn_position) {
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

        // Display learned data
        for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
          vpDisplay::displayCross(I_color, (int)it->pt.y, (int)it->pt.x, 10, vpColor::yellow, 3);
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
      }

      // Loop time
      loop_t = vpTime::measureTimeMs() - t;
      times_vec.push_back(loop_t);
    
    }
    //! -----------------------------------------------------------------------------------------------
    //! [END OF LOOP]

    // Terminate learning phase, save all on exit
    if (opt_learn && learn_cpt) {
      std::cout << "Save learning from " << learn_cpt << " images in file: " << opt_learning_data << std::endl;
      keypoint.saveLearningData(opt_learning_data, true, true);
    }

    if (!times_vec.empty()) {
    std::cout << "\nProcessing time, Mean: " << vpMath::getMean(times_vec) << " ms ; Median: " << vpMath::getMedian(times_vec)
              << " ; Std: " << vpMath::getStdev(times_vec) << " ms" << std::endl;
    }

    // Keep ros node alive
    ros::spinOnce();
    loop_rate.sleep();

  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
