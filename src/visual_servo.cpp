#include <full_visual_servo/visual_servo.h>


using namespace std;

visual_servoing::visual_servoing(ros::NodeHandle& nh) : node_handle(nh)
{

  client = node_handle.serviceClient<tracker_visp::YolactInitializeCaoPose>("/Pose_cao_initializer");
  trackerEstimation = node_handle.advertise<geometry_msgs::Pose>("/pose_estimation", 1);
  velocityInput = node_handle.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 1);
  startingPos = node_handle.advertise<geometry_msgs::PoseStamped>("/initialGuestPos", 1);
  servoPub = node_handle.advertise<std_msgs::UInt16>("/servo", 1);
  //subLearning = node_handle.subscribe("/tracker_params/learning_phase", 1, &visual_servoing::learningCallback, this);

  init_parameters();
  init_matrices();
  init_servo();
  learning_process();
  ROS_INFO_STREAM("Leaning process finished");
  ros::Duration(1.0).sleep();

  detection_process();

  
  
}

void visual_servoing::estimationCallback(const geometry_msgs::Pose& tracker_pose_P) 
{

  vpColVector v_ee(3), omega_ee(3), v(6), e1(6);
  //e1 = 0;

  if (!block_axis) {
    camTtarget = visp_bridge::toVispHomogeneousMatrix(tracker_pose_P); 
    targetTcam = camTtarget.inverse();
    cdTc = cdTtarget*targetTcam;
  }

  else {
    camTbase = homogeneousTransformation("camera_color_optical_frame", "edo_base_link");
    camTtarget = camTbase*baseTtarget;
    targetTcam = camTtarget.inverse();
    cdTc = cdTtarget*targetTcam;
  }

  s.buildFrom(camTtarget); //linear
  s_tu.buildFrom(cdTc); //angular

  v_cam = task.computeControlLaw();

  error = (task.getError()).sumSquare(); // error = s^2 - s_star^2

  //Convert velocities into the end-effector RF
  v_ee = camTee.getRotationMatrix()*v_cam.rows(1,3);
  
  omega_ee = camTee.getRotationMatrix()*v_cam.rows(4,6);
  v.insert(0, v_ee); v.insert(3, omega_ee);
  velocityData = insertData(v);

  if (error>=threshold && !block_axis) {
    
    velocityInput.publish(velocityData);
    vpColVector e = s.error(s_star); // e = (s-s*)

  }

  else {

    if (take_cTo) {
      // baseTtarget = homogeneousTransformation("edo_base_link", "handeye_target");
      // ros::Duration(3).sleep();
      // ROS_INFO("New pose took");
      take_cTo = false;
      baseTtarget.print();
    }

    else {    
      block_axis = true;
      //velocityData = approach(velocityData);
      velocityData.twist.linear.z = 0.04;
      velocityInput.publish(velocityData);
      }
    }



}




vpHomogeneousMatrix visual_servoing::homogeneousTransformation(string link1, string link3) 
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

geometry_msgs::PoseStamped visual_servoing::insertPosData(vpHomogeneousMatrix T_matrix) 
{
  geometry_msgs::PoseStamped to_PS;
  vpQuaternionVector q;
  T_matrix.extract(q);

  to_PS.pose.position.x = T_matrix[0][3];
  to_PS.pose.position.y = T_matrix[1][3]; 
  to_PS.pose.position.z = T_matrix[2][3];
 
  to_PS.pose.orientation.x = q[0]; 
  to_PS.pose.orientation.y = q[1]; 
  to_PS.pose.orientation.z = q[2]; 
  to_PS.pose.orientation.w = q[3];

  return to_PS;
}

geometry_msgs::TwistStamped visual_servoing::insertData(vpColVector data) 
{
  geometry_msgs::TwistStamped vel;
  vel.header.stamp = ros::Time::now();
  vel.twist.linear.x = data[0]; vel.twist.linear.y = data[1]; vel.twist.linear.z = data[2];
  vel.twist.angular.x = data[3]; vel.twist.angular.y = data[4]; vel.twist.angular.z = 0;

  return vel;
}



// Convert vpHomogeneus to TFMessage
geometry_msgs::TransformStamped visual_servoing::toMoveit(vpHomogeneousMatrix data, std::string header, std::string child_frame) {
  geometry_msgs::TransformStamped target;

  target.header.stamp=ros::Time::now();
  target.header.frame_id=header;
  target.child_frame_id=child_frame;
  target.transform=visp_bridge::toGeometryMsgsTransform(data); 

  return target;
}

// void visual_servoing::learningCallback(const std_msgs::Bool::ConstPtr& msg)
// { 
//   opt_learn = false;
//   learn_position = false;

//   if (!opt_auto_init) {
//     opt_learn = msg->data;
//   }

//   if (opt_learn) {
//     learn_position = true;
//   }

// }

void visual_servoing::learningCallback(const bool msg) {

  opt_learn = false;
}

void visual_servoing::init_parameters()
{
  std_msgs::UInt16 angle_to_servo;
  angle_to_servo.data = 90;	//degrees, final angle
  servoPub.publish(angle_to_servo);
  //Settings  
  tracker_path = ros::package::getPath("tracker_visp");
  opt_config = tracker_path + "/trackers/jenga_tracker_params.xml";
  opt_model = tracker_path + "/model/file_cao.cao";
  opt_init = tracker_path + "/model/jenga_single.init";
  opt_init_pos= tracker_path + "/model/file_init.pos";
  opt_learning_data = tracker_path + "/learning/1/data-learned.bin";
  opt_keypoint_config = tracker_path + "/learning/keypoint_config.xml";
  opt_learn = true;
  opt_auto_init = false;
  opt_pose_init = true;
  learn_position = false;

  // [Realsense camera configuration]
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

  try {
    realsense.open(config);
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
    std::cout << "Check if the Realsense camera is connected..." << std::endl;
  }
  catch (const rs2::error &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
    std::cout << "Check if the Realsense camera is connected..." << std::endl;
  }

  cam_color = realsense.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
  cam_depth = realsense.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithoutDistortion);

  // [Configuration of display]
  width = 640;
  height = 480;
  fps = 30;
  
  I_color.init(height, width);
  I_gray.init(height, width);
  I_depth_gray.init(height, width);
  I_depth_color.init(height, width);
  I_depth_raw.init(height, width);

  _posx = 100; _posy = 50;
  d1.init(I_color, _posx, _posy, "Color stream");
  d2.init(I_depth_color, _posx, _posy + I_gray.getHeight()+10, "Depth stream");
  ILearned.init(I_color.getHeight()*2,I_color.getWidth()*2);
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

}

int visual_servoing::init_matrices() 
{ 
  static const std::string PLANNING_GROUP = "edo";
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  eeTcam = homogeneousTransformation("edo_gripper_link_ee", "camera_color_optical_frame");
  camTtarget = homogeneousTransformation("camera_color_optical_frame", "handeye_target");
  targetTcam = homogeneousTransformation("handeye_target", "camera_color_optical_frame");
  baseTee = homogeneousTransformation("edo_base_link", "edo_gripper_link_ee");
  baseTtarget = homogeneousTransformation("edo_base_link", "handeye_target");
  eeTtarget = homogeneousTransformation("edo_gripper_link_ee", "handeye_target");
  camTee=homogeneousTransformation("edo_gripper_link_ee", "camera_color_optical_frame"); 
  camTbase = homogeneousTransformation("camera_color_optical_frame", "edo_base_link");

  vpTranslationVector t_off; t_off << 0.0, -0.0, -0.25;
  vpRxyzVector rxyz; rxyz << 0, 0, -M_PI_2;
  vpRotationMatrix R_off(rxyz);
  offset.buildFrom(t_off, R_off.inverse());
  
  //base2target = baseTee*eeTtarget; 
  //initialGuestPos = insertPosData(base2target*offset);
  //startingPos.publish(initialGuestPos);
  //ros::Duration(10).sleep();

  // bool initialPose{true};

  // ros::service::waitForService("/InitialGuess");
  // srv.request.initialGuessPose = initialPose;
  // ROS_INFO("Calling the service..");

  // if (client.call(srv)){
  //   if (srv.response.execution.data == true) {
  //     ROS_INFO("Approaching the target");
  //   }
  //   else {
  //     ROS_INFO("Initial guess cannot be provided");
  //     return EXIT_FAILURE;
  //   }
  
  // }

  // else {
  //   ROS_ERROR("Cannot receive response from server");
  // }

  vpThetaUVector cTo_tu = camTtarget.getThetaUVector();

  vpTranslationVector cdto;
  translX = 0.0324;  //the correct one is x = -0.0035;
  translY = 0.06;  //the correct one is y = 0.053;
  translZ = 0.27;   //the correct one is z = 0.134

  cdto.buildFrom(translX, translY, translZ);
  vpRotationMatrix cdRo{1, 0, 0, 
  0, 1, 0, 
  0, 0, 1};

  cdTtarget.buildFrom(cdto, cdRo);

  cdTc = cdTtarget*targetTcam;

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped target;

  target.header.stamp = ros::Time::now();
  target.header.frame_id = "camera_desired";
  target.child_frame_id = "handeye_target";
  target.transform=visp_bridge::toGeometryMsgsTransform(cdTtarget); 

  br.sendTransform(target);
 
}


void visual_servoing::init_servo()
{
  //Task details
  task.setServo(vpServo::EYEINHAND_CAMERA); //control law
  task.setInteractionMatrixType(vpServo::CURRENT); //interaction matrix $\bf L$ is computed from the visual features at the desired position
  
  const vpAdaptiveGain lambda(1.4, 0.6, 30);
  task.setLambda(lambda);

  //s = new vpFeatureTranslation(vpFeatureTranslation::cMo); alternativa con puntatore
  //PBVS
  s.buildFrom(camTtarget);
  s_star.buildFrom(cdTtarget);
  task.addFeature(s, s_star);

  s_tu.buildFrom(cdTc);
  task.addFeature(s_tu, s_tu_star); //, vpFeatureThetaU::selectTUy() | vpFeatureThetaU::selectTUx());

  error = 5;
  threshold = 0.00002;
  block_axis = false;
  vitesse = 0.0002;
  rapport = 0;

}



void visual_servoing::learning_process() 
{
  
  opt_learn = true;
  opt_auto_init = false; 
  opt_pose_init = true;
  learn_position = false;

  // [Map of images to use tracker with depth]
  depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformations;
  std::map<std::string, const vpImage<vpRGBa> *> mapOfImages;
  std::map<std::string, std::string> mapOfInitFiles;
  std::map<std::string, std::string> mapOfInitPoses;
  std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
  std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraPoses;
  std::vector<vpColVector> pointcloud;

  // Set keypoints type from code if not from file
  std::string detectorName = "FAST";
  std::string extractorName = "ORB";
  std::string matcherName = "BruteForce-Hamming";
  
  mapOfCameraTransformations["Camera2"] = depth_M_color;
  mapOfImages["Camera1"] = &I_color;
  mapOfImages["Camera2"] = &I_depth_color;
  mapOfInitFiles["Camera1"] = opt_init;
  mapOfInitPoses["Camera1"] = opt_init_pos;

  // Define tracker types
  
  trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
  trackerTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);

  tracker = new vpMbGenericTracker(trackerTypes);
  
  tracker->loadConfigFile(opt_config, opt_config);
  //tracker->loadModel(opt_model, opt_model);
  tracker->setCameraTransformationMatrix(mapOfCameraTransformations);
  tracker->setCameraParameters(cam_color, cam_depth);

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

   
  if (opt_auto_init) {
    // Preparation for automatic initialization
    if (!vpIoTools::checkFilename(opt_learning_data)) {
      std::cout << "Cannot enable auto detection. Learning file \"" << opt_learning_data << "\" doesn't exist" << std::endl;
      //return EXIT_FAILURE;
    }
    keypoint.loadLearningData(opt_learning_data, true);
    vpImageConvert::convert(I_color, I_gray);
    keypoint.createImageMatching(I_gray, IMatching);
    d3.setDownScalingFactor(vpDisplay::SCALE_AUTO);
    d3.init(IMatching, _posx+I_gray.getWidth()*2+10, _posy, "Detection-from-learned");
  } 

  else if (opt_yolact_init) {
    sensor_msgs::Image sensor_image = visp_bridge::toSensorMsgsImage(I_color);
	  sensor_msgs::CameraInfo sensor_camInfo = visp_bridge::toSensorMsgsCameraInfo(cam_color,width,height);
    // Send image and cam par to service, where Python node responds with cao_file and pose
    ROS_INFO("Subscribing to service...");
    ros::service::waitForService("/Pose_cao_initializer");
    srv.request.image = sensor_image;
    srv.request.camInfo = sensor_camInfo;
    ROS_INFO("Starting call to service..");
    if (client.call(srv))
    {
      ROS_INFO("Response from service..");
      opt_model = srv.response.caoFilePath.data;
      geometry_msgs::Transform initPose = srv.response.initPose;
      cMo = visp_bridge::toVispHomogeneousMatrix(initPose);
      tracker->loadModel(opt_model, opt_model);
      mapOfCameraPoses["Camera1"] = cMo;
      mapOfCameraPoses["Camera2"] = depth_M_color *cMo;
      tracker->initFromPose(mapOfImages,mapOfCameraPoses);
    }

    else{
      ROS_ERROR("Cannot receive response from server");
    }
  }

  else {
    if (opt_pose_init){
      // // Initialize from pose file .pos
      tracker->initFromPose(mapOfImages,mapOfInitPoses);
    }
    else {
      // // Initialize with clicks from .init
      tracker->initClick(mapOfImages, mapOfInitFiles, true);
    }
  }

  // Display and error
  tracker->setDisplayFeatures(opt_display_features);
  tracker->setProjectionErrorComputation(true);
  tracker->setProjectionErrorDisplay(opt_display_projection_error);

  // Force good moving edges ratio threshold
  tracker->setGoodMovingEdgesRatioThreshold(opt_setGoodME_thresh);  //default=0.4

  // Remove faces with name "occluded" from the tracking, a priori
  tracker->setUseEdgeTracking("occluded",false);
  tracker->setUseKltTracking("occluded",false);
  tracker->setUseDepthDenseTracking("occluded",false);

  // Rotating base 
  // Acquire images, visualize and update tracker input data
  realsense.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, &pointcloud, NULL, NULL);
  vpImageConvert::convert(I_color, I_gray);


  // Before the loop
  bool run_auto_init = false;
  if (opt_auto_init) {
    run_auto_init = true;
  }

  std::vector<double> times_vec;
  std::vector<double> train_t_vec;
  std::vector<double> detect_t_vec;

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
    while (!quit && node_handle.ok() ) {
      
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
          tracker->initFromPose(mapOfImages, mapOfCameraPoses);
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
        tracker->initClick(mapOfImages, mapOfInitFiles, true);
      }

      // Run the tracker
      try {
        if (run_auto_init) {
          // Turn display features off just after auto init to not display wrong moving-edge if the tracker fails
          tracker->setDisplayFeatures(false);
          run_auto_init = false;
        }
        tracker->track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
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
      
      //eeTc.buildFrom(t_cam, q_cam);
      //geometry_msgs::TransformStamped camera_RF = toMoveit(eeTc, "world", "object");
      //br.sendTransform(camera_RF);
      
      //geometry_msgs::TransformStamped camera_1 = toMoveit(wTc1, "world", "camera_1");
      //br.sendTransform(camera_1);

      // Get and publish object pose
      cMo = tracker->getPose();
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
      if (tracker->getTrackerType() & vpMbGenericTracker::EDGE_TRACKER) {
        // Check tracking errors
        proj_error = tracker->getProjectionError();
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
          tracker->setDisplayFeatures(opt_display_features);
          {
            std::stringstream ss;
            ss << "Nb features: " << tracker->getError().size();
            vpDisplay::displayText(I_color, I_color.getHeight() - 50, 20, ss.str(), vpColor::red);
          }
          {
            std::stringstream ss;
            ss << "Features: edges " << tracker->getNbFeaturesEdge()
              << ", klt " << tracker->getNbFeaturesKlt()
              << ", depth " << tracker->getNbFeaturesDepthDense();
            vpDisplay::displayText(I_color, I_color.getHeight() - 30, 20, ss.str(), vpColor::red);
          }
        }

        // Display model
        if (opt_display_model)
          tracker->display(I_color, I_depth_color, cMo, depth_M_color*cMo, cam_color, cam_depth, vpColor::green, 1);
        
        // Display frames
        vpDisplay::displayFrame(I_color, cMo, cam_color, 0.05, vpColor::none, 3);
        vpDisplay::displayFrame(I_depth_color, depth_M_color*cMo, cam_depth, 0.05, vpColor::none, 3);

      }

      vpMouseButton::vpMouseButtonType button;
      
      vpDisplay::flush(I_color);


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


      if (learn_position) {
        double train_t = vpTime::measureTimeMs();

        learn_cpt ++;

        // Detect keypoints on the current image
        std::vector<cv::KeyPoint> trainKeyPoints;
        keypoint.detect(I_color, trainKeyPoints);

        // Keep only keypoints on the object
        std::vector<vpPolygon> polygons;
        std::vector<std::vector<vpPoint> > roisPt;
        std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker->getPolygonFaces();
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
    d3.close(ILearned);

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
  
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
  }





}


void visual_servoing::detection_process() 
{
  
  opt_learn = false;
  opt_auto_init = true; 
  opt_yolact_init = false;
  keypoint.loadLearningData(opt_learning_data, true);
  vpImageConvert::convert(I_color,I_gray);
  keypoint.createImageMatching(I_gray, IMatching);
  d3.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  d3.init(IMatching, _posx+I_gray.getWidth()*2+10, _posy, "Detection-from-learned");


  // [Map of images to use tracker with depth]
  depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
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
  /*
  trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
  trackerTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);

  tracker = new vpMbGenericTracker(trackerTypes);
  
  tracker->loadConfigFile(opt_config, opt_config);
  tracker->loadModel(opt_model, opt_model);
  tracker->setCameraTransformationMatrix(mapOfCameraTransformations);
  tracker->setCameraParameters(cam_color, cam_depth);
  */

  // Set keypoints type from code if not from file
  std::string detectorName = "FAST";
  std::string extractorName = "ORB";
  std::string matcherName = "BruteForce-Hamming";
  

  // Load auto learn data, or initialize with clicks
  if (opt_auto_init) {
    // Preparation for automatic initialization
    if (!vpIoTools::checkFilename(opt_learning_data)) {
      std::cout << "Cannot enable auto detection. Learning file \"" << opt_learning_data << "\" doesn't exist" << std::endl;
      //return EXIT_FAILURE;
    }
    vpImageConvert::convert(I_color, I_gray);

    d3.setDownScalingFactor(vpDisplay::SCALE_AUTO);
    d3.init(IMatching, _posx+I_gray.getWidth()*2+10, _posy, "Detection-from-learned");
  } 

  else if (opt_yolact_init) {
    sensor_msgs::Image sensor_image = visp_bridge::toSensorMsgsImage(I_color);
	  sensor_msgs::CameraInfo sensor_camInfo = visp_bridge::toSensorMsgsCameraInfo(cam_color,width,height);
    // Send image and cam par to service, where Python node responds with cao_file and pose
    ROS_INFO("Subscribing to service...");
    ros::service::waitForService("/Pose_cao_initializer");
    srv.request.image = sensor_image;
    srv.request.camInfo = sensor_camInfo;
    ROS_INFO("Starting call to service..");
    if (client.call(srv))
    {
      ROS_INFO("Response from service..");
      opt_model=srv.response.caoFilePath.data;
      geometry_msgs::Transform initPose=srv.response.initPose;
      cMo=visp_bridge::toVispHomogeneousMatrix(initPose);
      tracker->loadModel(opt_model, opt_model);
      mapOfCameraPoses["Camera1"] = cMo;
      mapOfCameraPoses["Camera2"] = depth_M_color *cMo;
      tracker->initFromPose(mapOfImages,mapOfCameraPoses);
    }
    else{
      ROS_ERROR("Cannot receive response from server");
    }
  }

  else {
    if (opt_pose_init){
      // // Initialize from pose file .pos
      tracker->initFromPose(mapOfImages,mapOfInitPoses);
    }
    else {
      // // Initialize with clicks from .init
      tracker->initClick(mapOfImages, mapOfInitFiles, true);
    }
  }

  // Display and error
  tracker->setDisplayFeatures(opt_display_features);
  tracker->setProjectionErrorComputation(true);
  tracker->setProjectionErrorDisplay(opt_display_projection_error);

  // Force good moving edges ratio threshold
  tracker->setGoodMovingEdgesRatioThreshold(opt_setGoodME_thresh);  //default=0.4

  // Remove faces with name "occluded" from the tracking, a priori
  tracker->setUseEdgeTracking("occluded",false);
  tracker->setUseKltTracking("occluded",false);
  tracker->setUseDepthDenseTracking("occluded",false);
  
  // Before the loop
  bool run_auto_init = false;
  if (opt_auto_init) {
    run_auto_init = true;
  }

  std::vector<double> times_vec;
  std::vector<double> train_t_vec;
  std::vector<double> detect_t_vec;

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

    init_matrices();
    init_servo();

    
    //! [LOOP]
    //! -----------------------------------------------------------------------------------------------
    while (!quit && node_handle.ok() ) {
      
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
          tracker->initFromPose(mapOfImages, mapOfCameraPoses);
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
        tracker->initClick(mapOfImages, mapOfInitFiles, true);
      }

      // Run the tracker
      try {
        if (run_auto_init) {
          // Turn display features off just after auto init to not display wrong moving-edge if the tracker fails
          tracker->setDisplayFeatures(false);
          run_auto_init = false;
        }
        tracker->track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
      } catch (const vpException &e) {
        std::cout << "Tracker exception: " << e.getStringMessage() << std::endl;
        tracking_failed = true;
        if (opt_auto_init) {
          //std::cout << "Tracker needs to restart (tracking exception)" << std::endl;
          run_auto_init = true;
        }
      }


      //Rotating base
      if (!rotated && !tracking_failed) {

      // Run auto initialization from learned data
        if ((opt_auto_init && keypoint.matchPoint(I_gray, cam_color, cMo)) || opt_yolact_init) {

        // SERVO SEND ANGLE
          vpThetaUVector cTo_tu = cMo.getThetaUVector();
          std_msgs::UInt16 angle_to_servo;
          // std::cout << "Theta: \n" << angle << std::endl;
          if (cTo_tu[1]<0){	//radians, "right face seen from camera"
            angle_to_servo.data = 130;	//degrees, final angle
            servoPub.publish(angle_to_servo);
          }
          else {	//radians, "left face seen from camera"
            angle_to_servo.data = 50;	//degrees, final angle
            servoPub.publish(angle_to_servo);
          } 
        }
        rotated = true;
      }



      // Define camera's RF
      
      vpTranslationVector t_cam; t_cam << 0.54, 0.08, 0.4;
      vpQuaternionVector q_cam, q_unit; q_cam << 0, 0, 0.7071068, 0.7071068;
      
      //eeTc.buildFrom(t_cam, q_cam);
      //geometry_msgs::TransformStamped camera_RF = toMoveit(eeTc, "world", "object");
      //br.sendTransform(camera_RF);
      
      //geometry_msgs::TransformStamped camera_1 = toMoveit(wTc1, "world", "camera_1");
      //br.sendTransform(camera_1);

      // Get and publish object pose
      cMo = tracker->getPose();
      //cTo = cMo;
      //q_unit << 0, 0, 0, 1;
      //cTo.insert(q_unit); //only for simulation
      geometry_msgs::TransformStamped pose_target = toMoveit(cMo, "camera_color_optical_frame" , "handeye_target");
      br.sendTransform(pose_target);

      
      geometry_msgs::Pose cTo_P;
      cTo_P = visp_bridge::toGeometryMsgsPose(cMo); 
      trackerEstimation.publish(cTo_P);

      estimationCallback(cTo_P);
      

      // Check tracking errors
      double proj_error = 0;
      if (tracker->getTrackerType() & vpMbGenericTracker::EDGE_TRACKER) {
        // Check tracking errors
        proj_error = tracker->getProjectionError();
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
          tracker->setDisplayFeatures(opt_display_features);
          {
            std::stringstream ss;
            ss << "Nb features: " << tracker->getError().size();
            vpDisplay::displayText(I_color, I_color.getHeight() - 50, 20, ss.str(), vpColor::red);
          }
          {
            std::stringstream ss;
            ss << "Features: edges " << tracker->getNbFeaturesEdge()
              << ", klt " << tracker->getNbFeaturesKlt()
              << ", depth " << tracker->getNbFeaturesDepthDense();
            vpDisplay::displayText(I_color, I_color.getHeight() - 30, 20, ss.str(), vpColor::red);
          }
        }

        // Display model
        if (opt_display_model)
          tracker->display(I_color, I_depth_color, cMo, depth_M_color*cMo, cam_color, cam_depth, vpColor::green, 1);
        
        // Display frames
        vpDisplay::displayFrame(I_color, cMo, cam_color, 0.05, vpColor::none, 3);
        vpDisplay::displayFrame(I_depth_color, depth_M_color*cMo, cam_depth, 0.05, vpColor::none, 3);

      }

      vpMouseButton::vpMouseButtonType button;
      
      vpDisplay::flush(I_color);


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


      if (learn_position) {
        double train_t = vpTime::measureTimeMs();

        learn_cpt ++;
        
        // Detect keypoints on the current image
        std::vector<cv::KeyPoint> trainKeyPoints;
        keypoint.detect(I_color, trainKeyPoints);

        // Keep only keypoints on the object
        std::vector<vpPolygon> polygons;
        std::vector<std::vector<vpPoint> > roisPt;
        std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker->getPolygonFaces();
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
    std_msgs::UInt16 angle_to_servo;
    angle_to_servo.data = 90;	//degrees, final angle
    servoPub.publish(angle_to_servo);

    // Terminate learning phase, save all on exit
    if (opt_learn) {
      
      if (learn_cpt>0){
        std::cout << "Save learning from " << learn_cpt << " images in file: " << opt_learning_data << std::endl;
        keypoint.saveLearningData(opt_learning_data, true, true);
      }

    
    }

    if (!times_vec.empty()) {
      std_msgs::UInt16 angle_to_servo;
      angle_to_servo.data = 90;	//degrees, final angle
      servoPub.publish(angle_to_servo);
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
  
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
  }



}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "full_pbvs");
  ros::NodeHandle nh;   
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate loop_rate(20); 

  //visual_servoing learning(nh);

  ros::spinOnce();
  loop_rate.sleep();
  
  return 0;
}