/* This file is part of RGBDSLAM.
 * 
 * RGBDSLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * RGBDSLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with RGBDSLAM.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "parameter_server.h"

//using namespace std;

ParameterServer* ParameterServer::_instance = NULL;

void ParameterServer::defaultConfig() {
  double dInf = std::numeric_limits<double>::infinity();
  // Input data settings
  addOption("topic_image_mono",              std::string("/camera/rgb/image_color"),    "Color or grayscale image ros topic");
  addOption("camera_info_topic",             std::string("/camera/rgb/camera_info"),    "Required for backprojection if no pointcloud topic is given");
  addOption("topic_image_depth",             std::string("/camera/depth_registered/sw_registered/image_rect_raw"),        "Depth image ros topic");
  addOption("topic_points",                  std::string(""),                           "If omitted, xyz will be computed from depth image. ");
  addOption("wide_topic",                    std::string(""),                           "Topics for stereo cam, e.g. /wide_stereo/left/image_mono");
  addOption("wide_cloud_topic",              std::string(""),                           "Topics for stereo cam e.g. /wide_stereo/points2");
  addOption("bagfile_name",                  std::string(""),                           "Read data from a bagfile, make sure to enter the right topics above");
  addOption("subscriber_queue_size",         static_cast<int> (3),                      "Cache incoming data (carefully, RGB-D Clouds are 10MB each)");
  addOption("drop_async_frames",             static_cast<bool> (false),                 "Check timestamps of depth and visual image, reject if not in sync ");
  addOption("depth_scaling_factor",          static_cast<double> (1.0),                 "Some kinects have a wrongly scaled depth");
  addOption("data_skip_step",                static_cast<int> (1),                      "Skip every n-th frame completely  ");
  addOption("cloud_creation_skip_step",      static_cast<int> (2),                      "Downsampling factor (rows and colums, so size reduction is quadratic) for the point cloud. Only active if cloud is computed (i.e. \"topic_points\" is empty. This value multiplies to emm__skip_step and visualization_skip_step.");
  addOption("create_cloud_every_nth_node",   static_cast<int> (1),                      "Create a point cloud only for every nth frame");
  addOption("maximum_depth",                 static_cast<double> (dInf),                "Clip far points when reconstructing the cloud. In meter.");
  addOption("minimum_depth",                 static_cast<double> (0.1),                 "Clip near points when reconstructing the cloud. In meter.");
  addOption("encoding_bgr",                  static_cast<bool> (true),                   "Whether the color image encoding is bgr (fuerte openni_launch) as opposed to rgb (electric openni_camera)");
  //Camera settings (Internal calibration is not consistent though. It's best, the input is calibrated.)
  addOption("depth_camera_fx",               static_cast<double> (0.0),                 "Focal length w.r.t. horizontal pixel size. Use zero to use values from CameraInfo");
  addOption("depth_camera_fy",               static_cast<double> (0.0),                 "Focal length w.r.t. vertical pixel size. Use zero to use values from CameraInfo");
  addOption("depth_camera_cx",               static_cast<double> (0.0),                 "Horizontal image center. Use zero to use values from CameraInfo");
  addOption("depth_camera_cy",               static_cast<double> (0.0),                 "Vertical image center. Use zero to use values from CameraInfo");
  addOption("sigma_depth",                   static_cast<double> (0.01),                "Factor c for the standard deviation of depth measurements: sigma_Z = c * depth * depth. Khoshelham 2012 (0.001425) seems to be a bit overconfident.");

  // Output data settings
  addOption("store_pointclouds",             static_cast<bool> (true),                  "If the point clouds are not needed online, setting this to false saves lots of memory ");
  addOption("individual_cloud_out_topic",    std::string("/rgbdslam/batch_clouds"),     "Use this topic when sending the individual clouds with the computed transforms, e.g. for octomap_server");
  addOption("aggregate_cloud_out_topic",     std::string("/rgbdslam/aggregate_clouds"), "Use this topic when sending the all points in one big registered cloud");
  addOption("online_cloud_out_topic",        std::string("/rgbdslam/online_clouds"),    "Use this topic to get the latest cloud.");
  addOption("send_clouds_rate",              static_cast<double> (5),                   "When sending the point clouds (e.g. to RVIZ or Octomap Server) limit sending to this many clouds per second");
  addOption("publisher_queue_size",          static_cast<int> (5),                      "ROS standard parameter for all publishers");
  // Octomap data settings
  addOption("octomap_resolution",            static_cast<double> (0.05),                "Minimal voxel size of the OctoMap, when saved directly (not when used in conjunction with octomap_server)");
  addOption("octomap_autosave_step",         static_cast<int> (50),                     "Automatically save the octomap repeatedly after insertion of this many clouds");
  addOption("octomap_clear_after_save",      static_cast<bool> (false),                  "Clear out octomap after (final) saving.");
  addOption("octomap_clear_raycasted_clouds",static_cast<bool> (false),                  "Clear out point clouds after raycasting.");
  addOption("octomap_occupancy_threshold",   static_cast<double> (0.5),                 "Occupancy threshold for binary OctoMap");
  addOption("octomap_clamping_max",          static_cast<double> (0.999),               "Maximum value for clamping of occupancy threshold in voxels");
  addOption("octomap_clamping_min",          static_cast<double> (0.001),                 "Minimum value for clamping of occupancy threshold in voxels");
  addOption("octomap_prob_hit",              static_cast<double> (0.9),                 "Octomap sensor model: Probability value for hit.");
  addOption("octomap_prob_miss",             static_cast<double> (0.4),                 "Octomap sensor model: Probability value for miss.");
  addOption("octomap_online_creation",       static_cast<bool> (false),                 "Create the octomap during mapping. If using this, every node will be rendered to the octomap directly after they have been added to the graph (and graph optimization, if not skipped b/c of optimizer_skip_step).");
  addOption("screencast_path_prefix",        std::string(""),                           "If set: capture frames for a screencast with this path as filename-prefix.");
  addOption("transform_individual_clouds",   static_cast<bool> (false),                 "If set: Transform individually saved pcds into common coordinate frame.");
  addOption("compress_output_bagfile",       static_cast<bool> (true),                  "Whether to enable bz2 compression when saving bagfiles");
  addOption("occupancy_filter_threshold",    static_cast<double> (0.9),                 "Remove points located at voxels with occupancy probability below this.");

  // TF information settings 
  addOption("fixed_frame_name",              std::string("/map"),                       "The computed camera transforms are with respect to this frame. It is set to the identity for the first frame processed or, if ground truth is available, to the ground truth of the first frame");
  addOption("odom_frame_name",               std::string(""),                           "A fixed frame estimation from somewhere else (e.g. odometry, laser-based mapping). Doesn't need to correspond to the pose of the fixed_frame_name");
  addOption("odom_target_frame_name",        std::string(""),                           "If given, try to get the odometry transformation from odom_frame_name to this frame instead of the frame of the points. Meant to compute the offset between robot base and sensor online.");
  addOption("ground_truth_frame_name",       std::string(""),                           "Use empty string if no ground truth tf frame available");
  addOption("base_frame_name",               std::string("/openni_rgb_optical_frame"),               "If the camera is articulated use robot base");
  addOption("fixed_camera",                  static_cast<bool> (true),                  "Is camera fixed relative to base?");

  // Visual Features, to activate GPU-based features see CMakeLists.txt 
  addOption("feature_detector_type",         std::string("ORB"),                        "SIFTGPU, SURF, SIFT or ORB, or variants like GridDynamicORB or DynamicSURF (however, not GridXXX only, and not in combination with SIFTGPU)");
  addOption("feature_extractor_type",        std::string("ORB"),                        "SIFTGPU, SURF, SIFT or ORB");
  addOption("matcher_type",                  std::string("FLANN"),                      "SIFTGPU (matching on the gpu) or FLANN or BRUTEFORCE. FLANN is fast for SURF. For ORB FLANN is slower than Bruteforce for few keypoints (less than ~3000).");
  addOption("max_keypoints",                 static_cast<int> (600),                    "Extract no more than this many keypoints ");
  addOption("min_keypoints",                 static_cast<int> (000),                    "Deprecated: Extract no less than this many keypoints ");
  addOption("min_matches",                   static_cast<int> (20),                     "Don't try RANSAC if less than this many matches (if using SiftGPU and GLSL you should use max. 60 matches)");
  addOption("max_matches",                   static_cast<int> (300),                    "Reduce the feature matches to the best n, speeding up ransac (but not feature matching itself)");
  addOption("detector_grid_resolution",      static_cast<int> (3),                      "If >1, split image into x by x subimages (overlapping b/c of keypoint size) and detect keypoints in parallel. Good to spread ORB keypoints over the image and speed up the slow SURF detection.");
  addOption("sufficient_matches",            static_cast<int> (1e9),                    "Extract no less than this many only honored by the adjustable SURF and FAST features");
  addOption("adjuster_max_iterations",       static_cast<int> (5),                      "If outside of bounds for max_kp and min_kp, retry this many times with adapted threshold");
  addOption("use_feature_min_depth",         static_cast<bool>(false),                  "Consider the nearest point in the neighborhood of the feature as its depth, as it will dominate the motion");
  addOption("use_feature_mask",              static_cast<bool>(false),                  "Whether to extract features without depth");
  addOption("use_root_sift",                 static_cast<bool>(true),                   "Whether to use euclidean distance or Hellman kernel for feature comparison");

  // Frontend settings 
  addOption("max_translation_meter",         static_cast<double> (1e10),                "Sanity check for smooth motion.");
  addOption("max_rotation_degree",           static_cast<double> (360),                    "Sanity check for smooth motion.");
  addOption("min_translation_meter",         static_cast<double> (0.0),                 "Frames with motion less than this, will be omitted ");
  addOption("min_rotation_degree",           static_cast<double> (0.0),                 "Frames with motion less than this, will be omitted ");
  addOption("max_dist_for_inliers",          static_cast<double> (3),                   "Mahalanobis distance for matches to be considered inliers by ransac");
  addOption("ransac_iterations",             static_cast<int> (200),                    "Number of iterations for registration");
  addOption("ransac_termination_inlier_pct", static_cast<double> (60.0),                "Percentage of matches that need to be inliers to succesfully terminate ransac before the 'ransac_iterations' have been reached");
  addOption("g2o_transformation_refinement", static_cast<int> (0),                      "Use g2o to refine the ransac result for that many iterations, i.e. optimize the Mahalanobis distance in a final step. Use zero to disable.");
  addOption("max_connections",               static_cast<int> (-1),                     "Stop frame comparisons after this many succesfully found spation relations. Negative value: No limit.");
  addOption("geodesic_depth",                static_cast<int> (3),                      "For comparisons with neighbors, consider those with a graph distance (hop count) equal or below this value as neighbors of the direct predecessor");
  addOption("predecessor_candidates",        static_cast<int> (4),                      "Compare Features to this many direct sequential predecessors");
  addOption("neighbor_candidates",           static_cast<int> (4),                      "Compare Features to this many graph neighbours. Sample from the candidates");
  addOption("min_sampled_candidates",        static_cast<int> (4),                      "Compare Features to this many uniformly sampled nodes for corrspondences ");
  addOption("use_icp",                       static_cast<bool> (false),                 "Activate ICP Fallback. Ignored if ICP is not compiled in (see top of CMakeLists.txt) ");
  addOption("icp_method",                    std::string("icp"),                        "gicp, icp or icp_nl");
  addOption("gicp_max_cloud_size",           static_cast<int> (10000),                  "Subsample for increased speed");
  addOption("emm__skip_step",                static_cast<int> (8),                      "When evaluating the transformation, subsample rows and cols with this stepping");
  addOption("emm__mark_outliers",            static_cast<bool> (false),                 "Mark outliers in the observation likelihood evaluation with colors. Red: point would have blocked the view of an earlier observation. Cyan: An earlier observation should have blocked the view to this point");
  addOption("observability_threshold",       static_cast<double> (-0.6),                "What fraction of the aligned points are required to be in observable position (i.e. don't contradict the sensor physics)");
  addOption("allow_features_without_depth",  static_cast<bool> (false),                 "Keep matches without depth (currently has no benefit)");

  //Backend
  addOption("pose_relative_to",              std::string("first"),                      "This option allows to choose which frame(s) should be set fixed during optimization: first, previous, inaffected, largest_loop. The latter sets all frames as fixed to which no transformation has been found.");
  addOption("optimizer_iterations",          static_cast<double> (0.01),                "Maximum of iterations. If between 0 and 1, optimizer stops after improvement is less than the given fraction (default: 1%).");
  addOption("optimizer_skip_step",           static_cast<int> (1),                      "Optimize every n-th frame. Set negative for offline operation ");
  addOption("optimize_landmarks",            static_cast<bool> (false),                 "Consider the features as landmarks in optimization. Otherwise optimize camera pose graph only");
  addOption("concurrent_optimization",       static_cast<bool> (true),                  "Do graph optimization in a seperate thread");
  addOption("backend_solver",                std::string("pcg"),                        "Which solver to use in g2o for matrix inversion: 'csparse' , 'cholmod' or 'pcg'");

  // Robot odometry options
  addOption("use_robot_odom",                static_cast<bool> (false),                 "In addition to frame-to-frame transformation use odometry information in the graph");
  addOption("use_robot_odom_only",           static_cast<bool> (false),                 "In addition to frame-to-frame transformation use odometry information in the graph");
  addOption("constrain_2d",                  static_cast<bool> (false),                 "Constrain camera motion to 2d plus heading");
  addOption("use_odom_for_prediction",       static_cast<bool> (false),                 "Use odometry information to predict feature locations. Do not use Kd-Tree anymore.");
  addOption("odometry_tpc",                  std::string("/odom"),                      "Robot odometry topic. Not required if odometry comes via /tf. Only the pose of the odometry is used.");
  addOption("odometry_information_factor",   static_cast<double> (1e4),                 "Diagonal coefficients of the information matrix of the odometry edges. Default: Std Dev of 0.01m and 0.01rad");

  // Visualization Settings 
  addOption("use_glwidget",                  static_cast<bool> (true),                  "3D view");
  addOption("use_gui",                       static_cast<bool> (true),                  "GUI vs Headless Mode");
  addOption("show_2d_display",               static_cast<bool> (true),                  "show or hide 2D view initially");
  addOption("glwidget_without_clouds",       static_cast<bool> (false),                 "3D view should only display the graph");
  addOption("visualize_mono_depth_overlay",  static_cast<bool> (false),                 "Show Depth and Monochrome image as overlay in featureflow");
  addOption("visualization_skip_step",       static_cast<int> (1),                      "Draw only every nth pointcloud row and line, high values require higher squared_meshing_threshold ");
  addOption("visualize_keyframes_only",      static_cast<bool> (false),                 "Do not render point cloud of non-keyframes.");
  addOption("fast_rendering_step",           static_cast<int> (1),                      "Draw only every nth pointcloud during user interaction");
  addOption("octomap_display_level",         static_cast<int> (16),                     "Show this level of OctoMap octree");
  addOption("gl_point_size",                 static_cast<double> (1.0),                 "Point size, when not triangulating. See documentation of GL_POINT_SIZE.");
  addOption("gl_grid_size_xy",               static_cast<int> (0),                      "Grid size in the xy plane (sidelength in number of cell). Zero disables. Note that this is in the coordinate system of the point cloud");
  addOption("gl_grid_size_xz",               static_cast<int> (20),                     "Grid size in the xz plane (sidelength in number of cell). Zero disables. Note that this is in the coordinate system of the point cloud");
  addOption("gl_grid_size_yz",               static_cast<int> (0),                      "Grid size in the yz plane (sidelength in number of cell). Zero disables. Note that this is in the coordinate system of the point cloud");
  addOption("gl_cell_size",                  static_cast<double> (1.0),                 "Grid cell size in meter.");
  addOption("squared_meshing_threshold",     static_cast<double> (0.0009),              "Don't triangulate over depth jumps. Should be increased with increasing visualization_skip_step");
  addOption("show_axis",                     static_cast<bool> (true),                  "Do/don't visualize the pose graph in glwidget");
  addOption("scalable_2d_display",           static_cast<bool> (false),                 "Whether the input images are expanded. Consumes CPU time");
  addOption("cloud_display_type",            static_cast<std::string>("POINTS"),        "Drastically affects rendering time. GL_xxx type of compiled list GL_TRIANGLE_STRIP (fastest processing of new clouds), GL_POINTS (fastest display) GL_TRIANGLES (no good), or ELLIPSOIDS (very slow, but visualizes standard deviation)");

  // Misc 
  addOption("start_paused",                  static_cast<bool> (false),                  "Whether to directly start mapping with the first input image, or to wait for the user to start manually");
  addOption("batch_processing",              static_cast<bool> (false),                 "Store results and close after bagfile has been processed");
  addOption("concurrent_node_construction",  static_cast<bool> (true),                  "Detect+extract features for new frame, while current frame is inserted into graph ");
  addOption("concurrent_edge_construction",  static_cast<bool> (true),                  "Compare current frame to many predecessors in parallel. Note that SIFTGPU matcher and GICP are mutex'ed for thread-safety");
  addOption("concurrent_io",                 static_cast<bool> (true),                  "Whether saving/sending should be done in background threads.");
  addOption("voxelfilter_size",              static_cast<double> (-1.0),                "In meter voxefilter displayed and stored pointclouds, useful to reduce the time for, e.g., octomap generation. Set negative to disable");
  addOption("nn_distance_ratio",             static_cast<double> (0.95),                "Feature correspondence is valid if distance to nearest neighbour is smaller than this parameter times the distance to the 2nd neighbour. This needs to be 0.9-1.0 for ORB and SIFTGPU w/ FLANN, since SIFTGPU Features are normalized and ORB become ambiguous for blurry images. For SURF and SIFT (CPU) use 0.5-0.8");
  addOption("keep_all_nodes",                static_cast<bool> (false),                 "Keep all nodes with 'no motion' assumption");
  addOption("keep_good_nodes",               static_cast<bool> (false),                 "Keep nodes without transformation estimation but enough features (according to min_matches) with 'no motion' assumption. These are not rendered in visualization.");
  addOption("clear_non_keyframes",           static_cast<bool> (false),                 "Remove the net data of nodes when it becomes clear that they will not be used as keyframe. However, this makes matching against them impossible.");
  addOption("min_time_reported",             static_cast<double> (-1.0),                "For easy profiling. Negative: nothing should be reported");
  addOption("preserve_raster_on_save",       static_cast<bool> (false),                 "Filter NaNs when saving clouds, destroying the image raster");
  addOption("skip_first_n_frames",           static_cast<int> (0),                      "Useful to skip start of a bagfile");
  addOption("segment_to_optimize",           static_cast<int> (-1),                     "If segment information is available, optimize with higher weights on this segment (set negative to disable)");
  addOption("send_clouds_delay",             static_cast<double> (-10.0),                "If set, wait with sending clouds until they are x seconds old (uses the clouds timestamp).");
  addOption("save_octomap_delay",            static_cast<double> (10.0),                "If set, wait with sending clouds until they are x seconds old (uses the clouds timestamp).");
  //Debug
  addOption("show_cloud_with_id",            static_cast<int> (-1),                     "Show only one cloud (namely that perceived at the respective node id");
  addOption("use_error_shortcut",            static_cast<bool> (true),                  "Compute isotropic bounds first in inlier determination");
}


ParameterServer::ParameterServer() {
  pre = ros::this_node::getName();
  pre += "/config/";

  defaultConfig();
  getValues();
}

ParameterServer* ParameterServer::instance() {
  if (_instance == NULL) {
    _instance = new ParameterServer();
  }
  return _instance;
}

void ParameterServer::addOption(std::string name, boost::any value, std::string description){
    config[name] = value;
    descriptions[name] = description;
}

/* Used by GUI */
std::string ParameterServer::getDescription(std::string param_name) {
  return descriptions[param_name];
}

void ParameterServer::getValues() {
  std::map<std::string, boost::any>::const_iterator itr;
  for (itr = config.begin(); itr != config.end(); ++itr) {
    std::string name = itr->first;
    if (itr->second.type() == typeid(std::string)) {
      config[name] = getFromParameterServer<std::string> (pre + name,
          boost::any_cast<std::string>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<std::string>(itr->second));
    } else if (itr->second.type() == typeid(int)) {
      config[name] = getFromParameterServer<int> (pre + name,
          boost::any_cast<int>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<int>(itr->second));
    } else if (itr->second.type() == typeid(double)) {
      config[name] = getFromParameterServer<double> (pre + name,
          boost::any_cast<double>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<double>(itr->second));
    } else if (itr->second.type() == typeid(bool)) {
      config[name] = getFromParameterServer<bool> (pre + name,
          boost::any_cast<bool>(itr->second));
      ROS_DEBUG_STREAM("Value for " << name << ":             " << boost::any_cast<bool>(itr->second));
    }
  }
  checkValues();
}

void ParameterServer::checkValues() {
    if (get<std::string>("matcher_type").compare("SIFTGPU") == 0
            && get<bool>("concurrent_node_construction") == true) {
        config["concurrent_node_construction"] = static_cast<bool>(false);
        ROS_WARN("Cannot use concurrent node construction with SiftGPU matcher! 'concurrent_node_construction' was set to false. Everything should work fine, but the CPU-threading won't happen (because you are using the GPU instead).");
    }

    if (get<double>("voxelfilter_size") > 0 && get<double>("observability_threshold") > 0) {
        ROS_ERROR("You cannot use the voxelfilter (param: voxelfilter_size) in combination with the environment measurement model (param: observability_threshold)");
    }
    if (get<std::string>("matcher_type").compare("SIFTGPU") == 0
            && get<bool>("concurrent_edge_construction") == true) {
        config["concurrent_edge_construction"] = static_cast<bool>(false);
        ROS_WARN("Cannot use concurrent edge construction with SiftGPU matcher! 'concurrent_edge_construction' was set to false. Everything should work fine, but the CPU-threading won't happen (because you are using the GPU instead).");
    }
    if (get<double>("max_translation_meter") <= 0){
      double dInf = std::numeric_limits<double>::infinity();
      set("max_translation_meter", dInf);
    }
    if (get<double>("max_rotation_degree") <= 0){
      double dInf = std::numeric_limits<double>::infinity();
      set("max_rotation_degree", dInf);
    }
}
