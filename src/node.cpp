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


#include "node.h"
#include "features.h"
#include "transformation_estimation_euclidean.h"
#include "transformation_estimation.h"
#include <cmath>
#include "scoped_timer.h"
#include <Eigen/Geometry>
//#include "pcl/ros/conversions.h"
#include <pcl/common/transformation_from_correspondences.h>
//#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc.hpp"
//#include <qtconcurrentrun.h>
//#include <QtConcurrentMap> 

#ifdef USE_SIFT_GPU
#include "sift_gpu_wrapper.h"
#endif

//#include <math.h>
#include <fstream>

//#ifdef USE_ICP_CODE
//#include "../external/gicp/transform.h"
//#endif
#ifdef USE_ICP_BIN
#include "gicp-fallback.h"
#endif

#ifdef USE_ICP_CODE
#include "gicp/gicp.h"
#include "gicp/transform.h"
#endif

//#include <iostream>
#include "misc.h"
#include "misc2.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <opencv/highgui.h>
#ifdef USE_PCL_ICP
#include "icp.h"
#endif
#include <string>
#include <iostream>
#include <cmath>
QMutex Node::gicp_mutex;
QMutex Node::siftgpu_mutex;

/*Check for all keypoint coordinates whether valid depth is available */
void removeDepthless(std::vector<cv::KeyPoint>& feature_locations_2d, const cv::Mat& depth)
{
  cv::Point2f p2d;
  unsigned int i = 0; 
  while(i < feature_locations_2d.size())
  {
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= depth.cols || p2d.x < 0 ||
        p2d.y >= depth.rows || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    float Z;
    if(ParameterServer::instance()->get<bool>("use_feature_min_depth")){
      Z = getMinDepthInNeighborhood(depth, p2d, feature_locations_2d[i].size);
    } else {
      Z = depth.at<float>(round(p2d.y), round(p2d.x));//unfortunately rounding is necessary. Seldomly, casting the floating point coordinates (b/c subpix accuracy) shifted the point. Happend only for ORB, which produces coordinats like 10.9999959
    }
    // Check for invalid measurements
    if(std::isnan (Z))
    {
      ROS_INFO("%.3u. Feature (%f %f) has been extracted at NaN depth. Omitting", i, p2d.x, p2d.y);
      //FIXME Use parameter here to choose whether to use
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    i++; //Only increment if no element is removed from vector
  }
}

//!Construct node without precomputed point cloud. Computes the point cloud on
//!demand, possibly subsampled
Node::Node(const cv::Mat& visual, 
           const cv::Mat& depth,
           const cv::Mat& detection_mask,
           const sensor_msgs::CameraInfoConstPtr& cam_info, 
           myHeader depth_header,
           cv::Ptr<cv::Feature2D> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor) :
  id_(-1), seq_id_(-1), vertex_id_(-1), valid_tf_estimate_(true),
  timestamp_(depth_header.stamp),
  has_odometry_edge_(false),
  odometry_set_(false),
  matchable_(true),
  pc_col(new pointcloud_type()),
  flannIndex(NULL),
  header_(depth_header),
  base2points_(tf::Transform::getIdentity(), depth_header.stamp, ParameterServer::instance()->get<std::string>("base_frame_name"), depth_header.frame_id),
  ground_truth_transform_(tf::Transform::getIdentity(), depth_header.stamp, ParameterServer::instance()->get<std::string>("ground_truth_frame_name"), ParameterServer::instance()->get<std::string>("base_frame_name")),
  odom_transform_(tf::Transform::getIdentity(), depth_header.stamp, "missing_odometry", depth_header.frame_id),
  initial_node_matches_(0),
  cam_info_(*cam_info)
{
  ScopedTimer s("Node Constructor");
  ParameterServer* ps = ParameterServer::instance();

  //Create point cloud inf necessary
  if(ps->get<bool>("store_pointclouds") || 
     ps->get<int>("emm__skip_step") > 0 ||
     ps->get<bool>("use_icp") ||
     (ps->get<bool>("use_glwidget") && ps->get<bool>("use_gui") && ! ps->get<bool>("glwidget_without_clouds")))
  {
    pc_col = pointcloud_type::Ptr(createXYZRGBPointCloud(depth, visual, cam_info));
  }
  else //Else use empty one
  {
    pc_col = pointcloud_type::Ptr(new pointcloud_type());
  }
  pc_col->header = header_;

  cv::Mat gray_img; 
  if(visual.type() == CV_8UC3){
    cv::cvtColor(visual, gray_img, CV_RGB2GRAY);
  } else {
    gray_img = visual;
  }


#ifdef USE_SIFT_GPU
  std::vector<float> descriptors;
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"){
    ScopedTimer s("Feature Detection and Descriptor Extraction");
    SiftGPUWrapper* siftgpu = SiftGPUWrapper::getInstance();
    siftgpu->detect(gray_img, feature_locations_2d_, descriptors);
    ROS_WARN_COND(descriptors.size()==0, "No keypoints for current image!");
  } else 
#endif
  {
    ScopedTimer s("Feature Detection");
    ROS_FATAL_COND(detector.empty(), "No valid detector!");

    detector->detect( gray_img, feature_locations_2d_, detection_mask);// fill 2d locations
  }

  // project pixels to 3dPositions and create search structures for the gicp
#ifdef USE_SIFT_GPU
  if(ps->get<std::string>("feature_extractor_type") == "SIFTGPU"){

    if(ps->get<std::string>("feature_detector_type") != "SIFTGPU"){
      //not already extracted descriptors in detection step
      //clean keypoints from those without 3d FIXME: can be made more performant
      projectTo3D(feature_locations_2d_, feature_locations_3d_, depth, cam_info);
      SiftGPUWrapper* siftgpu = SiftGPUWrapper::getInstance();
      siftgpu->detect(gray_img, feature_locations_2d_, descriptors);
    } 
    if(descriptors.size() > 0){
      projectTo3DSiftGPU(feature_locations_2d_, feature_locations_3d_, depth, cam_info, descriptors, feature_descriptors_); 
      ROS_INFO("Siftgpu Feature Descriptors size: %d x %d", feature_descriptors_.rows, feature_descriptors_.cols);
    } else {
      ROS_WARN("No descriptors for current image!");
    }
    //std::cout << "feature descriptors = "<< std::endl << std::setprecision(3)  << feature_descriptors_<< std::endl;
  }
  else
#endif
  {
    //PREFILTER 
    removeDepthless(feature_locations_2d_, depth);
    size_t max_keyp = ps->get<int>("max_keypoints");
    if(feature_locations_2d_.size() > max_keyp){
      cv::KeyPointsFilter::retainBest(feature_locations_2d_, max_keyp);  
      feature_locations_2d_.resize(max_keyp);//Because retainBest doesn't retain exactly max_keyp?
    }

    /*for(unsigned int i = 0; i < feature_locations_2d_.size(); i++){
      feature_locations_2d_[i].class_id = i;
      feature_locations_2d_[i].pt.x = round(feature_locations_2d_[i].pt.x);
      feature_locations_2d_[i].pt.y = round(feature_locations_2d_[i].pt.y);
      ROS_WARN("%u. Keypoint %.3i: (%f %f)", i, feature_locations_2d_[i].class_id, feature_locations_2d_[i].pt.x, feature_locations_2d_[i].pt.y);
    }*/


    ScopedTimer s("Feature Extraction");
    extractor->compute(gray_img, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
    /*for(unsigned int i = 0; i < feature_locations_2d_.size(); i++){
      ROS_WARN("%.3u. EKeypoint1 %.3i: (%f %f)", i, feature_locations_2d_[i].class_id, feature_locations_2d_[i].pt.x, feature_locations_2d_[i].pt.y);
    }*/
    removeDepthless(feature_locations_2d_, depth);//FIXME: Unnecessary?
    /* for(unsigned int i = 0; i < feature_locations_2d_.size(); i++){
      ROS_WARN("%.3u. EKeypoint %.3i: (%f %f)", i, feature_locations_2d_[i].class_id, feature_locations_2d_[i].pt.x, feature_locations_2d_[i].pt.y);
    }  */
    projectTo3D(feature_locations_2d_, feature_locations_3d_, depth, cam_info);
    /* for(unsigned int i = 0; i < feature_locations_2d_.size(); i++){
      ROS_WARN("%.3u. EKeypoint3 %.3i: (%f %f)", i, feature_locations_2d_[i].class_id, feature_locations_2d_[i].pt.x, feature_locations_2d_[i].pt.y);
    }*/
    ROS_INFO("Keypoints: %zu", feature_locations_2d_.size());
    ROS_DEBUG("Feature Descriptors size: %d x %d", feature_descriptors_.rows, feature_descriptors_.cols);
  }
  assert(feature_locations_2d_.size() == feature_locations_3d_.size());
  assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 
  feature_matching_stats_.resize(feature_locations_2d_.size(), 0);
  ROS_INFO_NAMED("statistics", "Feature Count of Node:\t%d", (int)feature_locations_2d_.size());
  //computeKeypointDepthStats(depth, feature_locations_2d_);

#ifdef USE_ICP_CODE
  gicp_initialized = false;
  gicp_point_set_ = NULL;
  if(ps->get<int>("emm__skip_step") <= 0 && !ps->get<bool>("store_pointclouds") && ps->get<bool>("use_icp")) 
  {//if clearing out point clouds, the icp structure needs to be built before
    gicp_mutex.lock();
    gicp_point_set_ = this->getGICPStructure();
    gicp_mutex.unlock();
  }
#endif
  if(ps->get<bool>("use_root_sift") &&
     (ps->get<std::string>("feature_extractor_type") == "SIFTGPU" ||
      ps->get<std::string>("feature_extractor_type") == "SURF" ||
      ps->get<std::string>("feature_extractor_type") == "GFTT" ||
      ps->get<std::string>("feature_extractor_type") == "SIFT")){
    squareroot_descriptor_space(feature_descriptors_);
  }
}











Node::Node(const cv::Mat visual,
           cv::Ptr<cv::Feature2D> detector,
           cv::Ptr<cv::DescriptorExtractor> extractor,
           pointcloud_type::Ptr point_cloud,
           const cv::Mat detection_mask) : 
  id_(-1), seq_id_(-1), vertex_id_(-1), valid_tf_estimate_(true), matchable_(true),
  timestamp_(point_cloud->header.stamp),
  has_odometry_edge_(false),
  odometry_set_(false),
  pc_col(point_cloud),
  flannIndex(NULL),
  header_(point_cloud->header),
  base2points_(tf::Transform::getIdentity(), header_.stamp,ParameterServer::instance()->get<std::string>("base_frame_name"), header_.frame_id),
  ground_truth_transform_(tf::Transform::getIdentity(), header_.stamp, ParameterServer::instance()->get<std::string>("ground_truth_frame_name"), ParameterServer::instance()->get<std::string>("base_frame_name")),
  odom_transform_(tf::Transform::getIdentity(), header_.stamp, "missing_odometry", header_.frame_id),
  initial_node_matches_(0)
{
  //cv::namedWindow("matches");
  ParameterServer* ps = ParameterServer::instance();

  ROS_INFO_STREAM("Construction of Node with " << ps->get<std::string>("feature_detector_type") << " Features");
  ScopedTimer s("Node Constructor");

  cv::Mat gray_img; 
  if(visual.type() == CV_8UC3){ cv::cvtColor(visual, gray_img, CV_RGB2GRAY); } 
  else { gray_img = visual; }


#ifdef USE_SIFT_GPU
  std::vector<float> descriptors;
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"){
    ScopedTimer s("Feature Detection and Descriptor Extraction");
    SiftGPUWrapper* siftgpu = SiftGPUWrapper::getInstance();
    siftgpu->detect(gray_img, feature_locations_2d_, descriptors);
    ROS_FATAL_COND(descriptors.size() ==0, "Can't run SiftGPU");
  } else 
#endif
  if(ps->get<std::string>("feature_detector_type") != "GICP")
  {
    ScopedTimer s("Feature Detection");
    ROS_FATAL_COND(detector.empty(), "No valid detector!");
    detector->detect( gray_img, feature_locations_2d_, detection_mask);// fill 2d locations
  }

  // project pixels to 3dPositions and create search structures for the gicp
#ifdef USE_SIFT_GPU
  if(ps->get<std::string>("feature_detector_type") == "SIFTGPU"
     && descriptors.size() > 0)
  {
    // removes also unused descriptors from the descriptors matrix
    // build descriptor matrix and sets siftgpu_descriptors!
    projectTo3DSiftGPU(feature_locations_2d_, feature_locations_3d_, pc_col, descriptors, feature_descriptors_); //takes less than 0.01 sec
  }
  else 
#endif
  if(ps->get<std::string>("feature_detector_type") != "GICP")
  {
    projectTo3D(feature_locations_2d_, feature_locations_3d_, pc_col); //takes less than 0.01 sec
    // projectTo3d need a dense cloud to use the points.at(px.x,px.y)-Call
    ScopedTimer s("Feature Extraction");
    extractor->compute(gray_img, feature_locations_2d_, feature_descriptors_); //fill feature_descriptors_ with information 
  }

  if(ps->get<std::string>("feature_detector_type") != "GICP")
  {
    assert(feature_locations_2d_.size() == feature_locations_3d_.size());
    assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 
    feature_matching_stats_.resize(feature_locations_2d_.size(), 0);
    ROS_INFO_NAMED("statistics", "Feature Count of Node:\t%d", (int)feature_locations_2d_.size());
    /* Now inside the projection method:
    size_t max_keyp = ps->get<int>("max_keypoints");
    if(feature_locations_2d_.size() > max_keyp) {
      feature_locations_2d_.resize(max_keyp);
      feature_locations_3d_.resize(max_keyp);
      feature_descriptors_ = feature_descriptors_.rowRange(0,max_keyp);
    }
    */
    assert(feature_locations_2d_.size() == feature_locations_3d_.size());
    assert(feature_locations_3d_.size() == (unsigned int)feature_descriptors_.rows); 
  }

#ifdef USE_ICP_CODE
  gicp_initialized = false;
  gicp_point_set_ = NULL;
  if(!ps->get<bool>("store_pointclouds") && ps->get<bool>("use_icp")) 
  {//if clearing out point clouds, the icp structure needs to be built before
    gicp_mutex.lock();
    gicp_point_set_ = this->getGICPStructure();
    gicp_mutex.unlock();
  }
#endif

  if((!ps->get<bool>("use_glwidget") ||
      !ps->get<bool>("use_gui")) &&
     !ps->get<bool>("store_pointclouds") &&
     !ps->get<int>("emm__skip_step"))
  {
    ROS_WARN("Clearing out points");
    this->clearPointCloud();
  } else if(ps->get<double>("voxelfilter_size") > 0.0) {
    //Let it be voxelfiltered with the call to clearPointCloud
    /*
    double vfs = ps->get<double>("voxelfilter_size");
    pcl::VoxelGrid<point_type> sor;
    sor.setLeafSize(vfs,vfs,vfs);
    pointcloud_type::ConstPtr const_cloud_ptr = boost::make_shared<pointcloud_type> (*pc_col);                                                                 
    sor.setInputCloud (const_cloud_ptr);
    sor.filter (*pc_col);
    */
  }
  if(ps->get<bool>("use_root_sift") &&
     (ps->get<std::string>("feature_extractor_type") == "SIFTGPU" ||
      ps->get<std::string>("feature_extractor_type") == "SURF" ||
      ps->get<std::string>("feature_extractor_type") == "GFTT" ||
      ps->get<std::string>("feature_extractor_type") == "SIFT")){
    squareroot_descriptor_space(feature_descriptors_);
  }
}

Node::~Node() {
    delete flannIndex; flannIndex = NULL;
}

void Node::setOdomTransform(tf::StampedTransform gt){
    odom_transform_ = gt;
    odometry_set_=true;
}
void Node::setGroundTruthTransform(tf::StampedTransform gt){
    ground_truth_transform_ = gt;
}
void Node::setBase2PointsTransform(tf::StampedTransform& b2p){
    base2points_ = b2p;
}
tf::StampedTransform Node::getOdomTransform() const {
    return odom_transform_;
}
tf::StampedTransform Node::getGroundTruthTransform() const {
    return ground_truth_transform_;
}
tf::StampedTransform Node::getBase2PointsTransform() const {
    return base2points_;
}

#ifdef USE_ICP_CODE
bool Node::getRelativeTransformationTo_ICP_code(const Node* target_node,
                                                Eigen::Matrix4f& transformation,
                                                const Eigen::Matrix4f& initial_transformation)
{
  ScopedTimer s(__FUNCTION__);
  dgc_transform_t initial;
  Eigen2GICP(initial_transformation,initial);

  dgc_transform_t final_trafo;
  dgc_transform_identity(final_trafo);

  gicp_mutex.lock();
	dgc::gicp::GICPPointSet* gicp_point_set = this->getGICPStructure();
  ROS_INFO("this'  (%d) Point Set: %d", this->id_, gicp_point_set->Size());
	dgc::gicp::GICPPointSet* target_gicp_point_set = target_node->getGICPStructure();
  ROS_INFO("others (%d) Point Set: %d", target_node->id_, target_gicp_point_set->Size());
  int iterations = gicp_max_iterations;
  if(gicp_point_set->Size() > Node::gicp_min_point_cnt && 
     target_gicp_point_set->Size() > Node::gicp_min_point_cnt)
  {
   iterations = target_gicp_point_set->AlignScan(gicp_point_set, initial, final_trafo, gicp_d_max_);
   GICP2Eigen(final_trafo,transformation);
  } else {
    ROS_WARN("GICP Point Sets not big enough. Skipping ICP");
  }
  gicp_mutex.unlock();


  return iterations <= gicp_max_iterations;
}

void Node::clearGICPStructure() const
{
    gicp_mutex.lock();
    delete gicp_point_set_; gicp_point_set_ = NULL;
    gicp_mutex.unlock();
}
dgc::gicp::GICPPointSet* Node::getGICPStructure(unsigned int max_count) const
{
  ScopedTimer s(__FUNCTION__);
  if(max_count == 0) max_count = ParameterServer::instance()->get<int>("gicp_max_cloud_size");
  //Use Cache
  if(gicp_point_set_ != NULL){
    return gicp_point_set_;
  }
  
  dgc::gicp::GICPPointSet* gicp_point_set = new dgc::gicp::GICPPointSet();

  dgc::gicp::GICPPoint g_p;
  g_p.range = -1;
  for(int k = 0; k < 3; k++) {
    for(int l = 0; l < 3; l++) {
      g_p.C[k][l] = (k == l)?1:0;
    }
  }

  std::vector<dgc::gicp::GICPPoint> non_NaN;
  non_NaN.reserve((*pc_col).points.size());
  for (unsigned int i=0; i<(*pc_col).points.size(); i++ ){
    point_type&  p = (*pc_col).points.at(i);
    if (!std::isnan(p.z)) { // add points to candidate pointset for icp
      g_p.x=p.x;
      g_p.y=p.y;
      g_p.z=p.z;
      non_NaN.push_back(g_p);
    }
  }
  float step = non_NaN.size()/static_cast<float>(max_count);
  step =  step < 1.0 ? 1.0 : step; //only skip, don't use points more than once
  for (float i=0; i<non_NaN.size(); i+=step ){
    gicp_point_set->AppendPoint(non_NaN[static_cast<unsigned int>(i)]);
  }
  ROS_INFO("GICP point set size: %i", gicp_point_set->Size() );
  
  if(gicp_point_set->Size() > Node::gicp_min_point_cnt){
    ScopedTimer s("GICP structure creation");
    // build search structure for gicp:
    gicp_point_set->SetDebug(true);
    gicp_point_set->SetGICPEpsilon(gicp_epsilon);
    gicp_point_set->BuildKDTree();
    gicp_point_set->ComputeMatrices();
    gicp_point_set->SetMaxIterationInner(8); // as in test_gicp->cpp
    gicp_point_set->SetMaxIteration(gicp_max_iterations);
  }
  else
  {
    ROS_WARN("GICP point set too small, this node will not be algined with GICP!");
  }
  //ROS_INFO_STREAM("time for creating the structure: " << ((std::clock()-starttime_gicp*1.0) / (double)CLOCKS_PER_SEC));
  //ROS_INFO_STREAM("current: " << std::clock() << " " << "start_time: " << starttime_gicp);

  gicp_point_set_ = gicp_point_set;
  return gicp_point_set;
}
#endif

//Build flann index on demand
const cv::flann::Index* Node::getFlannIndex() const {
// build search structure for descriptor matching
  if (flannIndex == NULL
      && ParameterServer::instance()->get<std::string> ("matcher_type") == "FLANN" 
      && ParameterServer::instance()->get<std::string> ("feature_detector_type") != "GICP"
      && ParameterServer::instance()->get<std::string> ("feature_extractor_type") != "ORB")
  {
    ScopedTimer s(__FUNCTION__);
    //KDTreeIndexParams When passing an object of this type the index constructed will 
    //consist of a set of randomized kd-trees which will be searched in parallel.
    flannIndex = new cv::flann::Index(feature_descriptors_, cv::flann::KDTreeIndexParams(4));
    ROS_DEBUG("Built flannIndex (address %p) for Node %i", flannIndex, this->id_);
  }
  else if (flannIndex == NULL
      && ParameterServer::instance()->get<std::string> ("matcher_type") == "FLANN" 
      && ParameterServer::instance()->get<std::string> ("feature_extractor_type") == "ORB")
  {
    flannIndex = new cv::flann::Index(feature_descriptors_, cv::flann::LshIndexParams(10, 10, 2));
  }

  return flannIndex;
}

bool isNearer(const cv::DMatch& m1, const cv::DMatch& m2) { 
  return m1.distance < m2.distance; 
}
//Throw away the worst matches based on their distance - which is the nn_ratio set in featureMatching(...)
static void keepStrongestMatches(int n, std::vector<cv::DMatch>* matches)
{
  if(matches->size() > n)
  {
    //m1 better than m2?
    //auto lambda = [](const cv::DMatch& m1, const cv::DMatch& m2) { return m1.distance < m2.distance; };

    std::vector<cv::DMatch>::iterator nth = matches->begin() + n;
    std::nth_element(matches->begin(), nth, matches->end(), isNearer);
    matches->erase(nth, matches->end());
  }
}


//TODO: This function seems to be resistant to parallelization probably due to knnSearch
unsigned int Node::featureMatching(const Node* other, std::vector<cv::DMatch>* matches) const 
{
  ScopedTimer s(__FUNCTION__);
  assert(matches->size()==0);
  // number of neighbours found (two, to compare the best matches for distinctness
  const int k = 2;
  //unsigned int one_nearest_neighbour = 0, two_nearest_neighbours = 0;

  // number of neighbors found (has to be two, see l. 57)
  double sum_distances = 0.0;
  ParameterServer* ps = ParameterServer::instance();
  //const int min_kp = ps->get<int> ("min_keypoints");

  //using siftgpu, if available and wanted
  if(ps->get<std::string>("feature_detector_type") == "GICP"){
    return 0;
  }
#ifdef USE_SIFT_GPU
  if (ps->get<std::string> ("matcher_type") == "SIFTGPU") {
    siftgpu_mutex.lock();
    sum_distances = SiftGPUWrapper::getInstance()->match(siftgpu_descriptors, feature_descriptors_.rows, other->siftgpu_descriptors, other->feature_descriptors_.rows, matches);
    siftgpu_mutex.unlock();
  }
  else
#endif
  //using BruteForceMatcher for ORB features
  if (ps->get<std::string> ("matcher_type") != "BRUTEFORCE_OPENCV" && ps->get<std::string> ("feature_extractor_type") == "ORB")
  {
    ROS_INFO_COND(ps->get<std::string>("matcher_type") == "FLANN", "You specified FLANN with ORB, this is SLOW! Using BRUTEFORCE instead.");
    cv::Ptr<cv::DescriptorMatcher> matcher;
    if(ps->get<std::string> ("feature_extractor_type") == "ORB"){
        ScopedTimer s("My bruteforce Search", false, true);
        uint64_t* query_value =  reinterpret_cast<uint64_t*>(this->feature_descriptors_.data);
        uint64_t* search_array = reinterpret_cast<uint64_t*>(other->feature_descriptors_.data);
        for(unsigned int i = 0; i < this->feature_locations_2d_.size(); ++i, query_value += 4){//ORB feature = 32*8bit = 4*64bit
          int result_index = -1;
          int hd = bruteForceSearchORB(query_value, search_array, other->feature_locations_2d_.size(), result_index);
          if(hd >= 128) continue;//not more than half of the bits matching: Random
          cv::DMatch match(i, result_index, hd /256.0 + (float)rand()/(1000.0*RAND_MAX));
          matches->push_back(match);
        }
    }
    //else//any bruteforce
    if (ps->get<std::string> ("matcher_type") == "BRUTEFORCE_OPENCV" && ps->get<std::string> ("feature_extractor_type") == "ORB")
    {
      ScopedTimer s("OpenCV bruteforce Search", false, true);
      std::string brute_force_type("BruteForce-HammingLUT"); //L2 per default
      matcher = cv::DescriptorMatcher::create(brute_force_type);
      std::vector< std::vector<cv::DMatch> > bruteForceMatches;
      matcher->knnMatch(feature_descriptors_, other->feature_descriptors_, bruteForceMatches, k);
      double max_dist_ratio_fac = ps->get<double>("nn_distance_ratio");
      //if ((int)bruteForceMatches.size() < min_kp) max_dist_ratio_fac = 1.0; //if necessary use possibly bad descriptors
      srand((long)std::clock());
      std::set<int> train_indices;
      matches->clear();
      matches->reserve(bruteForceMatches.size());
      for(unsigned int i = 0; i < bruteForceMatches.size(); i++) {
          cv::DMatch m1 = bruteForceMatches[i][0];
          cv::DMatch m2 = bruteForceMatches[i][1];
          float dist_ratio_fac = m1.distance / m2.distance;
          if (dist_ratio_fac < max_dist_ratio_fac) {//this check seems crucial to matching quality
              int train_idx = m1.trainIdx;
              if(train_indices.count(train_idx) > 0)
                continue; //FIXME: Keep better
                
              train_indices.insert(train_idx);
              sum_distances += m1.distance;
              m1.distance = dist_ratio_fac + (float)rand()/(1000.0*RAND_MAX); //add a small random offset to the distance, since later the dmatches are inserted to a set, which omits duplicates and the duplicates are found via the less-than function, which works on the distance. Therefore we need to avoid equal distances, which happens very often for ORB
              matches->push_back(m1);
          } 

      }
    }
    //matcher->match(feature_descriptors_, other->feature_descriptors_, *matches);
  } 
  else if (ps->get<std::string>("matcher_type") == "FLANN") // && ps->get<std::string>("feature_extractor_type") != "ORB")
  {
    ScopedTimer s("OpenCV FLANN Search", false, true);
    int start_feature = 0; //feature_descriptors_.rows - std::min(2*ps->get<int>("max_matches"), feature_descriptors_.rows);//FIXME: search only for best keypoints (in SIFTGPU they are at the back)
    int num_features = feature_descriptors_.rows;                               //compute features per chunk
    { //old block of (removed) search for matches chunkwise
      // compare
      // http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
      cv::Mat indices(num_features-start_feature, k, CV_32S);
      cv::Mat dists(num_features-start_feature, k, CV_32F);
      cv::Mat relevantDescriptors = feature_descriptors_.rowRange(start_feature, num_features);

      // get the best two neighbors
      {
        ScopedTimer s("FLANN KNN-search");
        /* 64: The number of times the tree(s) in the index should be
         * recursively traversed. A higher value for this parameter would give
         * better search precision, but also take more time. If automatic
         * configuration was used when the index was created, the number of
         * checks required to achieve the specified precision was also
         * computed, in which case this parameter is ignored.
         */
        other->knnSearch(relevantDescriptors, indices, dists, k, cv::flann::SearchParams(16));
      }

      cv::DMatch match;
      double avg_ratio = 0.0;
      double max_dist_ratio_fac = ps->get<double>("nn_distance_ratio");
      std::set<int> train_indices;
      matches->clear();
      matches->reserve(indices.rows);
      for(int i = 0; i < indices.rows; ++i) {
        float dist_ratio_fac =  dists.at<float>(2*i) / dists.at<float>(2*i + 1);
        avg_ratio += dist_ratio_fac;
        //if (indices.rows < min_kp) dist_ratio_fac = 1.0; //if necessary use possibly bad descriptors
        if (max_dist_ratio_fac > dist_ratio_fac) {
          int train_idx = indices.at<int>(2 * i);
          if(train_indices.count(train_idx) > 0)
            continue; //FIXME: Keep better
            
          train_indices.insert(train_idx);
          match.queryIdx = i+start_feature;
          match.trainIdx = train_idx;
          match.distance = dist_ratio_fac; //dists_ptr[2 * i];
          sum_distances += match.distance;

          assert(match.trainIdx < other->feature_descriptors_.rows);
          assert(match.queryIdx < feature_descriptors_.rows);
          matches->push_back(match);
        }
      }
      
      ROS_INFO("Feature Matches between Nodes %3d (%4d features) and %3d (%4d features) (features %d to %d of first node):\t%4d. Percentage: %f%%, Avg NN Ratio: %f",
                this->id_, (int)this->feature_locations_2d_.size(), other->id_, (int)other->feature_locations_2d_.size(), start_feature, num_features, 
                (int)matches->size(), (100.0*matches->size())/((float)num_features-start_feature), avg_ratio / (num_features-start_feature));

    }//for
  }
  else {
      ROS_FATAL_STREAM("Cannot match features:\nNo valid combination for " <<
                       "matcher_type ("           << ps->get<std::string>("matcher_type") << ") and " <<
                       "feature_extractor_type (" << ps->get<std::string>("feature_extractor_type") << ") chosen.");
  }

  keepStrongestMatches(ps->get<int>("max_matches"), matches);

  ROS_DEBUG_NAMED("statistics", "count_matrix(%3d, %3d) =  %4d;",
                  this->id_+1, other->id_+1, (int)matches->size());
  ROS_DEBUG_NAMED("statistics", "dista_matrix(%3d, %3d) =  %f;",
                 this->id_+1, other->id_+1, sum_distances/ (float)matches->size());
  ROS_DEBUG_NAMED("statistics", "Feature Matches between Nodes %3d (%4d features) and %3d (%4d features):\t%4d",
                  this->id_, (int)this->feature_locations_2d_.size(),
                  other->id_, (int)other->feature_locations_2d_.size(),
                  (int)matches->size());

  //ROS_INFO("matches size: %i, rows: %i", (int) matches->size(), feature_descriptors_.rows);

  //assert(one_nearest_neighbour+two_nearest_neighbours > 0);
  //return static_cast<float>(one_nearest_neighbour) / static_cast<float>(one_nearest_neighbour+two_nearest_neighbours);
  return matches->size();
}



#ifdef USE_SIFT_GPU
void Node::projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
                              std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                              const cv::Mat& depth,
                              const sensor_msgs::CameraInfoConstPtr& cam_info,
                              std::vector<float>& descriptors_in, cv::Mat& descriptors_out)
{

  ScopedTimer s(__FUNCTION__);

  ParameterServer* ps = ParameterServer::instance();
  double depth_scaling = ps->get<double>("depth_scaling_factor");
  size_t max_keyp = ps->get<int>("max_keypoints");
  float x,y, z;//temp point, 
  //principal point and focal lengths:
  float fxinv = 1./ (ps->get<double>("depth_camera_fx") != 0 ? ps->get<double>("depth_camera_fx") : cam_info->K[0]); //(cloud->width >> 1) - 0.5f;
  float fyinv = 1./ (ps->get<double>("depth_camera_fy") != 0 ? ps->get<double>("depth_camera_fy") : cam_info->K[4]); //(cloud->width >> 1) - 0.5f;
  float cx = ps->get<double>("depth_camera_cx") != 0 ? ps->get<double>("depth_camera_cx") : cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
  float cy = ps->get<double>("depth_camera_cy") != 0 ? ps->get<double>("depth_camera_cy") : cam_info->K[5]; //(cloud->width >> 1) - 0.5f;
  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  std::list<int> featuresUsed;
  
  int index = -1;

  bool use_feature_min_depth = ParameterServer::instance()->get<bool>("use_feature_min_depth");
  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    ++index;

    p2d = feature_locations_2d[i].pt;
    float Z;
    if(use_feature_min_depth ){
      Z = getMinDepthInNeighborhood(depth, p2d, feature_locations_2d[i].size) * depth_scaling;
    } else {
      Z = depth.at<float>(p2d.y, p2d.x) * depth_scaling;//unfortunately rounding is necessary. Seldomly, casting the floating point coordinates (b/c subpix accuracy) shifted the point. Happend only for ORB, which produces coordinats like 10.9999959
    }
    // Check for invalid measurements
    if (std::isnan (Z))
    {
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    else
    {
      backProject(fxinv, fyinv, cx, cy, p2d.x, p2d.y, Z, x, y, z);
    }

    feature_locations_3d.push_back(Eigen::Vector4f(x,y, z, 1.0));
    featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
    i++; //Only increment if no element is removed from vector
    if(feature_locations_3d.size() >= max_keyp) break;
  }

  //create descriptor matrix
  int size = feature_locations_3d.size();
  descriptors_out = cv::Mat(size, 128, CV_32F);
  siftgpu_descriptors.resize(size * 128);
  assert(featuresUsed.size() == feature_locations_3d.size());
  for (int y = 0; y < size; ++y) {
    int id = featuresUsed.front();
    featuresUsed.pop_front();

    for (int x = 0; x < 128; ++x) {
      float tmp = descriptors_in[id * 128 + x];
      descriptors_out.at<float>(y, x) = tmp;
      siftgpu_descriptors[y * 128 + x] = tmp;
    }
  }

  feature_locations_2d.resize(feature_locations_3d.size());
}

#ifdef HEMACLOUDS
bool searchLabelInNeighborhood( const pointcloud_type::Ptr point_cloud, cv::Point2f center, float diameter, int label)
{
    // Get neighbourhood area of keypoint
    int radius = (diameter - 1)/2;
    int top   = center.y - radius; top   = top   < 0 ? 0 : top;
    int left  = center.x - radius; left  = left  < 0 ? 0 : left;
    int bot   = center.y + radius; bot   = bot   >= point_cloud->height ? point_cloud->height-1 : bot;
    int right = center.x + radius; right = right >= point_cloud->width ? point_cloud->width-1 : right;
    for(int x = left; x <= right; ++x){
      for(int y = top; y <= bot; ++y){
        if(point_cloud->at(x,y).label == label){
          return true;
        }
      } 
    } 
    return false;
}
#endif

void Node::projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
                              std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                              const pointcloud_type::Ptr point_cloud, 
                              std::vector<float>& descriptors_in, cv::Mat& descriptors_out)
{
  ScopedTimer s(__FUNCTION__);
  size_t max_keyp = ParameterServer::instance()->get<int>("max_keypoints");
  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  std::list<int> featuresUsed;

  int index = -1;
  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    ++index;

    p2d = feature_locations_2d[i].pt;
    point_type p3d = point_cloud->at((int) p2d.x,(int) p2d.y);

    // Check for invalid measurements
    if (std::isnan(p3d.z))
    {
      ROS_DEBUG("Feature %d has been extracted at NaN depth. Using pixel coordinates", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    
#ifdef HEMACLOUDS
    int target_label = ParameterServer::instance()->get<int>("segment_to_optimize");
    //FIXME: Reversed: Label must not be there
    if(target_label >= 0 && searchLabelInNeighborhood(point_cloud, p2d, 1, target_label)){ // Optimize transformation estimation specifically for features in segment
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
#endif
    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1));
    i++; //Only increment if no element is removed from vector
    featuresUsed.push_back(index);  //save id for constructing the descriptor matrix
    if(feature_locations_3d.size() >= max_keyp) break;
  }

  //create descriptor matrix
  int size = feature_locations_3d.size();
  descriptors_out = cv::Mat(size, 128, CV_32F);
  siftgpu_descriptors.resize(size * 128);
  for (int y = 0; y < size && featuresUsed.size() > 0; ++y) {
    int id = featuresUsed.front();
    featuresUsed.pop_front();

    for (int x = 0; x < 128; ++x) {
      descriptors_out.at<float>(y, x) = descriptors_in[id * 128 + x];
      siftgpu_descriptors[y * 128 + x] = descriptors_in[id * 128 + x];
    }
  }

  feature_locations_2d.resize(feature_locations_3d.size());

}
#endif

void Node::projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
                       std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                       pointcloud_type::ConstPtr point_cloud)
{
  ScopedTimer s(__FUNCTION__);

  size_t max_keyp = ParameterServer::instance()->get<int>("max_keypoints");
  double maximum_depth = ParameterServer::instance()->get<double>("maximum_depth");
  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }

  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= point_cloud->width || p2d.x < 0 ||
        p2d.y >= point_cloud->height || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    point_type p3d = point_cloud->at((int) p2d.x,(int) p2d.y);

    // Check for invalid measurements
    if ( (p3d.z > maximum_depth) || std::isnan(p3d.x) || std::isnan(p3d.y) || std::isnan(p3d.z))
    {
      ROS_DEBUG_NAMED(__FILE__, "Feature %d has been extracted at NaN depth. Omitting", i);
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }

    feature_locations_3d.push_back(Eigen::Vector4f(p3d.x, p3d.y, p3d.z, 1.0));
    i++; //Only increment if no element is removed from vector
    if(feature_locations_3d.size() >= max_keyp) break;
  }

  ROS_INFO("Feature 2d size: %zu, 3D: %zu", feature_locations_2d.size(), feature_locations_3d.size());
  feature_locations_2d.resize(feature_locations_3d.size());
  ROS_INFO("Feature 2d size: %zu, 3D: %zu", feature_locations_2d.size(), feature_locations_3d.size());
}

void Node::projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
                       std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                       const cv::Mat& depth,
                       const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  ScopedTimer s(__FUNCTION__);

  ParameterServer* ps = ParameterServer::instance();
  double depth_scaling = ps->get<double>("depth_scaling_factor");
  size_t max_keyp = ps->get<int>("max_keypoints");
  double maximum_depth = ps->get<double>("maximum_depth");
  float x,y,z;//temp point, 
  //principal point and focal lengths:
  float fxinv = 1./ (ps->get<double>("depth_camera_fx") != 0 ? ps->get<double>("depth_camera_fx") : cam_info->K[0]); //(cloud->width >> 1) - 0.5f;
  float fyinv = 1./ (ps->get<double>("depth_camera_fy") != 0 ? ps->get<double>("depth_camera_fy") : cam_info->K[4]); //(cloud->width >> 1) - 0.5f;
  float cx = ps->get<double>("depth_camera_cx") != 0 ? ps->get<double>("depth_camera_cx") : cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
  float cy = ps->get<double>("depth_camera_cy") != 0 ? ps->get<double>("depth_camera_cy") : cam_info->K[5]; //(cloud->width >> 1) - 0.5f;
  /*
  float cx = 325.1;//cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
  float cy = 249.7;//cam_info->K[5]; //(cloud->height >> 1) - 0.5f;
  float fxinv = 1.0/521.0;//1.0f / cam_info->K[0]; 
  float fyinv = 1.0/521.0;//1.0f / cam_info->K[4]; 
  */
  cv::Point2f p2d;

  if(feature_locations_3d.size()){
    ROS_INFO("There is already 3D Information in the FrameInfo, clearing it");
    feature_locations_3d.clear();
  }
  bool use_feature_min_depth = ParameterServer::instance()->get<bool>("use_feature_min_depth");
  for(unsigned int i = 0; i < feature_locations_2d.size(); /*increment at end of loop*/){
    p2d = feature_locations_2d[i].pt;
    if (p2d.x >= depth.cols || p2d.x < 0 ||
        p2d.y >= depth.rows || p2d.y < 0 ||
        std::isnan(p2d.x) || std::isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
      ROS_WARN_STREAM("Ignoring invalid keypoint: " << p2d); //Does it happen at all? If not, remove this code block
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    float Z;
    if(use_feature_min_depth){
      Z = getMinDepthInNeighborhood(depth, p2d, feature_locations_2d[i].size) * depth_scaling;
    } else {
      Z = depth.at<float>(round(p2d.y), round(p2d.x)) * depth_scaling;//unfortunately rounding is necessary. Seldomly, casting the floating point coordinates (b/c subpix accuracy) shifted the point. Happend only for ORB, which produces coordinats like 10.9999959
      //printMatrixInfo(depth, "Depth Image");
      //ROS_INFO("X Y Z: %f %f %f", p2d.x, p2d.y, Z);
    }
    // Check for invalid measurements
    if(std::isnan (Z))
    {
      ROS_DEBUG("Feature %d has been extracted at NaN depth. Omitting", i);
      //FIXME Use parameter here to choose whether to use
      feature_locations_2d.erase(feature_locations_2d.begin()+i);
      continue;
    }
    backProject(fxinv, fyinv, cx, cy, p2d.x, p2d.y, Z, x, y, z);

    feature_locations_3d.push_back(Eigen::Vector4f(x, y, z, 1.0));
    i++; //Only increment if no element is removed from vector
    if(feature_locations_3d.size() >= max_keyp) break;
  }

  //ROS_INFO("Feature 2d size: %zu, 3D: %zu", feature_locations_2d.size(), feature_locations_3d.size());
  feature_locations_2d.resize(feature_locations_3d.size());
  ROS_INFO("Feature 2d size: %zu, 3D: %zu", feature_locations_2d.size(), feature_locations_3d.size());
}


void Node::computeInliersAndError(const std::vector<cv::DMatch> & all_matches,
                                  const Eigen::Matrix4f& transformation4f,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                                  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
                                  size_t min_inliers, //break if this can't be reached
                                  std::vector<cv::DMatch>& inliers, //pure output var
                                  double& return_mean_error,//pure output var: rms-mahalanobis-distance
                                  //std::vector<double>& errors,
                                  double squaredMaxInlierDistInM) const
{ 
  inliers.clear();
  assert(all_matches.size() > 0);
  inliers.reserve(all_matches.size());
  //errors.clear();
  const size_t all_matches_size = all_matches.size();
  double mean_error = 0.0;
  Eigen::Matrix4d transformation4d = transformation4f.cast<double>();

//parallelization is detrimental here
//#pragma omp parallel for reduction (+: mean_error)
  for(int i=0; i < all_matches_size; ++i)
  //BOOST_FOREACH(const cv::DMatch& m, all_matches)
  {
    const cv::DMatch& m = all_matches[i];
    const Eigen::Vector4f& origin = origins[m.queryIdx];
    const Eigen::Vector4f& target = earlier[m.trainIdx];
    if(origin(2) == 0.0 || target(2) == 0.0){ //does NOT trigger on NaN
       continue;
    }
    double mahal_dist = errorFunction2(origin, target, transformation4d);
    if(mahal_dist > squaredMaxInlierDistInM){
      continue; //ignore outliers
    }
    if(!(mahal_dist >= 0.0)){
      ROS_WARN_STREAM("Mahalanobis_ML_Error: "<<mahal_dist);
      ROS_WARN_STREAM("Transformation for error !>= 0:\n" << transformation4d << "Matches: " << all_matches.size());
      continue;
    }
    mean_error += mahal_dist;
//#pragma omp critical
    inliers.push_back(m); //include inlier
  }


  if (inliers.size()<3){ //at least the samples should be inliers
    ROS_DEBUG("No inliers at all in %d matches!", (int)all_matches.size()); // only warn if this checks for all initial matches
    return_mean_error = 1e9;
  } else {
    mean_error /= inliers.size();
    return_mean_error = sqrt(mean_error);
  }

}


///Randomly choose <sample_size> of the matches
std::vector<cv::DMatch> sample_matches_prefer_by_distance(unsigned int sample_size, std::vector<cv::DMatch>& matches_with_depth)
{
    //Sample ids to pick matches lateron (because they are unique and the
    //DMatch operator< overload breaks uniqueness of the Matches if they have the
    //exact same distance, e.g., 0.0)
    std::set<std::vector<cv::DMatch>::size_type> sampled_ids;
    int safety_net = 0;
    while(sampled_ids.size() < sample_size && matches_with_depth.size() >= sample_size){
      //generate a set of samples. Using a set solves the problem of drawing a sample more than once
      int id1 = rand() % matches_with_depth.size();
      int id2 = rand() % matches_with_depth.size();
      if(id1 > id2) id1 = id2; //use smaller one => increases chance for lower id
      sampled_ids.insert(id1);
      if(++safety_net > 10000){ ROS_ERROR("Infinite Sampling"); break; } 
    }

    //Given the ids, construct the resulting vector
    std::vector<cv::DMatch> sampled_matches;
    sampled_matches.reserve(sampled_ids.size());
    BOOST_FOREACH(std::vector<cv::DMatch>::size_type id, sampled_ids){
      sampled_matches.push_back(matches_with_depth[id]);
    }
    return sampled_matches;
}

///Randomly choose <sample_size> of the matches
std::vector<cv::DMatch> sample_matches(unsigned int sample_size, std::vector<cv::DMatch>& matches_with_depth)
{
    //Sample ids to pick matches lateron (because they are unique and the
    //DMatch operator< overload breaks uniqueness of the Matches if they have the
    //exact same distance, e.g., 0.0)
    std::set<std::vector<cv::DMatch>::size_type> sampled_ids;
    int safety_net = 0;
    while(sampled_ids.size() < sample_size && matches_with_depth.size() >= sample_size){
      //generate a set of samples. Using a set solves the problem of drawing a sample more than once
      sampled_ids.insert(rand() % matches_with_depth.size());
      if(++safety_net > 10000){ ROS_ERROR("Infinite Sampling"); break; } 
    }

    //Given the ids, construct the resulting vector
    std::vector<cv::DMatch> sampled_matches;
    sampled_matches.reserve(sampled_ids.size());
    BOOST_FOREACH(std::vector<cv::DMatch>::size_type id, sampled_ids){
      sampled_matches.push_back(matches_with_depth[id]);
    }
    return sampled_matches;
}

///Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool Node::getRelativeTransformationTo(const Node* earlier_node,
                                       std::vector<cv::DMatch>* initial_matches,
                                       Eigen::Matrix4f& resulting_transformation,
                                       float& rmse, 
                                       std::vector<cv::DMatch>& matches) const
{
  ScopedTimer s(__FUNCTION__);
  static const bool allow_features_without_depth = ParameterServer::instance()->get<bool>("allow_features_without_depth");
  //VALIDATION
  assert(initial_matches != NULL);
  
  std::stringstream nodesstringstream; nodesstringstream << "Nodes " << (this->id_) << "<->" << (earlier_node->id_);
  std::string nodesstring = nodesstringstream.str();
  if(initial_matches->size() <= (unsigned int) ParameterServer::instance()->get<int>("min_matches")){
    ROS_INFO("%s: Only %d feature matches between %d and %d (minimal: %i)",nodesstring.c_str(), (int)initial_matches->size() , this->id_, earlier_node->id_, ParameterServer::instance()->get<int>("min_matches"));
    return false;
  }

  //PREPARATION
  //unsigned int min_inlier_threshold = int(initial_matches->size()*0.2);
  unsigned int min_inlier_threshold = (unsigned int) ParameterServer::instance()->get<int>("min_matches");
  if(min_inlier_threshold > 0.75 * initial_matches->size()){
    //FIXME: Evaluate whether beneficial
    ROS_INFO("Lowering min_inlier_threshold from %d to %d, because there are only %d matches to begin with", min_inlier_threshold, (int) (0.75 * initial_matches->size()), (int)initial_matches->size());
    min_inlier_threshold = 0.75 * initial_matches->size();
  }

  double inlier_error; //all squared errors
  srand((long)std::clock());
  
  // a point is an inlier if it's no more than max_dist_m m from its partner apart
  const float max_dist_m = ParameterServer::instance()->get<double>("max_dist_for_inliers");
  const int ransac_iterations = ParameterServer::instance()->get<int>("ransac_iterations");
  //std::vector<double> dummy;

  // initialize result values of all iterations 
  matches.clear();
  resulting_transformation = Eigen::Matrix4f::Identity();
  rmse = 1e6;
  unsigned int valid_iterations = 0;//, best_inlier_cnt = 0;
  const unsigned int sample_size = 4;// chose this many randomly from the correspondences:
  bool valid_tf = false; // valid is false iff the sampled points clearly aren't inliers themself 

  std::vector<cv::DMatch>* matches_with_depth = initial_matches;
  if(allow_features_without_depth){ 
      matches_with_depth = new std::vector<cv::DMatch>(); //matches without depth can validate but not create the trafo
      matches_with_depth->reserve(initial_matches->size());
    BOOST_FOREACH(const cv::DMatch& m, *initial_matches){
        if(!std::isnan(this->feature_locations_3d_[m.queryIdx](2)) 
           && !std::isnan(earlier_node->feature_locations_3d_[m.trainIdx](2)))
            matches_with_depth->push_back(m);
    }
  }
  std::sort(matches_with_depth->begin(), matches_with_depth->end()); //sort by distance, which is the nn_ratio

  int real_iterations = 0;
  for(int n = 0; (n < ransac_iterations && matches_with_depth->size() >= sample_size); n++) //Without the minimum number of matches, the transformation can not be computed as usual TODO: implement monocular motion est
  {
    //Initialize Results of refinement
    double refined_error = 1e6;
    std::vector<cv::DMatch> refined_matches; 
    std::vector<cv::DMatch> inlier = sample_matches_prefer_by_distance(sample_size, *matches_with_depth); //initialization with random samples 
    //std::vector<cv::DMatch> inlier = sample_matches(sample_size, *matches_with_depth); //initialization with random samples 
    Eigen::Matrix4f refined_transformation = Eigen::Matrix4f::Identity();

    real_iterations++;
    for(int refinements = 1; refinements < 20 /*got stuck?*/; refinements++) 
    {
        Eigen::Matrix4f transformation = getTransformFromMatches(this, earlier_node, inlier,valid_tf,max_dist_m);
        //Eigen::Matrix4f transformation = getTransformFromMatchesUmeyama(this, earlier_node, inlier,valid_tf);
        if (!valid_tf || transformation!=transformation)  //Trafo Contains NaN?
          break; // valid_tf is false iff the sampled points aren't inliers themself 

        //test which features are inliers 
        computeInliersAndError(*initial_matches, transformation, 
                               this->feature_locations_3d_, //this->feature_depth_stats_, 
                               earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                               std::max(min_inlier_threshold, static_cast<unsigned int>(refined_matches.size())), //break if no chance to reach this amount of inliers
                               inlier, inlier_error, max_dist_m*max_dist_m); 
        
        if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
          ROS_DEBUG("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
          break; //hopeless case
        }

        //superior to before?
        if (inlier.size() >= refined_matches.size() && inlier_error <= refined_error) {
          size_t prev_num_inliers = refined_matches.size();
          assert(inlier_error>=0);
          refined_transformation = transformation;
          refined_matches = inlier;
          refined_error = inlier_error;
          if(inlier.size() == prev_num_inliers) break; //only error improved -> no change would happen next iteration
        }
        else break;
    }  //END REFINEMENTS
    //Successful Iteration?
    if(refined_matches.size() > 0){ //Valid?
        valid_iterations++;
        ROS_DEBUG("Valid iteration: inliers/matches: %lu/%lu (min %u), refined error: %.2f (max %.2f), global error: %.2f", 
                refined_matches.size(), matches.size(), min_inlier_threshold,  refined_error, max_dist_m, rmse);

        //Acceptable && superior to previous iterations?
        if (refined_error <= rmse &&  
            refined_matches.size() >= matches.size() && 
            refined_matches.size() >= min_inlier_threshold)
        {
          ROS_DEBUG("%s: Improvment in iteration %d: inliers: %i (min %i), inlier_error: %.2f (max %.2f)",nodesstring.c_str(), real_iterations, (int)refined_matches.size(), (int) min_inlier_threshold,  refined_error, max_dist_m);
          rmse = refined_error;
          resulting_transformation = refined_transformation;
          matches.assign(refined_matches.begin(), refined_matches.end());
          //Performance hacks:
          if (refined_matches.size() > initial_matches->size()*0.5) n+=10;///Iterations with more than half of the initial_matches inlying, count tenfold
          if (refined_matches.size() > initial_matches->size()*0.75) n+=10;///Iterations with more than 3/4 of the initial_matches inlying, count twentyfold
          if (refined_matches.size() > initial_matches->size()*0.8) break; ///Can this get better anyhow?
        }
    }
  } //iterations
  if(valid_iterations == 0) // maybe no depth. Try identity?
  { 
    //IDENTITYTEST
    ROS_INFO("Last Resort: Trying identity as hypothesis");
    //1 ransac iteration with identity
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();//hypothesis
    std::vector<cv::DMatch> inlier; //result
    //test which samples are inliers 
    computeInliersAndError(*initial_matches, transformation, 
                           this->feature_locations_3d_, //this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                           min_inlier_threshold, //break if no chance to reach this amount of inliers
                           inlier, inlier_error, max_dist_m*max_dist_m); 
    
    //superior to before?
    if (inlier.size() > min_inlier_threshold && inlier_error < max_dist_m) {
      assert(inlier_error>=0);
      resulting_transformation = transformation;
      matches.assign(inlier.begin(), inlier.end());
      rmse = inlier_error;
      valid_iterations++;
      ROS_INFO("No-Motion guess for %i<->%i: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", this->id_, earlier_node->id_, (int)matches.size(), (int) min_inlier_threshold,  rmse, max_dist_m);
    }
  } //END IDENTITY AS GUESS
  ROS_INFO("%i good iterations (from %i), inlier pct %i, inlier cnt: %i, error (MHD): %.2f",valid_iterations, ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse);
  
  //ROS_INFO_STREAM("Transformation estimated:\n" << resulting_transformation);
  



  //G2O Refinement (minimize mahalanobis distance, include depthless features in optimization)
  //Optimize transform based on latest inliers (in "matches") and initial guess (in "resulting_transformation")
  const int g2o_iterations = ParameterServer::instance()->get<int>( "g2o_transformation_refinement");
  if(g2o_iterations > 0 && matches.size() > min_inlier_threshold)//Do not do this if RANSAC didn't succeed. Can't handle outliers
  {
    Eigen::Matrix4f transformation = resulting_transformation;//current hypothesis
    getTransformFromMatchesG2O(earlier_node, this,matches, transformation, g2o_iterations);
    std::vector<cv::DMatch> inlier; //result

    //Evaluate the new transformation
    computeInliersAndError(*initial_matches, transformation, 
                           this->feature_locations_3d_, //this->feature_depth_stats_, 
                           earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                           matches.size(), //minimum to reach
                           inlier,inlier_error, //Output!
                           max_dist_m*max_dist_m); 
    ROS_INFO_STREAM(nodesstring << ": g2o transformation estimated to Node " << earlier_node->id_ << ":\n" << transformation);
    //superior in inliers or equal inliers and better rmse?
    if (inlier.size() >= matches.size() || inlier.size() >= min_inlier_threshold && inlier_error < rmse) {
      //if More inliers -> Refine with them included
      if (inlier.size() > matches.size()) {
        //Refine using the new inliers
        getTransformFromMatchesG2O(earlier_node, this,inlier, transformation, g2o_iterations);
        computeInliersAndError(*initial_matches, transformation, 
                               this->feature_locations_3d_, //this->feature_depth_stats_, 
                               earlier_node->feature_locations_3d_, //earlier_node->feature_depth_stats_, 
                               inlier.size(), //minimum to reach
                               inlier,inlier_error, max_dist_m*max_dist_m); 
      }
      ROS_INFO("G2o optimization result for %i<->%i: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", this->id_, earlier_node->id_, (int)inlier.size(), (int) min_inlier_threshold,  inlier_error, max_dist_m);
      //Again superier? Then use the new result
      if (inlier.size() >= matches.size()) 
      {
        ROS_INFO_STREAM("Refined transformation estimate" << earlier_node->id_ << ":\n" << transformation);
        assert(inlier_error>=0);
        resulting_transformation = transformation;
        matches.assign(inlier.begin(), inlier.end());
        rmse = inlier_error;
        valid_iterations++;
        ROS_INFO("G2o refinement optimization result for %i<->%i: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", this->id_, earlier_node->id_, (int)matches.size(), (int) min_inlier_threshold,  rmse, max_dist_m);
      }
    }
    else {
      ROS_INFO("G2O optimization of RANSAC for %s rejected: G2O inliers: %i (min %i), inlier_error: %.2f (max %.2f) RANSAC inls: %i (min %i), inlier_error: %.2f", nodesstring.c_str(), (int)inlier.size(), (int) min_inlier_threshold,  inlier_error, max_dist_m, (int)matches.size(), (int) min_inlier_threshold,  rmse);
    }
  }
  


  ROS_INFO("%s: %i good iterations (from %i), inlier pct %i, inlier cnt: %i, error (MHD): %.2f",nodesstring.c_str(), valid_iterations, ransac_iterations, (int) (matches.size()*1.0/initial_matches->size()*100),(int) matches.size(),rmse);
  // ROS_INFO("best overall: inlier: %i, error: %.2f",best_inlier_invalid, best_error_invalid*100);

  bool enough_absolute = matches.size() >= min_inlier_threshold;
  return enough_absolute;
}


#ifdef USE_ICP_CODE
void Node::Eigen2GICP(const Eigen::Matrix4f& m, dgc_transform_t g_m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      g_m[i][j] = m(i,j);

}
void Node::GICP2Eigen(const dgc_transform_t g_m, Eigen::Matrix4f& m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      m(i,j) = g_m[i][j];
}

void Node::gicpSetIdentity(dgc_transform_t m){
  for(int i=0;i<4; i++)
    for(int j=0;j<4; j++)
      if (i==j)
        m[i][j] = 1;
      else
        m[i][j] = 0;
}
#endif



MatchingResult Node::matchNodePair(const Node* older_node)
{
  MatchingResult mr;
  try{
    bool found_transformation = false;
    if(ParameterServer::instance()->get<int>("max_connections") > 0 &&
       initial_node_matches_ > ParameterServer::instance()->get<int>("max_connections")) 
      return mr; //enough is enough
    const unsigned int min_matches = (unsigned int) ParameterServer::instance()->get<int>("min_matches");// minimal number of feature correspondences to be a valid candidate for a link

    this->featureMatching(older_node, &mr.all_matches); 

    double ransac_quality = 0;
    ROS_DEBUG_NAMED(__FILE__, "found %i inital matches",(int) mr.all_matches.size());
    if (mr.all_matches.size() < min_matches){
        ROS_INFO("Too few inliers between %i and %i for RANSAC method. Only %i correspondences to begin with.",
                 older_node->id_,this->id_,(int)mr.all_matches.size());
    } 
    else {//All good for feature based transformation estimation
        found_transformation = getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches); 
        //Statistics
        float nn_ratio = 0.0;
        if(found_transformation){
          //double w = 1.0 + (double)mr.inlier_matches.size()-(double)min_matches;///(double)mr.all_matches.size();
          for(unsigned int i = 0; i < mr.inlier_matches.size(); i++){
            nn_ratio += mr.inlier_matches[i].distance;
          }
          nn_ratio /= mr.inlier_matches.size();
          //ParameterServer::instance()->set("nn_distance_ratio", static_cast<double>(nn_ratio + 0.2));
          mr.final_trafo = mr.ransac_trafo;
          mr.edge.informationMatrix =   Eigen::Matrix<double,6,6>::Identity()*(mr.inlier_matches.size()/(mr.rmse*mr.rmse)); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)

          mr.edge.id1 = older_node->id_;//and we have a valid transformation
          mr.edge.id2 = this->id_; //since there are enough matching features,
          mr.edge.transform = mr.final_trafo.cast<double>();//we insert an edge between the frames
          if(ParameterServer::instance()->get<double>("observability_threshold") > 0){
            pairwiseObservationLikelihood(this, older_node, mr);
            found_transformation = observation_criterion_met(mr.inlier_points, mr.outlier_points, mr.occluded_points + mr.inlier_points + mr.outlier_points, ransac_quality);
          } else {
            found_transformation = true;
          }

          ROS_INFO("RANSAC found a %s transformation with %d inliers matches with average ratio %f", found_transformation? "valid" : "invalid", (int) mr.inlier_matches.size(), nn_ratio);

        } else {
          for(unsigned int i = 0; i < mr.all_matches.size(); i++){
            nn_ratio += mr.all_matches[i].distance;
          }
          nn_ratio /= mr.all_matches.size();
          ROS_INFO("RANSAC found no valid trafo, but had initially %d feature matches with average ratio %f",(int) mr.all_matches.size(), nn_ratio);

#ifdef USE_PCL_ICP
          if(((int)this->id_ - (int)older_node->id_) <= 1){ //Apply icp only for adjacent frames, as the initial guess needs to be in the global minimum
            mr.icp_trafo = Eigen::Matrix4f::Identity();
            int max_count = ParameterServer::instance()->get<int>("gicp_max_cloud_size");
            pointcloud_type::Ptr tmp1(new pointcloud_type());
            pointcloud_type::Ptr tmp2(new pointcloud_type());
            filterCloud(*pc_col, *tmp1, max_count);
            filterCloud(*(older_node->pc_col), *tmp2, max_count);
            mr.icp_trafo = icpAlignment(tmp2, tmp1, mr.icp_trafo);
            //pairwiseObservationLikelihood(this, older_node, mr);
            //double icp_quality;
            found_transformation = true; //observation_criterion_met(mr.inlier_points, mr.outlier_points, mr.occluded_points + mr.inlier_points + mr.outlier_points, icp_quality);
            if(found_transformation){
              ROS_ERROR("Found trafo via icp");
              mr.final_trafo = mr.icp_trafo;
              mr.edge.id1 = older_node->id_;//and we have a valid transformation
              mr.edge.id2 = this->id_; //since there are enough matching features,
              mr.edge.transform = mr.final_trafo.cast<double>();//we insert an edge between the frames
              std::cout << std::endl << mr.final_trafo << std::endl;
            }
          }
#endif
        }
    } 
    
#ifdef USE_ICP_CODE
    unsigned int ransac_inlier_points = 0, ransac_outlier_points = 0;
    if(ParameterServer::instance()->get<bool>("use_icp")){
        if((!found_transformation && (((int)this->id_ - (int)older_node->id_) <= 1)) //Apply icp only for adjacent frames, as the initial guess needs to be in the global minimum
            || found_transformation) //Apply icp only for adjacent frames, as the initial guess needs to be in the global minimum
           //|| (!found_transformation && initial_node_matches_ == 0)) //no matches were found, and frames are not too far apart => identity is a good initial guess.
        {
            ROS_INFO("Applying GICP for Transformation between Nodes %d and %d",this->id_ , older_node->id_);
            MatchingResult mr_icp;
            if(getRelativeTransformationTo_ICP_code(older_node,mr_icp.icp_trafo, mr.ransac_trafo) && //converged
               !((mr_icp.icp_trafo.array() != mr_icp.icp_trafo.array()).any())) //No NaNs
            {
                ROS_INFO("GICP for Nodes %u and %u Successful", this->id_, older_node->id_);
                double icp_quality;

                bool no_obs_test = (ParameterServer::instance()->get<double>("observability_threshold") < 0);
                pairwiseObservationLikelihood(this, older_node, mr_icp);
                if(no_obs_test || 
                    (observation_criterion_met(mr_icp.inlier_points, mr_icp.outlier_points, mr_icp.occluded_points + mr_icp.inlier_points + mr_icp.outlier_points, icp_quality)
                     && icp_quality > ransac_quality))
                {
                    //This signals a valid result:
                    found_transformation = true;
                    mr.edge.id1 = older_node->id_;//and we have a valid transformation
                    mr.edge.id2 = this->id_; //since there are enough matching features,
                    mr.final_trafo = mr_icp.icp_trafo;    
                    mr.edge.informationMatrix = Eigen::Matrix<double,6,6>::Identity()*(1e-2); //TODO: What do we do about the information matrix? 
                    mr.edge.mean = eigen2G2O(mr.final_trafo.cast<double>());//we insert an edge between the frames
                }
            }
        }
    }
#endif  

    if(found_transformation) {
        ROS_INFO("Returning Valid Edge");
        ++initial_node_matches_; //trafo is accepted
    }
    else {
        mr.edge.id1 = -1;
        mr.edge.id2 = -1;
    }
  }
  catch (std::exception e){
    ROS_ERROR("Caught Exception in comparison of Nodes %i and %i: %s", this->id_, older_node->id_, e.what());
  }

  return mr;
}

void Node::clearFeatureInformation(){
  //clear feature info, by swapping data with empty vector (so mem really gets freed)
  ROS_INFO("Deleting feature information of Node %i", this->id_);
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > f_l_3d;  
  f_l_3d.swap(feature_locations_3d_);
	std::vector<cv::KeyPoint> f_l_2d; 
  f_l_2d.swap(feature_locations_2d_);
	std::vector<float> f_l_siftgpu; 
  f_l_siftgpu.swap(siftgpu_descriptors);
  feature_descriptors_.release();
  delete flannIndex; flannIndex = NULL;
  matchable_ = false;
}
void Node::addPointCloud(pointcloud_type::Ptr new_pc){
  pc_col = new_pc;
  header_=pc_col->header;
}
void Node::reducePointCloud(double vfs){
  if(vfs > 0.0){
    ROS_INFO("Reducing points (%d) of Node %d", (int)pc_col->size(), this->id_);
    pcl::VoxelGrid<point_type> sor;
    sor.setLeafSize(vfs,vfs,vfs);
    pointcloud_type::ConstPtr const_cloud_ptr = boost::make_shared<pointcloud_type> (*pc_col);                                                                 
    sor.setInputCloud (const_cloud_ptr);
    sor.filter (*pc_col);
    ROS_INFO("Reduced points of Node %d to %d", this->id_, (int)pc_col->size());
  } else {
    ROS_WARN("Point Clouds can't be reduced because of invalid voxelfilter_size");
  }
}
long Node::getMemoryFootprint(bool write_to_log) const
{
  size_t size = 0, tmp = 0;
  tmp = sizeof(Node);
  ROS_INFO_COND(write_to_log, "Base Size of Node Class: %zu bytes", tmp); 
  size += tmp;

  tmp = feature_descriptors_.step * feature_descriptors_.rows;  
  ROS_INFO_COND(write_to_log, "Descriptor Information: %zu bytes", tmp);
  size += tmp;

  tmp = siftgpu_descriptors.size() * sizeof(float);
  ROS_INFO_COND(write_to_log, "SIFTGPU Descriptor Information: %zu bytes", tmp);
  size += tmp;

  tmp = feature_locations_2d_.size() * sizeof(cv::KeyPoint);
  ROS_INFO_COND(write_to_log, "Feature 2D Location Information: %zu bytes", tmp);
  size += tmp;

  tmp = feature_locations_3d_.size() * sizeof(Eigen::Vector4f);
  ROS_INFO_COND(write_to_log, "Feature 3D Location Information: %zu bytes", tmp);
  size += tmp;

  tmp = pc_col->size() * sizeof(point_type);
  ROS_INFO_COND(write_to_log, "Point Cloud: %zu bytes", tmp);
  size += tmp;
  ROS_INFO_COND(write_to_log, "Rough Summary: %zu Kbytes", size/1024);
  ROS_WARN("Rough Summary: %zu Kbytes", size/1024);
  return size;
}
void Node::clearPointCloud(){
    ROS_INFO("Deleting points of Node %i", this->id_);
    //clear only points, by swapping data with empty vector (so mem really gets freed)
    pc_col->width = 0;
    pc_col->height = 0;
    pointcloud_type pc_empty;
    pc_empty.points.swap(pc_col->points);
}

/*TODO use this to discount features at depth jumps (or duplicate them -> sensed position + minimum position
void Node::computeKeypointDepthStats(const cv::Mat& depth_img, const std::vector<cv::KeyPoint> keypoints)
{
    ROS_INFO("Computing Keypoint Depth Statistics");
    BOOST_FOREACH(cv::KeyPoint kp, keypoints)
    { 
      int radius = kp.size/2;
      int left = kp.pt.x-radius;
      int top  = kp.pt.y-radius;
      double nearest=0.0, farthest=0.0;
      cv::Mat keypoint_neighbourhood(depth_img, cv::Rect(left, top, (int)kp.size, (int)kp.size));
      ROS_DEBUG("Nearest: %f, Farthest: %f", nearest, farthest);
      if(isnan(nearest)) nearest = 1.0;
      if(isnan(farthest)) farthest = 10.0;
      cv::minMaxLoc(keypoint_neighbourhood, &nearest, &farthest);
      feature_depth_stats_.push_back(std::make_pair(nearest, farthest)); 
    }
}
*/

void pairwiseObservationLikelihood(const Node* newer_node, const Node* older_node, MatchingResult& mr)
{ 
      double likelihood, confidence;
      unsigned int inlier_points = 0, outlier_points = 0, all_points = 0, occluded_points = 0;
      #pragma omp parallel sections reduction (+: inlier_points, outlier_points, all_points, occluded_points)
      {
        #pragma omp section
        {
          unsigned int inlier_pts = 0, outlier_pts = 0, occluded_pts = 0, all_pts = 0;
          observationLikelihood(mr.final_trafo, newer_node->pc_col, older_node->pc_col, older_node->getCamInfo(), likelihood, confidence, inlier_pts, outlier_pts, occluded_pts, all_pts) ;
          ROS_INFO("Observation Likelihood: %d projected to %d: good_point_ratio: %d/%d: %g, occluded points: %d", newer_node->id_, older_node->id_, inlier_pts, inlier_pts+outlier_pts, ((float)inlier_pts)/(inlier_pts+outlier_pts), occluded_pts);
          //rejectionSignificance(mr.final_trafo, newer_node->pc_col, older_node->pc_col);
          inlier_points += inlier_pts;
          outlier_points += outlier_pts;
          occluded_points += occluded_pts;
          all_points += all_pts;
        }

        #pragma omp section
        {
          unsigned int inlier_pts = 0, outlier_pts = 0, occluded_pts = 0, all_pts = 0;
          observationLikelihood(mr.final_trafo.inverse(), older_node->pc_col, newer_node->pc_col, newer_node->getCamInfo(), likelihood, confidence, inlier_pts, outlier_pts, occluded_pts, all_pts) ;
          ROS_INFO("Observation Likelihood: %d projected to %d: good_point_ratio: %d/%d: %g, occluded points: %d", older_node->id_, newer_node->id_, inlier_pts, inlier_pts+outlier_pts, ((float)inlier_pts)/(inlier_pts+outlier_pts), occluded_pts);
          //rejectionSignificance(mr.final_trafo, newer_node->pc_col, older_node->pc_col);
          inlier_points += inlier_pts;
          outlier_points += outlier_pts;
          occluded_points += occluded_pts;
          all_points += all_pts;
        }
      }
      mr.inlier_points = inlier_points;
      mr.outlier_points = outlier_points;
      mr.occluded_points = occluded_points;
      mr.all_points = all_points;
}

///Compute the RootSIFT from SIFT according to Arandjelovic and Zisserman
void squareroot_descriptor_space(cv::Mat& descriptors)
{
  // Compute sums for L1 Norm
  cv::Mat sums_vec;
  descriptors = cv::abs(descriptors); //otherwise we draw sqrt of negative vals
  cv::reduce(descriptors, sums_vec, 1 /*sum over columns*/, CV_REDUCE_SUM, CV_32FC1);
  for(unsigned int row = 0; row < descriptors.rows; row++){
    if(sums_vec.at<float>(row) == 0) continue; //Do not normalize norm-zero vectors
    int offset = row*descriptors.cols;
    for(unsigned int col = 0; col < descriptors.cols; col++){
      descriptors.at<float>(offset + col) = 
        sqrt(descriptors.at<float>(offset + col) / sums_vec.at<float>(row) /*L1-Normalize*/);
    }
  }
}

void Node::knnSearch(cv::Mat& query,
                     cv::Mat& indices,
                     cv::Mat& dists,
                     int knn, 
                     const cv::flann::SearchParams& params) const
{
  this->getFlannIndex();//make sure it is constructed (cannot use it directly b/c of constness
  flannIndex->knnSearch(query, indices, dists, knn, params);
}

void copy_filter_cloud (const Eigen::Vector3f& center, float radius, const Node* old_node, Node* clone)
{
  float sq_radius = radius*radius;
  BOOST_FOREACH(const point_type& p, old_node->pc_col->points)
  {
    float sq_distance = (p.getVector3fMap() - center).squaredNorm();
    if(sq_distance <= sq_radius){
      clone->pc_col->push_back(p);
    }
  }
  ROS_INFO("Copied %zu/%zu points to Node clone", clone->pc_col->size(), old_node->pc_col->size());
}

void copy_filter_features (const Eigen::Vector3f& center, float radius, const Node* old_node, Node* clone)
{
  assert(old_node->feature_locations_3d_.size() == old_node->feature_locations_2d_.size());
  float sq_radius = radius*radius;

  ROS_DEBUG_STREAM("Center: " << center);
  //Determine which features qualify
  std::list<size_t> featuresToCopy;
  for(size_t i = 0; i < old_node->feature_locations_3d_.size(); i++){
    float sq_distance = (old_node->feature_locations_3d_.at(i).head<3>() - center).squaredNorm();
    ROS_DEBUG("Sq. distance of feature %zu: %f/%f", i, sq_distance, sq_radius);
    if(sq_distance <= sq_radius){
      featuresToCopy.push_back(i);
    }
  }
  ROS_INFO("Copying %zu/%zu features to Node clone", featuresToCopy.size(), old_node->feature_locations_3d_.size());

  //Copy qualified features
  size_t size = featuresToCopy.size();
  clone->feature_locations_3d_.reserve(size);
  clone->feature_locations_2d_.reserve(size);
  clone->feature_matching_stats_.reserve(size);
  clone->feature_descriptors_ = cv::Mat(size, 128, CV_32F);
  clone->siftgpu_descriptors.resize(size * 128);
  size_t i = 0; // the index in the clone
  BOOST_FOREACH(size_t usedFeatureIndex, featuresToCopy)
  {
      clone->feature_locations_3d_[i] = old_node->feature_locations_3d_[usedFeatureIndex];
      clone->feature_locations_2d_[i] = old_node->feature_locations_2d_[usedFeatureIndex];
      clone->feature_matching_stats_[i] = old_node->feature_matching_stats_[usedFeatureIndex];
      clone->feature_descriptors_.row(i) = old_node->feature_descriptors_.row(usedFeatureIndex);
      for(int j = 0; j < 128; j++){
        clone->siftgpu_descriptors[j+i]  = old_node->siftgpu_descriptors[j+usedFeatureIndex];
      }
      i++;
  }
}

Node* Node::copy_filtered(const Eigen::Vector3f& center, float radius) const
{
  Node* clone = new Node();
  //Copy-filter point cloud
  clone->pc_col = pointcloud_type::Ptr(new pointcloud_type());
  copy_filter_cloud(center, radius, this, clone);
  copy_filter_features(center, radius, this, clone);
  this->getMemoryFootprint(true);
  clone->getMemoryFootprint(true);
  return clone;
}
