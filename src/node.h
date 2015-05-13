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


#ifndef RGBD_SLAM_NODE_H_
#define RGBD_SLAM_NODE_H_

#ifdef HEMACLOUDS
#define PCL_NO_PRECOMPILE
#endif

#include "ros/ros.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
//#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
/*
#include <pcl/registration/icp.h>
#include <pcl/registration/impl/icp.hpp>
#include <pcl/registration/registration.h>
#include <pcl/registration/impl/registration.hpp>
*/
#include "parameter_server.h"
//for ground truth
#include <tf/transform_datatypes.h>
#include <QMutex>

#ifdef USE_ICP_BIN
#include "gicp-fallback.h"
#endif

#ifdef USE_ICP_CODE
#include "gicp/gicp.h"
#include "gicp/transform.h"
#endif

#include "matching_result.h" 
#include <Eigen/StdVector>
#include "header.h"

typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > std_vector_of_eigen_vector4f;
//!Holds the data for one graph node and provides functionality to compute relative transformations to other Nodes.
class Node {
public:
	///Visual must be CV_8UC1, depth CV_32FC1, 
	///detection_mask must be CV_8UC1 with non-zero 
	///at potential keypoint locations
	Node(const cv::Mat& visual,
			 const cv::Mat& depth,
			 const cv::Mat& detection_mask,
       const sensor_msgs::CameraInfoConstPtr& cam_info, 
       myHeader depth_header,
			 cv::Ptr<cv::FeatureDetector> detector,
			 cv::Ptr<cv::DescriptorExtractor> extractor);
	///Visual must be CV_8UC1
	///detection_mask must be CV_8UC1 with non-zero 
	///at potential keypoint locations
	Node(const cv::Mat visual,
			 cv::Ptr<cv::FeatureDetector> detector,
			 cv::Ptr<cv::DescriptorExtractor> extractor,
			 pointcloud_type::Ptr point_cloud,
			 const cv::Mat detection_mask = cv::Mat());
	//default constructor. TODO: still needed?
	Node(){}
	///Delete the flannIndex if built
	~Node();

	///Compare the features of two nodes and compute the transformation
  MatchingResult matchNodePair(const Node* older_node);
  //MatchingResult matchNodePair2(const Node* older_node);

  ///Transform, e.g., from Joint/Wheel odometry
  void setOdomTransform(tf::StampedTransform odom);
  ///Transform, e.g., from MoCap
  void setGroundTruthTransform(tf::StampedTransform gt);
  ///Transform, e.g., from kinematics
  void setBase2PointsTransform(tf::StampedTransform& b2p);
  ///Transform, e.g., from Joint/Wheel odometry
  tf::StampedTransform getOdomTransform() const;
  ///Transform, e.g., from MoCap
  tf::StampedTransform getGroundTruthTransform() const;
  ///Transform, e.g., from kinematics
  tf::StampedTransform getBase2PointsTransform() const;

	///Compute the relative transformation between the nodes
	bool getRelativeTransformationTo(const Node* target_node, 
			std::vector<cv::DMatch>* initial_matches,
			Eigen::Matrix4f& resulting_transformation, 
			float& rmse,
			std::vector<cv::DMatch>& matches) const;

#ifdef USE_ICP_BIN
	// initial_transformation: optional transformation applied to this->pc before
	// using icp
	bool getRelativeTransformationTo_ICP_bin(const Node* target_node,Eigen::Matrix4f& transformation,
			const Eigen::Matrix4f* initial_transformation = NULL);
#endif
	
  //!Fills "matches" and returns ratio of "good" features 
  //!in the sense of distinction via the "nn_distance_ratio" setting (see parameter server)
	unsigned int featureMatching(const Node* other, std::vector<cv::DMatch>* matches) const;

#ifdef USE_ICP_CODE
	bool getRelativeTransformationTo_ICP_code(const Node* target_node,
                                            Eigen::Matrix4f& transformation,
                                            const Eigen::Matrix4f& initial_transformation);
	
	static const double gicp_epsilon = 1e-3;
	static const double gicp_d_max_ = 5.0; // 10cm
	static const int gicp_max_iterations = 10;
	static const int gicp_min_point_cnt = 100;
		
	bool gicp_initialized;
	void Eigen2GICP(const Eigen::Matrix4f& m, dgc_transform_t g_m);
	void GICP2Eigen(const dgc_transform_t g_m, Eigen::Matrix4f& m);
	void gicpSetIdentity(dgc_transform_t m);
  dgc::gicp::GICPPointSet* getGICPStructure(unsigned int max_count = 0) const;
  void clearGICPStructure() const;
  protected:
    mutable dgc::gicp::GICPPointSet* gicp_point_set_;

  public:
#endif

  ///Center needs to be in the coordinate frame of the node (i.e. the camera is 0,0,0)
  Node* copy_filtered(const Eigen::Vector3f& center, float radius) const;

  void clearFeatureInformation();
  void addPointCloud(pointcloud_type::Ptr pc_col);


  //!erase the points from the cloud to save memory
  void clearPointCloud();
  //!reduce the points from the cloud using a voxelgrid_filter to save memory
  void reducePointCloud(double voxelfilter_size);
	//PointCloud pc;
	///pointcloud_type centrally defines what the pc is templated on
  int id_;         //<number of camera nodes in the graph when the node was added
	int seq_id_;      //<number of images that have been processed (even if they were not added)
	int vertex_id_;   //<id of the corresponding vertex in the g2o graph
  bool valid_tf_estimate_;      //<Flags whether the data of this node should be considered for postprocessing steps, e.g., visualization, trajectory, map creation
  bool matchable_;        //< Flags whether the data for matching is (still) available
  ros::Time timestamp_; 		//< Time of the data belonging to the node
  bool has_odometry_edge_;
  bool odometry_set_;
  pointcloud_type::Ptr pc_col;
#ifdef USE_PCL_ICP
  pointcloud_type::Ptr filtered_pc_col; //<Used for icp. May not contain NaN
#endif
  ///descriptor definitions
	cv::Mat feature_descriptors_;         

  ///backprojected 3d descriptor locations relative to cam position in homogeneous coordinates (last dimension is 1.0)
	std_vector_of_eigen_vector4f feature_locations_3d_;  
	std::vector<float> siftgpu_descriptors;

  ///Where in the image are the descriptors
	std::vector<cv::KeyPoint> feature_locations_2d_; 
  ///Contains the minimum and maximum depth in the feature's range (not used yet)
  std::vector<std::pair<float, float> > feature_depth_stats_;
  ///Contains how often a feature has been an (inlier) match
  std::vector<unsigned char> feature_matching_stats_;
#ifdef HEMACLOUDS
  ///Contains a weight derived from external information in the clouds (i.e. only used for hemas clouds)
  std::vector<float> feature_weights_;
#endif

#ifdef  DO_FEATURE_OPTIMIZATION

  std::map<int, int> kpt_to_landmark;
#endif

  long getMemoryFootprint(bool print) const;
  void knnSearch(cv::Mat& query,
                 cv::Mat& indices,
                 cv::Mat& dists,
                 int knn, 
                 const cv::flann::SearchParams& params) const;

  myHeader header_;
  const sensor_msgs::CameraInfo& getCamInfo() const {return cam_info_;}

protected:
  const cv::flann::Index* getFlannIndex() const;
  static QMutex gicp_mutex;
  static QMutex siftgpu_mutex;
	mutable cv::flann::Index* flannIndex;
  tf::StampedTransform base2points_; //!<contains the transformation from the base (defined on param server) to the point_cloud
  tf::StampedTransform ground_truth_transform_;//!<contains the transformation from the mocap system
  tf::StampedTransform odom_transform_;        //!<contains the transformation from the wheel encoders/joint states
  int initial_node_matches_;
  sensor_msgs::CameraInfo cam_info_; 
  //void computeKeypointDepthStats(const cv::Mat& depth_img, const std::vector<cv::KeyPoint> keypoints);

#ifdef USE_SIFT_GPU
	//! return the 3D projection of valid keypoints using information from the point cloud and remove invalid keypoints (NaN depth) 
	void projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
                          std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                            const pointcloud_type::Ptr point_cloud, 
                            std::vector<float>& descriptors_in, cv::Mat& descriptors_out);
	//! return the 3D projection of valid keypoints using information from the depth image and remove invalid keypoints (NaN depth) 
	void projectTo3DSiftGPU(std::vector<cv::KeyPoint>& feature_locations_2d,
                          std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                          const cv::Mat& depth,
                          const sensor_msgs::CameraInfoConstPtr& cam_info,
                          std::vector<float>& descriptors_in, cv::Mat& descriptors_out);
#endif
	//! return the 3D projection of valid keypoints using information from the point cloud and remove invalid keypoints (NaN depth) 
	void projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
                   std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                   const pointcloud_type::ConstPtr point_cloud);
	//! return the 3D projection of valid keypoints using information from the depth image and remove invalid keypoints (NaN depth) 
	void projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
                   std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
                   const cv::Mat& depth,
                   const sensor_msgs::CameraInfoConstPtr& cam_info);


	///Get the norm of the translational part of an affine matrix (Helper for isBigTrafo)
	void mat2dist(const Eigen::Matrix4f& t, double &dist){
		dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
	}

  
  ///Retrieves and stores the transformation from base to point cloud at capturing time 
  void retrieveBase2CamTransformation();
	// helper for ransac
	void computeInliersAndError(const std::vector<cv::DMatch> & initial_matches,
                              const Eigen::Matrix4f& transformation,
                              const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
                              const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& targets,
                              size_t min_inliers, //break if this can't be reached
                              std::vector<cv::DMatch>& new_inliers, //pure output var
                              double& mean_error, //pure output var //std::vector<double>& errors,
                              double squaredMaxInlierDistInM = 0.0009) const; //output var;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

void pairwiseObservationLikelihood(const Node* newer_node, const Node* older_node, MatchingResult& mr);
///Compute the RootSIFT from SIFT according to Arandjelovic and Zisserman
void squareroot_descriptor_space(cv::Mat& feature_descriptors);
// Compute the transformation from matches using pcl::TransformationFromCorrespondences
#endif
