#ifndef RGBD_SLAM_MISC_H_
#define RGBD_SLAM_MISC_H_
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
#include <QMatrix4x4>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv.h>
#include "point_types.h"
#include "g2o/types/slam3d/vertex_se3.h"

///Overlay the monochrom edges and depth jumps
void overlay_edges(cv::Mat visual, cv::Mat depth, cv::Mat& visual_edges, cv::Mat& depth_edges);
///Print tf::Transform via ROS_INFO
void printTransform(const char* name, const tf::Transform t) ;
///Print tf::Transform via ROS_INFO
void printTransform(const char* name, const tf::StampedTransform t) ;
///Write Transformation to textstream
void logTransform(QTextStream& out, const tf::Transform& t, double timestamp, const char* label = NULL);
void printQMatrix4x4(const char* name, const QMatrix4x4& m);

///Conversion Function
QMatrix4x4    g2o2QMatrix(const g2o::SE3Quat se3) ;
///Conversion Function
tf::Transform g2o2TF(     const g2o::SE3Quat se3) ;
template <typename T >
QMatrix4x4 eigenTF2QMatrix(const T& transf) 
{
  Eigen::Matrix<qreal, 4, 4, Eigen::RowMajor> m = transf.matrix();
  QMatrix4x4 qmat( static_cast<qreal*>( m.data() )  );
  printQMatrix4x4("From Eigen::Transform", qmat); 
  return qmat;
}

template <typename T >
tf::Transform eigenTransf2TF(const T& transf) 
{
    tf::Transform result;
    tf::Vector3 translation;
    translation.setX(transf.translation().x());
    translation.setY(transf.translation().y());
    translation.setZ(transf.translation().z());

    tf::Quaternion rotation;
    Eigen::Quaterniond quat;
    quat = transf.rotation();
    rotation.setX(quat.x());
    rotation.setY(quat.y());
    rotation.setZ(quat.z());
    rotation.setW(quat.w());

    result.setOrigin(translation);
    result.setRotation(rotation);
    //printTransform("from conversion", result);
    return result;
}
///Conversion Function
g2o::SE3Quat  eigen2G2O(  const Eigen::Matrix4d& eigen_mat);
///Conversion Function
g2o::SE3Quat  tf2G2O(     const tf::Transform t);

/// get euler angles and translation from 4x4 homogenous
void mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist);
/// get euler angles from 4x4 homogenous
void mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw);
/// get translation-distance from 4x4 homogenous
void mat2dist(const Eigen::Matrix4f& t, double &dist);


///Creates a pointcloud from rgb8 or mono8 coded images + float depth
pointcloud_type* createXYZRGBPointCloud (const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& cam_info); 
pointcloud_type* createXYZRGBPointCloud (const cv::Mat& depth_msg, const cv::Mat& rgb_msg, const sensor_msgs::CameraInfoConstPtr& cam_info); 

#ifndef HEMACLOUDS
///Helper function to aggregate pointclouds in a single coordinate frame. idx is only for compatibilty to the
///Alternative function below
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, pointcloud_type &cloud_to_append_to,
                                   const tf::Transform transformation, float Max_Depth, int idx=0);

#else
///Same as other but different output cloud type
void transformAndAppendPointCloud (const pointcloud_type &cloud_in, 
                                   pcl::PointCloud<hema::PointXYZRGBCamSL> &cloud_to_append_to,
                                   const tf::Transform transformation, float max_Depth, int idx);
#endif

//geometry_msgs::Point pointInWorldFrame(const Eigen::Vector4f& point3d, g2o::SE3Quat transf);
geometry_msgs::Point pointInWorldFrame(const Eigen::Vector4f& point3d, const g2o::VertexSE3::EstimateType& transf);
// true if translation > 10cm or largest euler-angle>5 deg
// used to decide if the camera has moved far enough to generate a new nodes
//bool isBigTrafo(const Eigen::Matrix4f& t);
bool isBigTrafo(const Eigen::Isometry3d& t);
bool isBigTrafo(const g2o::SE3Quat& t);
//! Computes whether the motion per time is bigger than the parameters max_translation_meter and max_rotation_degree define
bool isSmallTrafo(const g2o::SE3Quat& t, double seconds = 1.0);
bool isSmallTrafo(const Eigen::Isometry3d& t, double seconds = 1.0);


//bool overlappingViews(LoadedEdge3D edge);
//bool triangleRayIntersection(Eigen::Vector3d triangle1,Eigen::Vector3d triangle2, Eigen::Vector3d ray_origin, Eigen::Vector3d ray);


///Convert the CV_32FC1 image to CV_8UC1 with a fixed scale factor
void depthToCV8UC1(cv::Mat& float_img, cv::Mat& mono8_img);

///Return the macro string for the cv::Mat type integer
std::string openCVCode2String(unsigned int code);

///Print Type and size of image
void printMatrixInfo(const cv::Mat& image, std::string name = std::string(""));

//!Return true if frames should be dropped because they are asynchronous
bool asyncFrameDrop(ros::Time depth, ros::Time rgb);

double errorFunction(const Eigen::Vector4f& x1, const double x1_depth_cov, 
                      const Eigen::Vector4f& x2, const double x2_depth_cov, 
                      const Eigen::Matrix4f& tf_1_to_2);

double errorFunction2(const Eigen::Vector4f& x1, 
                      const Eigen::Vector4f& x2, 
                      const Eigen::Matrix4d& tf_1_to_2);

float getMinDepthInNeighborhood(const cv::Mat& depth, cv::Point2f center, float diameter);

void observationLikelihood(const Eigen::Matrix4f& proposed_transformation,//new to old
                             pointcloud_type::Ptr new_pc,
                             pointcloud_type::Ptr old_pc,
                             const sensor_msgs::CameraInfo& old_cam_info,
                             double& likelihood, 
                             double& confidence,
                             unsigned int& inliers,
                             unsigned int& outliers,
                             unsigned int& occluded,
                             unsigned int& all) ;

/** This function computes the p-value of the null hypothesis that the transformation is the true one.
 * It is too sensitive to outliers
 */
double rejectionSignificance(const Eigen::Matrix4f& proposed_transformation,//new to old
                             pointcloud_type::Ptr new_pc,
                             pointcloud_type::Ptr old_pc);

///Quality is output param
bool observation_criterion_met(unsigned int inliers, unsigned int outliers, unsigned int all, double& quality);

/*
cv::Point nearest_neighbor(const cv::Mat source&, const cv::Mat& destination, const Eigen::Matrix4f& transformation_source_to_destination, cv::Point query_point);
Eigen::Vector3f nearest_neighbor(const cv::Mat source&, const cv::Mat& destination, const Eigen::Matrix4f& transformation_source_to_destination, Eigen::Vector3f query_point);
*/
void getColor(const point_type& p, unsigned char& r, unsigned char& g, unsigned char& b);
#endif
