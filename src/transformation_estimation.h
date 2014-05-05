#ifndef RGBD_SLAM_TRAFO_EST_H_
#define RGBD_SLAM_TRAFO_EST_H_

#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>
class Node; //Fwd declaration

//!Do sparse bundle adjustment for the node pair
void getTransformFromMatchesG2O(const Node* earlier_node,
                                const Node* newer_node,
                                const std::vector<cv::DMatch> & matches,
                                Eigen::Matrix4f& transformation_estimate, //Input (initial guess) and Output
                                int iterations = 10);
#endif
