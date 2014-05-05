#ifndef RGBD_SLAM_TF_EUC_H_
#define RGBD_SLAM_TF_EUC_H_
#include <Eigen/Core>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
class Node;
// Compute the transformation from matches using pcl::TransformationFromCorrespondences
Eigen::Matrix4f getTransformFromMatches(const Node* newer_node,
                                        const Node* older_node, 
                                        const std::vector<cv::DMatch> & matches,
                                        bool& valid, 
                                        float max_dist_m = -1);

// Compute the transformation from matches using Eigen::umeyama
Eigen::Matrix4f getTransformFromMatchesUmeyama(const Node* newer_node,
                                               const Node* older_node,
                                               std::vector<cv::DMatch> matches,
                                               bool& valid);

inline bool containsNaN(const Eigen::Matrix4f& mat){
    return (mat.array() != mat.array()).any(); //No NaNs
}
#endif
