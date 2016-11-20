#include "transformation_estimation_euclidean.h"
#include "node.h"
#include <pcl/common/transformation_from_correspondences.h>
#include <Eigen/Geometry>
#include "parameter_server.h"

Eigen::Matrix4f getTransformFromMatches(const Node* newer_node,
                                        const Node* earlier_node,
                                        const std::vector<cv::DMatch>& matches,
                                        bool& valid, 
                                        const float max_dist_m) 
{
  pcl::TransformationFromCorrespondences tfc;
  valid = true;
  std::vector<Eigen::Vector3f> t, f;
  float weight = 1.0;

  BOOST_FOREACH(const cv::DMatch& m, matches)
  {
    Eigen::Vector3f from = newer_node->feature_locations_3d_[m.queryIdx].head<3>();
    Eigen::Vector3f to = earlier_node->feature_locations_3d_[m.trainIdx].head<3>();
    if(std::isnan(from(2)) || std::isnan(to(2)))
      continue;

    weight = 1.0/(from(2) * to(2));
#ifdef HEMACLOUDS
    ParameterServer* ps = ParameterServer::instance();

    //Create point cloud inf necessary
    if(ps->get<int>("segment_to_optimize") > 0){
      weight =1/( earlier_node->feature_locations_3d_[m.trainIdx][3] \
                + newer_node->feature_locations_3d_[m.queryIdx][3]);
    } else {
      weight =1/( earlier_node->feature_locations_3d_[m.trainIdx][2] \
                + newer_node->feature_locations_3d_[m.queryIdx][2]);
    }
#endif
    if(false){//is that code useful?
      //Validate that 3D distances are corresponding
      if (max_dist_m > 0) {  //storing is only necessary, if max_dist is given
        if(f.size() >= 1)
        {
          float delta_f = (from - f.back()).squaredNorm();//distance to the previous query point
          float delta_t = (to   - t.back()).squaredNorm();//distance from one to the next train point

          if ( abs(delta_f-delta_t) > max_dist_m * max_dist_m ) {
            valid = false;
            return Eigen::Matrix4f();
          }
        }
        f.push_back(from);
        t.push_back(to);    
      }
    }

    tfc.add(from, to, weight);// 1.0/(to(2)*to(2)));//the further, the less weight b/c of quadratic accuracy decay
  }

  // get relative movement from samples
  return tfc.getTransformation().matrix();
}

Eigen::Matrix4f getTransformFromMatchesUmeyama(const Node* newer_node,
                                               const Node* earlier_node,
                                               std::vector<cv::DMatch> matches,
                                               bool& valid) 
{
  Eigen::Matrix<float, 3, Eigen::Dynamic> tos(3,matches.size()), froms(3,matches.size());
  std::vector<cv::DMatch>::const_iterator it = matches.begin();
  for (int i = 0 ;it!=matches.end(); it++, i++) {
    Eigen::Vector3f f = newer_node->feature_locations_3d_[it->queryIdx].head<3>(); //Oh my god, c++
    Eigen::Vector3f t = earlier_node->feature_locations_3d_[it->trainIdx].head<3>();
    if(std::isnan(f(2)) || std::isnan(t(2)))
      continue;
    froms.col(i) = f;
    tos.col(i) = t;
  }
  Eigen::Matrix4f res = Eigen::umeyama(froms, tos, false);
  valid = !containsNaN(res);
  return res;
}
