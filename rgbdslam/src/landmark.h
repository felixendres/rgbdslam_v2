/*
 * landmark.h
 *
 *  Created on: Jun 1, 2012
 *      Author: Nikolas Engelhard
 */

#ifndef LANDMARK_H_
#define LANDMARK_H_

#include "node.h" // DO_FEATURE_OPTIMIZATION is defined there

//#include "g2o/types/slam3d/edge_se3_pointxyz_depth.h"
#include "g2o/types/slam3d/edge_se3_pointxyz_depth.h"
//#include "g2o/types/slam3d/vertex_pointxyz.h"

#include "g2o/types/slam3d/vertex_pointxyz.h"
//#include "g2o/types/sba/types_sba.h"

#include "g2o/types/slam3d/se3quat.h"
#include "matching_result.h"

typedef g2o::VertexPointXYZ  LM_vertex_type;
typedef g2o::EdgeSE3PointXYZDepth Proj_edge_type;

struct Landmark {

 int id;

 LM_vertex_type* g2o_vertex;
 std::map<int,int> observations; // maps node_id to keypt_id (if landmark was seen in this image)
 // std::map<int,int> observations_with_edges; // maps node_id to keypt_id (if landmark was seen in this image)
// g2o::HyperGraph::EdgeSet proj_edges;

 Landmark(){
  g2o_vertex = NULL;
 }
};


Eigen::Matrix3d point_information_matrix(double distance);
#endif /* LANDMARK_H_ */
