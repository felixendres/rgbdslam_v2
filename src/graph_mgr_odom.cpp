#include "graph_manager.h"
#include "scoped_timer.h"
#include "misc.h"
#include "g2o/core/robust_kernel_factory.h"
#include <tf/transform_listener.h>
#include "g2o/types/slam3d/edge_se3.h"


//specifies the information matrix for the odometry edge
//tf::Transform odomTf is in the camera coordinate system
void createOdometryEdge(int id1, int id2, tf::Transform& odomTf, LoadedEdge3D& edge)
{
  double infoCoeff = ParameterServer::instance()->get<double>("odometry_information_factor");
  edge.id1 = id1;
  edge.id2 = id2;

  tf::Vector3 origin = odomTf.getOrigin();
  if(std::fabs(origin[0])<1e-5) origin[0] =0;
  if(std::fabs(origin[1])<1e-5) origin[1] =0;
  if(std::fabs(origin[2])<1e-5) origin[2] =0;
  odomTf.setOrigin(origin);
  if(std::fabs(tf::getYaw(odomTf.getRotation())) <1e-7) odomTf.setRotation(tf::createIdentityQuaternion());

//comment in when parameters are calibrated correctly
//make sure that the final information matrix is given in the camera coordinate system
//-> for this the information matrix needs to be transformed into the camera coordinate system
//1. calibrate odometry -> extract noise parameters
//2. construct information matrix according to traveled distance and the noise parameters#
//3. transform information matrix to camera coordinate frame
/* 
  //parameters of Gaussian probabilistic motion model
  //http://www.mrpt.org/tutorials/programming/odometry-and-motion-models/probabilistic_motion_models/
  double alpha1 = 0.59;
  double alpha2 = 0.0014;
  double alpha3 = 0.02;
  double alpha4 = 0.00001;

  //hack as we know the transformation
  double var_dist_min = 0.001;
  double var_theta_min = 0.01;
  double var_dist = alpha3 * std::fabs(odomTf.getOrigin()[2]); //this is the translation
  var_dist = std::max(var_dist,var_dist_min);
  var_dist = var_dist_min + var_dist*var_dist;
  double var_theta = alpha1 * std::fabs(tf::getYaw(odomTf.getRotation())); //this is the translation
  var_theta = var_theta_min + var_theta*var_theta;
  var_theta = std::max(var_theta,var_theta_min);
*/  
 
 edge.transform = tf2G2O(odomTf);
  edge.informationMatrix = Eigen::Matrix<double, 6, 6>::Ones(); //Do not influence optimization
  edge.informationMatrix = edge.informationMatrix*0.001*infoCoeff;

//  edge.informationMatrix = Eigen::Matrix<double, 6, 6>::Zero(); //Do not influence optimization
//  edge.informationMatrix(0,0) = infoCoeff*(1./var_dist); //0.1m accuracy in the floor plane
//  edge.informationMatrix(1,1) = 10000000; //
//  edge.informationMatrix(2,2) = infoCoeff*(1./var_dist);//1./(0.001+var_dist);//0.01m information on rotation w.r. to floor
//  edge.informationMatrix(4,4) = infoCoeff*(1./var_theta);//0.02rad information on rotation w.r. to floor
//  edge.informationMatrix(3,3) = 10000000;//0.02rad information on rotation w.r. to floor
//  edge.informationMatrix(5,5) = 10000000;//1/var_angle; //rotation about vertical
}

void GraphManager::addOdometry(ros::Time timestamp,
         tf::TransformListener* listener) 
{
  //return if graph is empty
  if (graph_.size() < 2) return;

  //Variables Used in loop
  //look through graph to find nodes that do not have odom edges
  std::map<int, Node*>::reverse_iterator rit = graph_.rbegin();
  std::map<int, Node*>::reverse_iterator prev_rit = graph_.rbegin();
  prev_rit++;

  std::string odom_fixed_frame = ParameterServer::instance()->get<std::string>("odom_frame_name");
  std::string odom_target_frame = ParameterServer::instance()->get<std::string>("odom_target_frame_name");
  std::string odom_target_frame1;//for start's frame in loop
  std::string odom_target_frame2;//for end's frame in loop
  tf::StampedTransform deltaTransform;
  std::string error_msg;
  bool ok = false;
  
  for (; prev_rit != graph_.rend(); ++prev_rit, ++rit) {
    Node *earlier_node = prev_rit->second;
    Node *later_node = rit->second;
    //if we have already received the odometry for the node which we would like to point to
    //we know we can interpolate instead of extrapolate the odometry and insert an edge as well as the odometry to the node
    if (!earlier_node->has_odometry_edge_) {
        ROS_INFO("Processing odometry between %d and %d.", earlier_node->id_, later_node->id_); 
        //Get Frames
        if(odom_target_frame.empty()){//Use data frame
          odom_target_frame1 = earlier_node->pc_col->header.frame_id;
          odom_target_frame2 = later_node->pc_col->header.frame_id;
        } else{//Use value from parameter
          odom_target_frame1 = odom_target_frame;
          odom_target_frame2 = odom_target_frame;
        }

        ROS_WARN_STREAM("ODOM Target Frame " << odom_target_frame1 << " " << odom_target_frame2 << " " << odom_fixed_frame); 
 
        ok = listener->canTransform(odom_target_frame1, earlier_node->timestamp_, //frame and time of earlier node
                                    odom_target_frame2, later_node->timestamp_,   //frame and time of newer node
                                    odom_fixed_frame, &error_msg);
        if(ok){
          //from http://wiki.ros.org/tf/Tutorials/Time%20travel%20with%20tf%20%28C%2B%2B%29
          //listener.lookupTransform("/turtle2", now, "/turtle1", past, "/world", transform);
          listener->lookupTransform(odom_target_frame1, earlier_node->timestamp_, //frame and time of earlier node
                                    odom_target_frame2, later_node->timestamp_,   //frame and time of newer node
                                    odom_fixed_frame, deltaTransform);

          printTransform("Odometry Delta", deltaTransform);
          //ADD edge here
          LoadedEdge3D edge;
          createOdometryEdge(earlier_node->id_, later_node->id_, deltaTransform, edge);
          QMatrix4x4 motion_estimate =eigenTF2QMatrix(edge.transform);//not used
          addOdometryEdgeToG2O(edge, earlier_node, later_node, motion_estimate);
          earlier_node->has_odometry_edge_=true;
    //      }
        } else {//couldn't transform
          ROS_ERROR("Cannot transform between node %d (time %d.%09d) and %d (time %d.%09d). Stated reason: %s", 
                    earlier_node->id_, earlier_node->timestamp_.sec, earlier_node->timestamp_.nsec, later_node->id_, later_node->timestamp_.sec, later_node->timestamp_.nsec, error_msg.c_str());
          ROS_ERROR_STREAM(listener->allFramesAsString());
        }
    } else {// Encountered node with odometry edge = true
        //ROS_DEBUG("Node already has odometry: %d", earlier_node->id_); 
      //break; //One could save the latest node before which all nodes have odometry, but running through the nodes is quick
    }

  }
  //Hack: Update display after adding edge
  Q_EMIT updateTransforms(getAllPosesAsMatrixList());
}

bool GraphManager::addOdometryEdgeToG2O(const LoadedEdge3D& edge,
                                        Node* n1, Node* n2,  
                                        QMatrix4x4& motion_estimate) //Pure output
{
    ScopedTimer s(__FUNCTION__);
    assert(n1);
    assert(n2);
    assert(n1->id_ == edge.id1);
    assert(n2->id_ == edge.id2);

    QMutexLocker locker(&optimizer_mutex_);
    g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n1->vertex_id_));
    g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(optimizer_->vertex(n2->vertex_id_));

    if(!v1 && !v2){
      ROS_ERROR("Missing both vertices: %i, %i, cannot create edge", edge.id1, edge.id2);
      return false;
    } else if (!v1 && v2) {
      ROS_ERROR("Missing previous id. This is unexpected by the programmer");
      return false;
    } else if (!v2 && v1) {
      ROS_ERROR("Creating new id for odometry. This is unexpected by the programmer");
      return false;
    }
    else
    { 
      g2o::EdgeSE3* g2o_edge = new g2o::EdgeSE3;
      g2o_edge->vertices()[0] = v1;
      g2o_edge->vertices()[1] = v2;
      Eigen::Isometry3d meancopy(edge.transform); 
      g2o_edge->setMeasurement(meancopy);
      g2o_edge->setInformation(edge.informationMatrix);
      optimizer_->addEdge(g2o_edge);
      ROS_INFO_STREAM("Added Edge ("<< edge.id1 << "-" << edge.id2 << ") to Optimizer:\n" << edge.transform.matrix() << "\nInformation Matrix:\n" << edge.informationMatrix);
      cam_cam_edges_.insert(g2o_edge);
      odometry_edges_.insert(g2o_edge);
      current_match_edges_.insert(g2o_edge); //Used if all previous vertices are fixed ("pose_relative_to" == "all")
    } 
    if (ParameterServer::instance()->get<std::string>("pose_relative_to") == "inaffected") {
      v1->setFixed(false);
      v2->setFixed(false);
    }
    else if(ParameterServer::instance()->get<std::string>("pose_relative_to") == "largest_loop") {
      earliest_loop_closure_node_ = std::min(earliest_loop_closure_node_, edge.id1);
      earliest_loop_closure_node_ = std::min(earliest_loop_closure_node_, edge.id2);
    }
    ROS_INFO("Added odometry edge between %d and %d.", n1->id_, n2->id_);
    return true;
}

