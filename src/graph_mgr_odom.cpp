#include "graph_manager.h"
#include "scoped_timer.h"
#include "misc.h"
#include "g2o/core/robust_kernel_factory.h"
#include <tf/transform_listener.h>

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

  tf::Transform base2cam;
  base2cam.setOrigin(tf::Vector3(0,0,0));
  base2cam.setRotation(tf::createQuaternionFromRPY(-0.14207,0,-0.011164));
//  base2cam.setOrigin(tf::Vector3(0.1,-0.04,0.3));
//  base2cam.setRotation(tf::createQuaternionFromRPY(-1.71207,0,-1.57));

  //tf::Transform odomTf = base2cam * odomTf * base2cam.inverse();
  odomTf = base2cam.inverse() * odomTf * base2cam;

  //parameters of Gaussian probabilistic motion model
  //http://www.mrpt.org/tutorials/programming/odometry-and-motion-models/probabilistic_motion_models/
  double calibration_factor = 1;
  double alpha1 = 0.59*calibration_factor;
  //double alpha1 = 100*calibration_factor;
  double alpha2 = 0.0014*calibration_factor;
  double alpha3 = 0.02*calibration_factor;//0.0169;
  double alpha4 = 0.00001*calibration_factor;

  //hack as we know the transformation
  double var_dist_min = 0.001;
  double var_theta_min = 0.01;
  double var_dist = alpha3 * std::fabs(odomTf.getOrigin()[2]); //this is the translation
//  var_dist = std::max(var_dist,var_dist_min);
  var_dist = var_dist_min + var_dist*var_dist;
  double var_theta = alpha1 * std::fabs(tf::getYaw(odomTf.getRotation())); //this is the translation
  var_theta = var_theta_min + var_theta*var_theta;
//  var_theta = std::max(var_theta,var_theta_min);
  
  edge.transform = tf2G2O(odomTf);
  edge.informationMatrix = Eigen::Matrix<double, 6, 6>::Zero(); //Do not influence optimization
  edge.informationMatrix(0,0) = infoCoeff*(1./var_dist); //0.1m accuracy in the floor plane
  edge.informationMatrix(1,1) = 10000000; //
  edge.informationMatrix(2,2) = infoCoeff*(1./var_dist);//1./(0.001+var_dist);//0.01m information on rotation w.r. to floor
  edge.informationMatrix(4,4) = infoCoeff*(1./var_theta);//0.02rad information on rotation w.r. to floor
  edge.informationMatrix(3,3) = 10000000;//0.02rad information on rotation w.r. to floor
  edge.informationMatrix(5,5) = 10000000;//1/var_angle; //rotation about vertical
  //edge.informationMatrix(4,4) = 0;//0.02rad information on rotation w.r. to floor

/*  if((id1 < 671) || (id1 > 852) || (id2 <671) || (id2 > 852)){
  edge.informationMatrix = Eigen::Matrix<double, 6, 6>::Zero(); //Do not influence optimization
  edge.informationMatrix(0,0) = 10; //0.1m accuracy in the floor plane
  edge.informationMatrix(1,1) = 10000000; //
  edge.informationMatrix(2,2) = 10;//1./(0.001+var_dist);//0.01m information on rotation w.r. to floor
  edge.informationMatrix(4,4) = 10;//0.02rad information on rotation w.r. to floor
  edge.informationMatrix(3,3) = 10000000;//0.02rad information on rotation w.r. to floor
  edge.informationMatrix(5,5) = 10000000;//1/var_angle; //rotation about vertical
 // }
*/

//  std::cout << "Juergen dist " << std::endl;
//  double dist=odom_base.getOrigin().length();
//  std::cout << odom_base.getOrigin()[0] << " " << odom_base.getOrigin()[1] << " " <<odom_base.getOrigin()[2] << std::endl;
//  double angle=tf::getYaw(odom_base.getRotation());
//  double var_dist = alpha3*dist + alpha4*angle;
//  var_dist=var_dist*var_dist;
//  double var_angle = alpha1*angle + alpha2*dist;
//  var_angle=var_angle*var_angle;

//     // See http://www.mrpt.org/Probabilistic_Motion_Models
//     // -----------------------------------
//
//  Eigen::Matrix3d c_odo;
//  c_odo.fill(0.);
//  c_odo(0,0)=var_dist;
//  c_odo(1,1)=var_dist;
//  c_odo(2,2)=var_angle;
//
//  double cos_phi_2 = cos(angle/2);
//  double sin_phi_2 = sin(angle/2);
//  double x = odom_base.getOrigin()[0];
//  double y = odom_base.getOrigin()[1];
//  Eigen::Matrix3d jac;
//  jac(0,0)=cos_phi_2;
//  jac(0,1)=-sin_phi_2;
//  jac(0,2)=-sin_phi_2 * x - cos_phi_2 * y;
//  jac(1,0)=sin_phi_2;
//  jac(1,1)=cos_phi_2;
//  jac(1,2)=cos_phi_2 * x - sin_phi_2 * y;
//  jac(2,2)=1;
//
//  Eigen::Matrix3d cov = jac * c_odo * jac.transpose();
//  Eigen::Matrix3d inv_cov =cov.inverse();
//  double cov_angle=inv_cov(2,2);
//  Eigen::Matrix3d cov3d;
//  cov3d.fill(0.);
//  cov3d.block(0,0,2,2)=cov.block(0,0,2,2);
//  cov3d(2,2)=10000000;
//  Eigen::Matrix3d rot = Eigen::Quaterniond(-0.5,0.5,-0.5,0.5).toRotationMatrix();
//  cov3d = rot * cov3d * rot.transpose();
//  Eigen::Matrix3d cov3d_inv=cov3d.inverse();

//  edge.transform = tf2G2O(odomTf);
//  edge.informationMatrix = Eigen::Matrix<double, 6, 6>::Zero(); //Do not influence optimization
//  edge.informationMatrix(0,0) = infoCoeff*(1./var_dist); //0.1m accuracy in the floor plane
//  edge.informationMatrix(1,1) = 0; //
//  edge.informationMatrix(2,2) = infoCoeff*(1./var_dist);//1./(0.001+var_dist);//0.01m information on rotation w.r. to floor
//  edge.informationMatrix(3,3) = infoCoeff*(1./var_theta);//0.02rad information on rotation w.r. to floor
//  edge.informationMatrix(4,4) = 0;//0.02rad information on rotation w.r. to floor
//  edge.informationMatrix(5,5) = 0;//1/var_angle; //rotation about vertical


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
    Node *start = prev_rit->second;
    Node *end = rit->second;
    //if we have already received the odometry for the node which we would like to point to
    //we know we can interpolate instead of extrapolate the odometry and insert an edge as well as the odometry to the node
    if (!start->has_odometry_edge_) {
        //Get Frames
        if(odom_target_frame.empty()){//Use data frame
          odom_target_frame1 = start->pc_col->header.frame_id;
          odom_target_frame2 = end->pc_col->header.frame_id;
        } else{//Use value from parameter
          odom_target_frame1 = odom_target_frame;
          odom_target_frame2 = odom_target_frame;
        }

        ROS_WARN_STREAM("ODOM Target Frame " << odom_target_frame1 << " " << odom_target_frame2 << " " << odom_fixed_frame); 
 
        ok = listener->canTransform(odom_target_frame1, start->timestamp_, //frame and time of earlier node
                                    odom_target_frame2, end->timestamp_,   //frame and time of newer node
                                    odom_fixed_frame, &error_msg);
        if(ok){
          //from http://wiki.ros.org/tf/Tutorials/Time%20travel%20with%20tf%20%28C%2B%2B%29
          //listener.lookupTransform("/turtle2", now, "/turtle1", past, "/world", transform);
          listener->lookupTransform(odom_target_frame1, start->timestamp_, //frame and time of earlier node
                                    odom_target_frame2, end->timestamp_,   //frame and time of newer node
                                    odom_fixed_frame, deltaTransform);

          printTransform("Odometry Delta", deltaTransform);
          //ADD edge here
          //JUERGEN: TODO: REMOVE
     //     if(((start->id_ < 671) || (start->id_ > 852)) & ((end->id_ <671) || (end->id_ > 852))){

          LoadedEdge3D edge;
          createOdometryEdge(start->id_, end->id_, deltaTransform, edge);
          QMatrix4x4 motion_estimate =eigenTF2QMatrix(edge.transform);//not used
          addOdometryEdgeToG2O(edge, start, end, motion_estimate);
          start->has_odometry_edge_=true;
    //      }
        } else {//couldn't transform
          ROS_ERROR("Cannot transform between node %d (time %d.%09d) and %d (time %d.%09d). Stated reason: %s", 
                    start->id_, start->timestamp_.sec, start->timestamp_.nsec, end->id_, end->timestamp_.sec, end->timestamp_.nsec, error_msg.c_str());
          ROS_ERROR_STREAM(listener->allFramesAsString());
        }
    } else {// Encountered node with odometry edge = true
      //break;
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

    if (ParameterServer::instance()->get<std::string>("pose_relative_to") == "inaffected") {
      v1->setFixed(false);
      v2->setFixed(false);
    }
    else if(ParameterServer::instance()->get<std::string>("pose_relative_to") == "largest_loop") {
      earliest_loop_closure_node_ = std::min(earliest_loop_closure_node_, edge.id1);
      earliest_loop_closure_node_ = std::min(earliest_loop_closure_node_, edge.id2);
    }
    return true;
}
/*
// returns true, iff node could be added to the cloud
void GraphManager::addOdometry(ros::Time timestamp,
                tf::TransformListener* listener) {
        //return if graph is empty
        if (graph_.size() < 2) {
                return;
        }

        //look through graph to find nodes that do not have odom edges
        bool insert_odometry = false;
        std::map<int, Node*>::reverse_iterator rit = graph_.rbegin();
        std::map<int, Node*>::reverse_iterator prev_rit = graph_.rbegin();
        prev_rit++;

        for (; prev_rit != graph_.rend(); ++prev_rit, ++rit) {
                Node *start = prev_rit->second;
                Node *end = rit->second;
                ROS_WARN("Timestamp of Node %d: %f", start->id_, start->timestamp_.toSec());
                ROS_WARN("Timestamp of Node %d: %f", end->id_, end->timestamp_.toSec());

                //if we have already received the odometry for the node which we would like to point to
                //we know we can interpolate instead of extrapolate the odometry and insert an edge as well as the odometry to the node
                if ((end->timestamp_ < timestamp) && (!start->has_odometry_edge_)) {

                        //get start transform from node or if not yet available from tf
                        ParameterServer* ps = ParameterServer::instance();
                        std::string odom_frame = ps->get<std::string>("odom_frame_name");
                        //std::string base_frame = ps->get<std::string>("base_frame_name");
                        tf::StampedTransform start_tf;
                        if (start->odometry_set_) {
                                start_tf = start->getOdomTransform();
                        } else {
                                ROS_WARN("1: Computing odom2points for node %d", start->id_);
                                tf::StampedTransform odom2points;
                                try {
                                        listener->lookupTransform(odom_frame, start->pc_col->header.frame_id, 
                                                        start->timestamp_, odom2points);
                                } catch (tf::TransformException ex) {
                                        ROS_ERROR("%s", ex.what());
                                        tf::StampedTransform odom2base;
                                        
                                        try {
                                            listener->lookupTransform(odom_frame, ps->get<std::string>("base_frame_name"),
                                                            start->timestamp_, odom2base);
                                            {
                                              tf::Transform base2points;
                                              ROS_WARN("Using Standard kinect /openni_camera -> /openni_rgb_optical_frame as transformation ");
                                              //emulate the transformation from kinect openni_camera frame to openni_rgb_optical_frame
                                              //base2points.setRotation(tf::createQuaternionFromRPY(-1.57,M_PI,-1.57));
                                              //base2points.setOrigin(tf::Point(0,-0.04,0));
                                              base2points.setRotation(tf::createQuaternionFromRPY(0,0,0));
                                              base2points.setOrigin(tf::Point(0,0,0));
                                              odom2points  = tf::StampedTransform(odom2base * base2points, odom2points.stamp_, odom2points.frame_id_, odom2points.child_frame_id_);
                                            }
                                        } catch (tf::TransformException ex) {
                                          ROS_ERROR("Is there no odometry at all? %s", ex.what());
                                          continue;
                                        }
                                }
                                start->setOdomTransform(odom2points);
                        }

                        //get end transform from node or if not yet available from tf
                        tf::StampedTransform odom2points;
                        ROS_WARN("2: Computing odom2points for node %d", end->id_);
                        try {
                                listener->lookupTransform(odom_frame, end->pc_col->header.frame_id,
                                                end->timestamp_, odom2points);
                        } catch (tf::TransformException ex) {
                              ROS_ERROR("%s", ex.what());
                              tf::StampedTransform odom2base;
                              try {
                                      listener->lookupTransform(odom_frame, ps->get<std::string>("base_frame_name"),
                                                      start->timestamp_, odom2base);
                                      {
                                        tf::Transform base2points;
                                        ROS_WARN("Using Standard kinect /openni_camera -> /openni_rgb_optical_frame as transformation ");
                                        //emulate the transformation from kinect openni_camera frame to openni_rgb_optical_frame
                                        //base2points.setRotation(tf::createQuaternionFromRPY(-1.57,M_PI,-1.57));
                                        //base2points.setOrigin(tf::Point(0,-0.04,0));
                                        base2points.setRotation(tf::createQuaternionFromRPY(0,0,0));
                                        base2points.setOrigin(tf::Point(0,0,0));
                                        odom2points  = tf::StampedTransform(odom2base * base2points, odom2points.stamp_, odom2points.frame_id_, odom2points.child_frame_id_);
                                        printTransform("Odom to Points", odom2points);
                                          printTransform("Odom to Base", odom2base);
                                        }
                              } catch (tf::TransformException ex) {
                                ROS_ERROR("Is there no odometry at all? %s", ex.what());
                                continue;
                              }
                        } 
                        end->setOdomTransform(odom2points);

                         //Get motion estimate from transformations
                        tf::Transform delta_transform = start->getOdomTransform().inverse() * end->getOdomTransform();
                        printTransform("Odometry Delta", delta_transform);
                        //ADD edge here
                        LoadedEdge3D edge;
                        edge.id1 = start->id_;
                        edge.id2 = end->id_;
                        edge.transform = tf2G2O(delta_transform);
                        QMatrix4x4 motion_estimate = g2o2QMatrix(edge.transform);

                        //Real odometry
                        edge.informationMatrix =
                                        Eigen::Matrix<double, 6, 6>::Identity() * 100; //0.1m /0.1rad error
                         ////Assumption: Motion in the plane
                         //odom_edge.informationMatrix(0,0) = 100; //0.1m accuracy in the floor plane
                         //odom_edge.informationMatrix(1,1) = 100; //
                         //odom_edge.informationMatrix(2,2) = 1000000;//0.01m information on rotation w.r. to floor
                         //odom_edge.informationMatrix(3,3) = 400000000;//0.02rad information on rotation w.r. to floor
                         //odom_edge.informationMatrix(4,4) = 400000000;//0.02rad information on rotation w.r. to floor
                         //odom_edge.informationMatrix(5,5) = 1600; //0.4rad (~20°) on rotation about vertical

                        addOdometryEdgeToG2O(edge, start, end, motion_estimate);
                        start->has_odometry_edge_=true;
                } else if ((start->has_odometry_edge_)
                                || (end->timestamp_ > timestamp)) { //next node has odometry edge --> we are done
                        break;
                }
        }
}
*/

