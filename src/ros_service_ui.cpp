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
#include <QtGui>
#include <QPixmap>
#include <QFont>
#include <QIcon>
#include <QKeySequence>
#include "ros_service_ui.h"
#include <limits>
#include "ros/ros.h"

RosUi::RosUi(const char* service_namespace) : filename("quicksave.pcd"), record_on(false)
{
    ros::NodeHandle n(service_namespace);
    server   = n.advertiseService("ros_ui",   &RosUi::services,   this);
    server_b = n.advertiseService("ros_ui_b", &RosUi::services_b, this);
    server_f = n.advertiseService("ros_ui_f", &RosUi::services_f, this);
    server_s = n.advertiseService("ros_ui_s", &RosUi::services_s, this);
    this->pause_on = ParameterServer::instance()->get<bool>("start_paused");
}


void RosUi::sendFinished() {
    ROS_INFO("Finished Sending");
}


void RosUi::pause(bool _pause_on) {
    if(this->pause_on == _pause_on){
        return;
    }
    this->pause_on = _pause_on;
    Q_EMIT togglePause();
    if(!_pause_on) {
        ROS_INFO("Processing.");
    } else {
        ROS_INFO("Stopped processing.");
    }
}


bool RosUi::services(rgbdslam::rgbdslam_ros_ui::Request  &req,
                     rgbdslam::rgbdslam_ros_ui::Response &)
{
  ROS_INFO_STREAM("Got Service Request. Command: " << req.command);
  if     (req.command == "reset"          ){ Q_EMIT reset(); }
  else if(req.command == "quick_save"     ){ Q_EMIT saveAllClouds(filename); }
  else if(req.command == "save_g2o_graph" ){ Q_EMIT saveG2OGraph("graph.g2o"); }
  else if(req.command == "save_trajectory"){ Q_EMIT saveTrajectory("trajectory"); }
  else if(req.command == "send_all"       ){ Q_EMIT sendAllClouds();}
  else if(req.command == "frame"          ){ Q_EMIT getOneFrame(); }
  else if(req.command == "delete_frame"   ){ Q_EMIT deleteLastFrame(); }
  else if(req.command == "optimize"       ){ Q_EMIT optimizeGraph(); }
  else if(req.command == "reload_config"  ){ ParameterServer::instance()->getValues();}
    else{
      ROS_ERROR("Invalid service call command: %s", req.command.c_str());
      ROS_ERROR("RGBDSLAM's services have changed in Feb '13, please revise your service calls");
      ROS_INFO("Valid commands are: {\n - reset\n - frame\n - quick_save\n -  send_all\n -  delete_frame\n -  optimize\n -  reload_config\n - save_trajectory\n}");
        return false;
    }
    return true;
}

bool RosUi::services_b(rgbdslam::rgbdslam_ros_ui_b::Request  &req,
                       rgbdslam::rgbdslam_ros_ui_b::Response &)
{
  ROS_INFO_STREAM("Got Service Request. Command: " << req.command << ". Value: " << ( req.value ? "True" : "False"));
  if     (req.command == "pause"            ){ pause(req.value); }
  else if(req.command == "mapping"          ){ Q_EMIT toggleMapping(req.value); }
  else if(req.command == "store_pointclouds"){ toggleCloudStorage(req.value); }
    else{
      ROS_ERROR("Invalid service call commands: %s", req.command.c_str());
        ROS_ERROR("Valid commands are: {pause, record, mapping, store_pointclouds}");
        return false;
    }
  return true;
}

bool RosUi::services_s(rgbdslam::rgbdslam_ros_ui_s::Request  &req,
                       rgbdslam::rgbdslam_ros_ui_s::Response &)
{
    ROS_INFO_STREAM("Got Service Request. Command: " << req.command << ". Value: " << req.value );
    QString filename = QString::fromStdString(req.value);
    if     (req.command == "save_octomap"   ){ Q_EMIT saveOctomapSig(filename); }
    else if(req.command == "save_cloud"     ){ Q_EMIT saveAllClouds(filename); }
    else if(req.command == "save_g2o_graph" ){ Q_EMIT saveG2OGraph(filename); }
    else if(req.command == "save_trajectory"){ Q_EMIT saveTrajectory(filename); }
    else if(req.command == "save_features"  ){ Q_EMIT saveAllFeatures(filename); }
    else if(req.command == "save_individual"){ Q_EMIT saveIndividualClouds(filename); }
    else{
        ROS_ERROR("Invalid service call command: %s", req.command.c_str());
        ROS_INFO("Valid commands are: {\n - save_octomap\n -  save_cloud\n -  save_g2o_graph\n -  save_trajectory\n -  save_features\n -  save_individual\n}");
        return false;
    }
    return true;
}
bool RosUi::services_f(rgbdslam::rgbdslam_ros_ui_f::Request  &req,
                       rgbdslam::rgbdslam_ros_ui_f::Response &)
{
    ROS_INFO_STREAM("Got Service Request. Command: " << req.command << ". Value: " << req.value );
    if(req.command == "set_max"){
        Q_EMIT setMaxDepth(req.value/100.0);
        return true;
    }
    else{
        ROS_ERROR("Command is set_max");
        return false;
    }
}

void RosUi::toggleCloudStorage(bool storage) {
  ParameterServer::instance()->set("store_pointclouds", storage);
  ROS_INFO_COND(storage, "Point clouds will be stored");
  ROS_INFO_COND(!storage, "Point clouds will not be stored");
}
