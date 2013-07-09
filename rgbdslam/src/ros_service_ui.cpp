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
    createActions(service_namespace);
    this->pause_on = ParameterServer::instance()->get<bool>("start_paused");
}

void RosUi::reloadConfig() {
    ParameterServer::instance()->getValues();
    ROS_INFO("Reloaded Parameters from Parameter Server");
}

void RosUi::resetCmd() {
    ROS_INFO("Graph Reset");
    Q_EMIT reset();
    ROS_INFO("A fresh new graph is waiting");
}

void RosUi::quickSaveAll() {
    Q_EMIT saveAllClouds(filename);
    ROS_INFO("Saving Whole Model to %s", qPrintable(filename));
}

void RosUi::saveFeatures() {
    Q_EMIT saveAllFeatures(QString("feature_database.yml"));
    ROS_INFO("Saving Whole Model to %s", qPrintable(filename));
}

void RosUi::saveOctomap() {
    Q_EMIT saveOctomapSig("/tmp/rgbdslam_octomap.ot");
    ROS_INFO("Saving Whole Model to /tmp/rgbdslam_octomap.ot");
}

void RosUi::saveAll() {
    Q_EMIT saveAllClouds(filename);
    ROS_INFO("Saving Whole Model to %s", qPrintable(filename));
}

void RosUi::saveIndividual() {
    QString tmpfilename(filename);
    tmpfilename.remove(".pcd", Qt::CaseInsensitive);
    tmpfilename.remove(".ply", Qt::CaseInsensitive);
    Q_EMIT saveIndividualClouds(filename);
    ROS_INFO("Saving Model Node-Wise");
}

void RosUi::sendAll() {
    Q_EMIT sendAllClouds();
    ROS_INFO("Sending Whole Model");
}

void RosUi::sendFinished() {
    ROS_INFO("Finished Sending");
}

void RosUi::getOneFrameCmd() {
    Q_EMIT getOneFrame();
    ROS_INFO("Getting a single frame");
}

void RosUi::deleteLastFrameCmd() {
    Q_EMIT deleteLastFrame();
    ROS_INFO("Deleting the last node from the graph");
}

void RosUi::bagRecording(bool _record_on) {
    if(this->record_on == _record_on){
        return;
    }
    this->record_on = _record_on;
    Q_EMIT toggleBagRecording();
    if(_record_on) {
        ROS_INFO("Recording Bagfile.");
    } else {
        ROS_INFO("Stopped Recording.");
    }
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

void RosUi::setMax(float val){
    Q_EMIT setMaxDepth(val/100.0);
}

void RosUi::createActions(const char* service_namespace) {
    //srv_reset der bei triggern resetCmd() aufruft
    ros::NodeHandle n(service_namespace);
    server   = n.advertiseService("ros_ui", &RosUi::services, this);
    server_b = n.advertiseService("ros_ui_b", &RosUi::services_b, this);
    server_f = n.advertiseService("ros_ui_f", &RosUi::services_f, this);
}

bool RosUi::services(rgbdslam::rgbdslam_ros_ui::Request  &req,
                     rgbdslam::rgbdslam_ros_ui::Response &res )
{
    if     (req.comand == "reset"          ){ resetCmd(); }
    else if(req.comand == "quick_save"     ){ quickSaveAll(); }
    else if(req.comand == "save_cloud"     ){ saveAll(); }
    else if(req.comand == "save_features"  ){ saveFeatures(); }
    else if(req.comand == "save_g2o_graph" ){ Q_EMIT saveG2OGraph("graph.g2o"); }
    else if(req.comand == "save_trajectory"){ Q_EMIT saveTrajectory("trajectory"); }
    else if(req.comand == "save_individual"){ saveIndividual(); }
    else if(req.comand == "send_all"       ){ sendAll(); }
    else if(req.comand == "frame"          ){ getOneFrame(); }
    else if(req.comand == "delete_frame"   ){ deleteLastFrame(); }
    else if(req.comand == "optimize"       ){ Q_EMIT optimizeGraph(); }
    else if(req.comand == "reload_config"  ){ reloadConfig(); }
    else{
        ROS_ERROR("Invalid service call commands: %s", req.comand.c_str());
        ROS_INFO("Valid commands are: {\n - reset\n -  quick_save\n -  save_all\n -  save_individual\n -  send_all\n -  delete_frame\n -  optimize\n -  reload_config\n - save_g2o_graph\n - save_trajectory\n}");
        return false;
    }
    return true;
}

bool RosUi::services_b(rgbdslam::rgbdslam_ros_ui_b::Request  &req,
                       rgbdslam::rgbdslam_ros_ui_b::Response &res )
{
    if     (req.comand == "pause" ){ pause(req.value); }
    else if(req.comand == "record"){ bagRecording(req.value); }
    else if(req.comand == "mapping"){ Q_EMIT toggleMapping(req.value); }
    else if(req.comand == "store_pointclouds"){ toggleCloudStorage(req.value); }
    else{
        ROS_ERROR("Invalid service call commands: %s", req.comand.c_str());
        ROS_ERROR("Valid commands are: {pause, record, mapping, store_pointclouds}");
        return false;
    }
  return true;
}

bool RosUi::services_f(rgbdslam::rgbdslam_ros_ui_f::Request  &req,
                       rgbdslam::rgbdslam_ros_ui_f::Response &res )
{
    if(req.comand == "set_max"){
        setMax(req.value);
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
