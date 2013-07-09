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
#ifndef ROS_UI_H
#define ROS_UI_H

#include <QMainWindow>
#include <QGridLayout>
#include "glviewer.h"
#include "parameter_server.h"
#include <QObject>
#include "rgbdslam/rgbdslam_ros_ui.h"
#include "rgbdslam/rgbdslam_ros_ui_b.h"
#include "rgbdslam/rgbdslam_ros_ui_f.h"

class QAction;

//!Headless version of the Graphical_UI
/**works with service-calls and offers the possibility to run rgbdslam without a GUI */
class RosUi: public QObject{
    Q_OBJECT

public:
    RosUi(const char* service_namespace);

    ///a service-client for all methods not needing an argument: {reset, quick_save, save_all, save_individual, send_all, delete_frame}
    bool services(rgbdslam::rgbdslam_ros_ui::Request  &req, rgbdslam::rgbdslam_ros_ui::Response &res);
    ///a service-client for all methods with bool as argument: {pause, record}
    bool services_b(rgbdslam::rgbdslam_ros_ui_b::Request  &req, rgbdslam::rgbdslam_ros_ui_b::Response &res);
    ///a service-client for changing the maximal depth of a point: {set_max}
    bool services_f(rgbdslam::rgbdslam_ros_ui_f::Request  &req, rgbdslam::rgbdslam_ros_ui_f::Response &res);
Q_SIGNALS:
    ///User selected to reset the graph
    void reset(); 
    ///User selected to start or resume processing
    void togglePause();
    ///User selected to do SLAM or Localization only
    void toggleMapping(bool);
    ///User selected to start or resume bag recording
    void toggleBagRecording();
    ///User wants the next frame to be processed
    void getOneFrame();
    ///User wants the last node to be removed from the graph
    void deleteLastFrame();
    void sendAllClouds(); ///< Signifies the sending of the whole model
    ///User wants the current world model to be saved to a pcd-file or ply file
    void saveAllClouds(QString filename);
    void saveOctomapSig(QString filename);
    ///User wants the feature locations and descriptions saved to yaml or xml file
    void saveAllFeatures(QString filename);
    ///User wants the g2o graph saved 
    void saveG2OGraph(QString filename);
    ///User wants the current world model to be saved to one pcd-file per node
    void saveIndividualClouds(QString file_basename);
    void setMaxDepth(float max_depth);
    ///User wants logfiles of the trajectory estimate (and ground truth if available)
    void saveTrajectory(QString);
    void optimizeGraph();
     
public Q_SLOTS:
    void sendFinished();

private:
    void resetCmd();
    void reloadConfig();
    void sendAll();
    void setMax(float val);
    void saveAll();
    void saveOctomap();
    void saveFeatures();
    void saveIndividual();
    void quickSaveAll();
    void pause(bool);
    void bagRecording(bool);
    void toggleCloudStorage(bool);
    void getOneFrameCmd();
    void deleteLastFrameCmd();
private:
    void createActions(const char* service_namespace);
    bool pause_on;
    QString filename;
    bool record_on;
    ros::ServiceServer server;
    ros::ServiceServer server_b;
    ros::ServiceServer server_f;
};

#endif
