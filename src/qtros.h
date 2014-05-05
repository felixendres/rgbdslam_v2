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


//!Sets up a thread for ROS event processing
/** 
 * QtThread based class encapsulating the ros basics,
 * i.e., init, node handle creation, spining and quitting.
 * To quit via qt, connect the quitNow slot to, e.g., 
 * the aboutToQuit-Signal of qapplication.
 */
#ifndef QT_ROS_H
#define QT_ROS_H
#include "ros/ros.h"
#include <QThread>
#include <QObject>

class QtROS : public QThread {
  Q_OBJECT

  public:
    ///Note: The constructor will block until connected with roscore
    ///Instead of ros::spin(), start this thread with the start() method
    ///to run the event loop of ros
    QtROS(int argc, char *argv[], const char* node_name);
    ros::NodeHandle getNodeHandle(){ return *n; }
    //! This method contains the ROS event loop. Feel free to modify 
    void run();
  public Q_SLOTS:
    //!Connect to aboutToQuit signals, to stop the thread
    void quitNow();
  Q_SIGNALS:
    //!Triggered if ros::ok() != true
    void rosQuits();
  private:
    bool quitfromgui;
    ros::NodeHandle* n;
};
#endif
