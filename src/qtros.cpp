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


#include "qtros.h"

QtROS::QtROS(int argc, char *argv[], const char* node_name) {
  std::cout << "Initializing Node...\n";
  ros::init(argc, argv, node_name);
  n = new ros::NodeHandle(node_name); //Use node name as Ros Namespace
  ROS_INFO("Connected to roscore");
  quitfromgui = false; 
}

void QtROS::quitNow(){ 
  quitfromgui = true; 
}

void QtROS::run(){ 
  ros::Rate r(30); // 30 hz. Kinect has 30hz and we are far from processing every frame anyhow.
  while(ros::ok() && !quitfromgui) {
    ros::spinOnce(); 
    r.sleep();}
  if (!quitfromgui) {
    Q_EMIT rosQuits();
    ROS_INFO("ROS-Node Terminated\n"); 
    ros::shutdown();//Not sure if necessary
  }
  ros::Duration d(0.5);
  d.sleep();
  exit(0);
}
