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

#include "bagloader.h"
#include "qtros.h"
#include <QApplication>
#include <QObject>
#include "qt_gui.h"
#include <Eigen/Core>
#include "parameter_server.h"
#include "ros_service_ui.h"

//TODO:
//better potential-edge-selection through flann or place recognition
//Better separation of function, communication, parameters and gui
//Better integration of the calibration data
//Correct implementation of odometry
//Multi-Camera-fusion

///Connect Signals and Slots only relevant for the graphical interface
void gui_connections(Graphical_UI* gui, BagLoader* loader)
{
    QObject::connect(gui, SIGNAL(openBagFile(QString)), loader, SLOT(loadBagFileAsync(QString)));
    if (ParameterServer::instance()->get<bool>("use_glwidget") && gui->getGLViewer() != NULL) {
      GLViewer* glv = gui->getGLViewer();
	    QObject::connect(loader, SIGNAL(setPointCloud(pointcloud_type *, QMatrix4x4)), glv, SLOT(addPointCloud(pointcloud_type *, QMatrix4x4)), Qt::BlockingQueuedConnection ); //Needs to block, otherwise the opengl list compilation makes the app unresponsive. This effectively throttles the processing rate though
      typedef const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >* cnst_ft_vectors;
      QObject::connect(glv, SIGNAL(cloudRendered(pointcloud_type *)), loader, SLOT(clearPointCloud(pointcloud_type *))); // 
    }
    QObject::connect(loader, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
    QObject::connect(loader, SIGNAL(setGUIInfo2(QString)), gui, SLOT(setInfo2(QString)));
    QObject::connect(loader, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
    QObject::connect(gui, SIGNAL(togglePause()), loader, SLOT(togglePause()));
}

/** On program startup:
 * Create 
 * - a Qt Application 
 * - an Object representing the ROS Node and its callback loop, 
 * - an bagloader, handling input
 * - also a GUI
 * - let the above communicate internally via QT Signals, where communcication needs to be across threads or if the communication is conditional on the ROS node's parameterization.
 */
int main(int argc, char** argv)
{
  setlocale(LC_NUMERIC,"C");//Avoid expecting german decimal separators in launch files

  //create thread object, to run the ros event processing loop in parallel to the qt loop
  QtROS qtRos(argc, argv, "rgbdslam_bag_viewer"); //ros node name & namespace

  //Depending an use_gui on the Parameter Server, a gui- or a headless application is used
  QApplication app(argc, argv); 

  //Instantiate the kinect image loader
  BagLoader loader;

  Graphical_UI* gui = new Graphical_UI("RGBDSLAM's Bagviewer");
  gui->show();
  gui->set2DStream(false);
  gui_connections(gui, &loader);


  //If one thread receives a exit signal from the user, signal the other thread to quit too
  QObject::connect(&app, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
  QObject::connect(&qtRos, SIGNAL(rosQuits()), &app, SLOT(quit()));

  qtRos.start();// Run main loop.
  
  std::string bagfile_name = ParameterServer::instance()->get<std::string>("bagfile_name");
  if(!bagfile_name.empty()){
    loader.loadBagFileAsync(bagfile_name);
  }
  app.exec();
}


