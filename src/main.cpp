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

#include "openni_listener.h"
#include "qtros.h"
#include <QApplication>
#include <QObject>
#include "qt_gui.h"
#include "glviewer.h"
#include <Eigen/Core>
#include "parameter_server.h"
#include "ros_service_ui.h"

//TODO:
//better potential-edge-selection through flann or place recognition
//Better separation of function, communication, parameters and gui
//Better integration of the calibration data
//Correct implementation of odometry
//Multi-Camera-fusion

class Renderable; //Fwd decl for signal renderableOctomap
///Connect Signals and Slots for the ui control
void ui_connections(QObject* ui, GraphManager* graph_mgr, OpenNIListener* listener)
{
  Qt::ConnectionType ctype = Qt::AutoConnection;
  if (ParameterServer::instance()->get<bool>("concurrent_io")) 
    ctype = Qt::DirectConnection;
  QObject::connect(ui, SIGNAL(reset()), graph_mgr, SLOT(reset()), ctype);
  QObject::connect(ui, SIGNAL(optimizeGraph()), graph_mgr, SLOT(optimizeGraph()), ctype);
  QObject::connect(ui, SIGNAL(togglePause()), listener, SLOT(togglePause()), ctype);
  QObject::connect(ui, SIGNAL(getOneFrame()), listener, SLOT(getOneFrame()), ctype);
  QObject::connect(ui, SIGNAL(deleteLastFrame()), graph_mgr, SLOT(deleteLastFrame()), ctype);
  QObject::connect(ui, SIGNAL(sendAllClouds()), graph_mgr, SLOT(sendAllClouds()), ctype);
  QObject::connect(ui, SIGNAL(saveAllClouds(QString)), graph_mgr, SLOT(saveAllClouds(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveOctomapSig(QString)), graph_mgr, SLOT(saveOctomap(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveAllFeatures(QString)), graph_mgr, SLOT(saveAllFeatures(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveIndividualClouds(QString)), graph_mgr, SLOT(saveIndividualClouds(QString)), ctype);
  QObject::connect(ui, SIGNAL(saveTrajectory(QString)), graph_mgr, SLOT(saveTrajectory(QString)), ctype);
  QObject::connect(ui, SIGNAL(toggleMapping(bool)), graph_mgr, SLOT(toggleMapping(bool)), ctype);
  QObject::connect(ui, SIGNAL(saveG2OGraph(QString)), graph_mgr, SLOT(saveG2OGraph(QString)), ctype);
}

///Connect Signals and Slots only relevant for the graphical interface
void gui_connections(Graphical_UI* gui, GraphManager* graph_mgr, OpenNIListener* listener)
{
    QObject::connect(listener,  SIGNAL(newVisualImage(QImage)), gui, SLOT(setVisualImage(QImage)));
    QObject::connect(listener,  SIGNAL(newFeatureFlowImage(QImage)), gui, SLOT(setFeatureFlowImage(QImage)));
    QObject::connect(listener,  SIGNAL(newFeatureImage(QImage)), gui, SLOT(setFeatureImage(QImage)));
    QObject::connect(listener,  SIGNAL(newDepthImage(QImage)), gui, SLOT(setDepthImage(QImage)));
    QObject::connect(graph_mgr, SIGNAL(sendFinished()), gui, SLOT(sendFinished()));
    QObject::connect(graph_mgr, SIGNAL(iamBusy(int, const char*, int)), gui, SLOT(showBusy(int, const char*, int)));
    QObject::connect(graph_mgr, SIGNAL(progress(int, const char*, int)), gui, SLOT(setBusy(int, const char*, int)));
    QObject::connect(listener, SIGNAL(iamBusy(int, const char*, int)), gui, SLOT(showBusy(int, const char*, int)));
    QObject::connect(listener, SIGNAL(progress(int, const char*, int)), gui, SLOT(setBusy(int, const char*, int)));
    QObject::connect(graph_mgr, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
    QObject::connect(graph_mgr, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
    QObject::connect(gui, SIGNAL(printEdgeErrors(QString)), graph_mgr, SLOT(printEdgeErrors(QString)));
    QObject::connect(gui, SIGNAL(pruneEdgesWithErrorAbove(float)), graph_mgr, SLOT(pruneEdgesWithErrorAbove(float)));
    QObject::connect(gui, SIGNAL(clearClouds()), graph_mgr, SLOT(clearPointClouds()));
    QObject::connect(gui, SIGNAL(occupancyFilterClouds()), graph_mgr, SLOT(occupancyFilterClouds()));
    QObject::connect(gui, SIGNAL(saveBagfile(QString)), graph_mgr, SLOT(saveBagfileAsync(QString)));
    QObject::connect(gui, SIGNAL(openPCDFiles(QStringList)), listener, SLOT(loadPCDFiles(QStringList)));
    QObject::connect(gui, SIGNAL(openBagFile(QString)), listener, SLOT(loadBagFileFromGUI(QString)));
    if (ParameterServer::instance()->get<bool>("use_glwidget") && gui->getGLViewer() != NULL) {
      GLViewer* glv = gui->getGLViewer();
      QObject::connect(graph_mgr, SIGNAL(setPointCloud(pointcloud_type *, QMatrix4x4)), glv, SLOT(addPointCloud(pointcloud_type *, QMatrix4x4)), Qt::BlockingQueuedConnection ); //Needs to block, otherwise the opengl list compilation makes the app unresponsive. This effectively throttles the processing rate though
      typedef const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >* cnst_ft_vectors;
      QObject::connect(graph_mgr, SIGNAL(setFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >*)), glv, SLOT(addFeatures(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >*))); //, Qt::DirectConnection);
      QObject::connect(graph_mgr, SIGNAL(setGraphEdges(const QList<QPair<int, int> >*)), glv, SLOT(setEdges(const QList<QPair<int, int> >*)));
      QObject::connect(graph_mgr, SIGNAL(updateTransforms(QList<QMatrix4x4>*)), glv, SLOT(updateTransforms(QList<QMatrix4x4>*)));
      QObject::connect(graph_mgr, SIGNAL(deleteLastNode()), glv, SLOT(deleteLastNode()));
      QObject::connect(graph_mgr, SIGNAL(resetGLViewer()),  glv, SLOT(reset()));
      QObject::connect(graph_mgr, SIGNAL(renderableOctomap(Renderable*)),  glv, SLOT(setRenderable(Renderable*)));
      //QObject::connect(glv, SIGNAL(clickedPosition(float,float,float)), graph_mgr, SLOT(filterNodesByPosition(float,float,float)));
      if(!ParameterServer::instance()->get<bool>("store_pointclouds")) {
          QObject::connect(glv, SIGNAL(cloudRendered(pointcloud_type *)), graph_mgr, SLOT(clearPointCloud(pointcloud_type const *))); // 
      } else if(ParameterServer::instance()->get<double>("voxelfilter_size") > 0.0) {
          QObject::connect(glv, SIGNAL(cloudRendered(pointcloud_type *)), graph_mgr, SLOT(reducePointCloud(pointcloud_type const *))); // 
      }
    }
    QObject::connect(listener, SIGNAL(setGUIInfo(QString)), gui, SLOT(setInfo(QString)));
    QObject::connect(listener, SIGNAL(setGUIStatus(QString)), gui, SLOT(setStatus(QString)));
    QObject::connect(listener, SIGNAL(setGUIInfo2(QString)), gui, SLOT(setInfo2(QString)));
    QObject::connect(graph_mgr, SIGNAL(setGUIInfo2(QString)), gui, SLOT(setInfo2(QString)));
}

/** On program startup:
 * Create 
 * - a Qt Application 
 * - an Object representing the ROS Node and its callback loop, 
 * - an OpenNIListener, setting up subscribers and callbacks for various formats of RGBD data
 * - a GraphManager, getting Nodes constructed from the RGBD data
 * - A Class providing a service call interface for ROS
 * - If applicable also a GUI
 * - let the above communicate internally via QT Signals, where communcication needs to be across threads or if the communication is conditional on the ROS node's parameterization.
 */
int main(int argc, char** argv)
{
  setlocale(LC_NUMERIC,"C");//Avoid expecting german decimal separators in launch files

  //create thread object, to run the ros event processing loop in parallel to the qt loop
  QtROS qtRos(argc, argv, "rgbdslam"); //ros node name & namespace

  //Depending an use_gui on the Parameter Server, a gui- or a headless application is used
  QCoreApplication* app;
  if(ParameterServer::instance()->get<bool>("use_gui")) {
    app = new QApplication(argc, argv);
  } else {
    app = new QCoreApplication(argc, argv);
  }

  GraphManager graph_mgr;
  //Instantiate the kinect image listener
  OpenNIListener listener(&graph_mgr);
  std::string bagfile_name = ParameterServer::instance()->get<std::string>("bagfile_name");
  if(!bagfile_name.empty()) 
  {
    QObject::connect(&listener, SIGNAL(bagFinished()), app, SLOT(quit()));
    QObject::connect(&listener, SIGNAL(bagFinished()), &qtRos, SLOT(quitNow()));
    listener.loadBagFileFromGUI(bagfile_name.c_str());
    //QtConcurrent::run(&listener, &OpenNIListener::loadBag, bagfile_name);
  }

  Graphical_UI* gui = NULL;
  if (qobject_cast<QApplication*>(app)){
      gui = new Graphical_UI();
      gui->show();
      gui_connections(gui, &graph_mgr, &listener);
      ui_connections(gui, &graph_mgr, &listener);//common connections for the user interfaces
  } else {
      ROS_WARN("Running without graphical user interface! See README or wiki page for how to interact with RGBDSLAM.");
  }
  //Create Ros service interface with or without gui
  RosUi ui("rgbdslam"); //ui namespace for service calls
  ui_connections(&ui, &graph_mgr, &listener);//common connections for the user interfaces

  //If one thread receives a exit signal from the user, signal the other thread to quit too
  QObject::connect(app, SIGNAL(aboutToQuit()), &qtRos, SLOT(quitNow()));
  QObject::connect(&qtRos, SIGNAL(rosQuits()), app, SLOT(quit()));

  qtRos.start();// Run main loop.
  app->exec();
  //if(ros::ok()) ros::shutdown();//If not yet done through the qt connection
  //ros::waitForShutdown(); //not sure if necessary. 
  delete app;
}


