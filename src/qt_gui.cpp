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


/* This is the main widget of the application.
 * It sets up some not yet useful menus and
 * three qlabels in the layout of the central widget
 * that can be used to show qimages via the slots
 * setDepthImage and setVisualImage.
 */
#include <QtGui>
#include <QDir>
#include <QPixmap>
#include <QFont>
#include <QIcon>
#include <QKeySequence>
#include "qt_gui.h"
#include <limits>
#include "glviewer.h"
#include <QList>
#include <QComboBox>

///Constructs a QT GUI for easy control of RGBDSLAM
Graphical_UI::Graphical_UI() : filename("quicksave.pcd"), glviewer(NULL)
{
  setup();
  setWindowTitle(tr("RGBDSLAM"));
}
///Constructs a QT GUI for easy control of RGBDSLAM
Graphical_UI::Graphical_UI(QString title) : filename("quicksave.pcd"), glviewer(NULL)
{
  setup();
  setWindowTitle(title);
}

void Graphical_UI::setup(){
    infoText = new QString(tr(
                "<p><b>RGBDSLAMv2</b> uses visual features to identify corresponding 3D locations "
                "in RGB-D data. The correspondences are used to reconstruct the camera motion. "
                "The SLAM-backend g2o is used to integrate the transformations between"
                "the RGBD-images and compute a globally consistent 6D trajectory.</p>"
                "<p></p>"));
    licenseText = new QString(tr(
                 "<p>RGBDSLAMv2 is free software: you can redistribute it and/or modify"
                 "it under the terms of the GNU General Public License as published by"
                 "the Free Software Foundation, either version 3 of the License, or"
                 "(at your option) any later version.</p>"
                 "<p>RGBDSLAMv2 is distributed in the hope that it will be useful,"
                 "but WITHOUT ANY WARRANTY; without even the implied warranty of"
                 "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the"
                 "GNU General Public License for more details.</p>"
                 "<p>You should have received a copy of the GNU General Public License"
                 "along with RGBDSLAMv2.  If not, refer to <a href=\"http://www.gnu.org/licenses/\">http://www.gnu.org/licenses</a>.</p>"));
    mouseHelpText = new QString(tr(
                "<p><b>3D Viewer Mouse Commands:</b>"
                "<ul><li><i>Left button:</i> Rotate view around x/y.</li>"
                "    <li><i>Ctrl + left button:</i> rotate view around x/z.</li>"
                "    <li><i>Shift + left button:</i> Translate view.</li>"
                "    <li><i>Wheel:</i> zoom view.</li>"
                "    <li><i>Double click (on background):</i> reset camera position to latest pose.</li>"
                "    <li><i>Ctrl + Double click (on background):</i> reset camera position to first pose (only works if follow mode is off).</li>"
                "    <li><i>Double click on object:</i> set pivot to clicked point (only works if follow mode is off).</li>"
                "<ul></p>")); feature_flow_image_label = new QLabel(*mouseHelpText);

    ParameterServer* ps = ParameterServer::instance();
    // create widgets for image and map display
    feature_flow_image_label->setWordWrap(true);
    feature_flow_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    feature_flow_image_label->setMinimumSize(4,3);
    if(ps->get<bool>("scalable_2d_display")) {
      feature_flow_image_label->setScaledContents(true);
    }

    std::string visual_topic = ParameterServer::instance()->get<std::string>("topic_image_mono");
    QString vl("Waiting for visual image on topic<br/><i>\""); vl += visual_topic.c_str(); vl += "\"</i>";
    visual_image_label = new QLabel(vl);
    visual_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    visual_image_label->setAlignment(Qt::AlignCenter);
    visual_image_label->setMinimumSize(4,3);
    if(ps->get<bool>("scalable_2d_display")) {
      visual_image_label->setScaledContents(true);
    }

    std::string depth_topic = ParameterServer::instance()->get<std::string>("topic_image_depth");
    QString dl("Waiting for depth image on topic<br/><i>\""); dl += depth_topic.c_str(); 
    dl += "\"</i><br/>";

    depth_image_label = new QLabel(dl);
    depth_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    depth_image_label->setAlignment(Qt::AlignCenter);
    depth_image_label->setMinimumSize(4,3);
    if(ParameterServer::instance()->get<bool>("scalable_2d_display")) {
      depth_image_label->setScaledContents(true);
    }


    feature_image_label = new QLabel(tr("<i>Waiting for feature image...</i>"));
    feature_image_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    feature_image_label->setAlignment(Qt::AlignCenter);
    feature_image_label->setVisible(false);
    feature_image_label->setMinimumSize(4,3);
    if(ps->get<bool>("scalable_2d_display")) {
      feature_image_label->setScaledContents(true);
    }
    //transform_label = new QLabel(tr("<i>Waiting for transformation matrix...</i>"));
    //transform_label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    //transform_label->setAlignment(Qt::AlignCenter);
    if(ps->get<bool>("use_glwidget")) glviewer = new GLViewer(this);//displays the cloud in 3d

    //QFont typewriter_font;
    //typewriter_font.setStyleHint(QFont::TypeWriter);
    //transform_label->setFont(typewriter_font);

    // setup the layout:
    // use a splitter as main widget
    vsplitter = new QSplitter(Qt::Vertical);
    setCentralWidget(vsplitter);
    // add glviewer as top item to splitter
    if(ParameterServer::instance()->get<bool>("use_glwidget")) vsplitter->addWidget(glviewer);
    // arrange image labels in horizontal layout
    //QHBoxLayout* h_layout = new QHBoxLayout;
    QSplitter* hsplitter = new QSplitter(Qt::Horizontal);
    //h_layout->setSpacing(0);
    //h_layout->setContentsMargins(0, 0, 0, 0);
    hsplitter->addWidget(visual_image_label);
    hsplitter->addWidget(depth_image_label);
    hsplitter->addWidget(feature_image_label);
    hsplitter->addWidget(feature_flow_image_label);
    //QWidget* bottom_widget = new QWidget;
    //bottom_widget->setLayout(h_layout);
    // add them to the splitter
    vsplitter->addWidget(hsplitter);

    createMenus();

    tmpLabel = new QLabel();
    statusBar()->insertWidget(0,tmpLabel, 0);
    QString message = tr("Ready for RGB-D SLAM");
    statusBar()->showMessage(message);
    infoLabel2 = new QLabel(tr("Empty Map"));
    infoLabel2->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    infoLabel2->setAlignment(Qt::AlignRight);
    statusBar()->addPermanentWidget(infoLabel2, 0);
    if(ParameterServer::instance()->get<bool>("start_paused")){
      infoLabel = new QLabel(tr("<i>Press Enter or Space to Start</i>"));
    } else {
      infoLabel = new QLabel(tr("<i>No data yet.</i>"));
    }
    infoLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    infoLabel->setAlignment(Qt::AlignRight);
    statusBar()->addPermanentWidget(infoLabel, 0);

    setMinimumSize(790, 290);
    resize(1000, 700);
}
void Graphical_UI::setFeatureImage(QImage qimage){
  if(feature_image_label->isVisible()){
    feature_image_label->setAlignment(Qt::AlignCenter);
    feature_image_label->setPixmap(QPixmap::fromImage(qimage));
    feature_image_label->repaint();
  }
  feature_image = qimage;
}

void Graphical_UI::setFeatureFlowImage(QImage qimage){
  if(feature_flow_image_label->isVisible()){
    feature_flow_image_label->setAlignment(Qt::AlignCenter);
    feature_flow_image_label->setPixmap(QPixmap::fromImage(qimage));
    feature_flow_image_label->repaint();
  }
}
void Graphical_UI::setVisualImage(QImage qimage){
  if(visual_image_label->isVisible()){
    visual_image_label->setPixmap(QPixmap::fromImage(qimage));
    visual_image_label->repaint();
  }
}

void Graphical_UI::setDepthImage(QImage qimage){
  if(depth_image_label->isVisible()){
      depth_image_label->setPixmap(QPixmap::fromImage(qimage));
      depth_image_label->repaint();
  }
}

void Graphical_UI::reloadConfig() {
    ParameterServer::instance()->getValues();
    QString message = tr("Configuration Reloaded");
    statusBar()->showMessage(message);
}

void Graphical_UI::resetCmd() {
    Q_EMIT reset();
    if(ParameterServer::instance()->get<bool>("use_glwidget")) glviewer->reset();
    QString message = tr("Graph Reset");
    statusBar()->showMessage(message);
    infoLabel->setText("A fresh new graph is waiting");
}

void Graphical_UI::setStatus(QString message){
    statusBar()->showMessage(message);
}
void Graphical_UI::setInfo2(QString message){
    infoLabel2->setText(message);
    infoLabel2->repaint();
}
void Graphical_UI::setInfo(QString message){
    infoLabel->setText(message);
    infoLabel->repaint();
}

void Graphical_UI::saveG2OGraphDialog() {
    QString graph_filename = QFileDialog::getSaveFileName(this, "Save G2O Graph to File", "graph.g2o", tr("G2O (*.g2o)"));
    QString message;
    if(graph_filename.isEmpty()){
      message = tr("Filename empty. Aborting save");
    } else {
      Q_EMIT saveG2OGraph(graph_filename);
      message = tr("Saving Graph");
    }
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::quickSaveAll() {
    Q_EMIT saveAllClouds(filename);
    QString message = tr("Saving Whole Model to ");
    message.append(QDir::currentPath());
    message.append(filename);
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::openBagFileDialog() {
    QString filename = QFileDialog::getOpenFileName(this, "Open Bag File", "", tr("BAG (*.bag)"));
    QString message;
    if(filename.isEmpty()){
      message = tr("Filename empty. Aborting.");
    } else {
      message = tr("Opening Bagfile ");
      message += filename;
      Q_EMIT openBagFile(filename);
    }
    statusBar()->showMessage(message);
}

void Graphical_UI::openPCDFilesDialog() {
    QStringList filenamelist = QFileDialog::getOpenFileNames(this, "Open PCD Files", "", tr("PCD (*.pcd)"));
    QString message;
    if(filenamelist.isEmpty()){
      message =  tr("Empty file list. Aborting");
    } else {
      for(int i=0; i < filenamelist.size(); i++){
        std::cout << qPrintable(filenamelist.at(i)) << std::endl;
      }
      Q_EMIT openPCDFiles(filenamelist);
      message =  tr("Opening PCD Files");
    }
    statusBar()->showMessage(message);
}
void Graphical_UI::saveFeatures() {
    filename = QFileDialog::getSaveFileName(this, "Save Features to File", filename, tr("YAML (*.yml);;XML (*.xml)"));
    QString message;
    if(filename.isEmpty()){
      message = tr("Filename empty. Aborting.");
    } else {
      message = tr("Saving Features to ");
      message += filename;
      Q_EMIT saveAllFeatures(filename);
    }
    statusBar()->showMessage(message);
}

void Graphical_UI::computeOctomap() {
  Q_EMIT computeOctomapSig(filename);
}

void Graphical_UI::saveOctomap() {
    filename = QFileDialog::getSaveFileName(this, "Create and Save Octomap to File", filename, tr("OcTree (*.ot)"));
    QString message;
    if(filename.isEmpty()){
      message = tr("Filename empty. Aborting.");
    } else {
      Q_EMIT saveOctomapSig(filename);
      QString message = tr("Creating Octomap");
    }
    statusBar()->showMessage(message);
}

void Graphical_UI::saveAll() {
    filename = QFileDialog::getSaveFileName(this, "Save Point CLoud to File", filename, tr("PCD (*.pcd);;PLY (*ply)"));
    Q_EMIT saveAllClouds(filename);
    QString message = tr("Saving Whole Model");
    statusBar()->showMessage(message);
}

void Graphical_UI::showEdgeErrors() {
    QString myfilename = QFileDialog::getSaveFileName(this, "Save Current Trajectory Estimate", "trajectory", tr("All Files (*.*)"));
    Q_EMIT printEdgeErrors(myfilename);
    QString message = tr("Triggering Edge Printing");
    statusBar()->showMessage(message);
}
void Graphical_UI::optimizeGraphTrig() {
    Q_EMIT optimizeGraph();
    QString message = tr("Triggering Optimizer");
    statusBar()->showMessage(message);
}
void Graphical_UI::saveVectorGraphic() {
    QMessageBox::warning(this, tr("Don't render to pdf while point clouds are shown"), tr("This is meant for rendereing the pose graph. Rendering clouds to pdf will generate huge files!"));
    QString myfilename = QFileDialog::getSaveFileName(this, "Save Current 3D Display to Vector Graphic", "pose_graph.pdf", tr("All Files (*.*)"));
    if(!myfilename.size() == 0){
      glviewer->drawToPS(myfilename);
      QString message = tr("Drawing current Display");
      statusBar()->showMessage(message);
    } else {
      QString message = tr("Empty Filename");
    }
}
void Graphical_UI::toggleScreencast(bool on) {
  if(on){
    QString myfilename = QFileDialog::getSaveFileName(this, "Path Prefix for Screencast Frames", "rgbdslam_frame_", tr("All Files (*.*)"));
    if(myfilename.size() > 0){
      ParameterServer::instance()->set("screencast_path_prefix", std::string(qPrintable(myfilename)));
      QString message = tr("Saving screencast frames.");
      statusBar()->showMessage(message);
    }
  }
  else {
    ParameterServer::instance()->set("screencast_path_prefix", std::string());
  }
}
void Graphical_UI::saveTrajectoryDialog() {
    QString myfilename = QFileDialog::getSaveFileName(this, "Save Current Trajectory Estimate", "trajectory", tr("All Files (*.*)"));
    Q_EMIT saveTrajectory(myfilename);
    QString message = tr("Saving current trajectory estimate (and possibly ground truth).");
    statusBar()->showMessage(message);
}
void Graphical_UI::saveIndividual() {
    QString tmpfilename(filename);
    tmpfilename.remove(".pcd", Qt::CaseInsensitive);
    tmpfilename.remove(".ply", Qt::CaseInsensitive);

    tmpfilename = QFileDialog::getSaveFileName(this, "Save point cloud to one file per node", tmpfilename, tr("PCD (*.pcd)"));
    tmpfilename.remove(".pcd", Qt::CaseInsensitive);
    Q_EMIT saveIndividualClouds(tmpfilename);
    QString message = tr("Saving Model Node-Wise");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::saveBagDialog() {
    QString filename = QFileDialog::getSaveFileName(this, "Save Clouds to Bag File", "", tr("BAG (*.bag)"));
    Q_EMIT saveBagfile(filename);
    QString message = tr("Writing Whole Model to Bagfile");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}
void Graphical_UI::sendAll() {
    Q_EMIT sendAllClouds();
    QString message = tr("Sending Whole Model");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::setRotationGrid() {
    bool ok;
    double value = QInputDialog::getDouble(this, tr("Set Rotation Stepping in Degree"),
                                           tr("Enter Stepping:"), 1.00, -10000000, 10000000, 2, &ok);
    if(ok){ glviewer->setRotationGrid(value); }
}
void Graphical_UI::setStereoShift() {
    bool ok;
    double value = QInputDialog::getDouble(this, tr("Set Stereo Camera shift"),
                                           tr("Enter Shift:"), 0.10, -10000000, 10000000, 2, &ok);
    if(ok){ glviewer->setStereoShift(value); }
}

void Graphical_UI::setParam(QString text){
    std::map<std::string, boost::any>&  config = ParameterServer::instance()->getConfigData();
    std::map<std::string, boost::any>::const_iterator itr;
    for (itr = config.begin(); itr != config.end(); ++itr) {
      if(itr->first == qPrintable(text)){
        bool ok;
        if (itr->second.type() == typeid(std::string)) {
          QString prev_val = boost::any_cast<std::string>(itr->second).c_str();
          QString new_val = QInputDialog::getText(this, text, tr("New Value:"), 
                                                  QLineEdit::Normal, prev_val, &ok);
          if (ok){
            ParameterServer::instance()->set(itr->first, std::string(qPrintable(new_val)));
          }
        } 
        else if (itr->second.type() == typeid(int)) {
          int prev_val = boost::any_cast<int>(itr->second);
          int new_val = QInputDialog::getInt(this, text, tr("New Value:"), 
                                             prev_val, 
                                             std::numeric_limits<int>::min(), 
                                             std::numeric_limits<int>::max(), 1, &ok);
          if (ok){
            ParameterServer::instance()->set(itr->first, new_val);
          }
        } 
        else if (itr->second.type() == typeid(double)) {
          double prev_val = boost::any_cast<double>(itr->second);
          double new_val = QInputDialog::getDouble(this, text, tr("New Value:"), 
                                             prev_val,
                                             std::numeric_limits<double>::min(), 
                                             std::numeric_limits<double>::max(), 3, &ok);
          if (ok){
            ParameterServer::instance()->set(itr->first, new_val);
          }
        } 
        else if (itr->second.type() == typeid(bool)) {
          QStringList truefalse;
          truefalse.append("False");//item 0 
          truefalse.append("True"); //item 1
          //Per definition the cast from bool to int results in 0 or 1
          int prev_val = boost::any_cast<bool>(itr->second);
          QString new_val = QInputDialog::getItem(this, text, tr("New Value:"), 
                                                  truefalse, prev_val, false, &ok);
          if (ok){
            if(new_val == "False")
              ParameterServer::instance()->set(itr->first, false);
            else
              ParameterServer::instance()->set(itr->first, true);
          }
        }
      }
    }
}


void Graphical_UI::setParam() {
    std::map<std::string, boost::any>&  config = ParameterServer::instance()->getConfigData();
    std::map<std::string, boost::any>::const_iterator itr;
    QStringList list;
    for (itr = config.begin(); itr != config.end(); ++itr) {
      QString name(itr->first.c_str());
      //QString description(ParameterServer::instance()->getDescription(itr->first).c_str());
      list.append(name);
    }
    bool ok = false;
    QString name = QInputDialog::getItem(this, tr("Select Parameter to Change"), 
                                         tr("Note: Changing parameters during runtime is not honored for all parameters.\n"
                                            "In particular changing parameters used during the initial setup "
                                            "(e.g. topic names) will have no effect.\n\n"
                                            "Options should be set via launchfiles!\n\n"
                                            "This functionality is mostly for quickly trying out the effect of "
                                            "individual parameters.\n\nParameter Name"), 
                                         list, 0, false, &ok);
    if(ok) setParam(name);
}
void Graphical_UI::toggleFullscreen(bool mode){
    this->menuBar()->setVisible(!mode);
    this->gridlayout->setMargin(mode?0:5);
    this->statusBar()->setVisible(!mode);
    if(mode){
      this->showFullScreen();
    } else {
      this->showNormal();
    }
}
void Graphical_UI::pruneEdgesWithHighError(){
    bool ok;
    float value = QInputDialog::getDouble(this, tr("Set Max Edge Error"),
                                          tr("No Text"), 1.00, -10000000, 10000000, 4, &ok);
    if(ok){
    	Q_EMIT pruneEdgesWithErrorAbove(value);
    }
}
void Graphical_UI::sendFinished() {
    
    QString message = tr("Finished Sending");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::getOneFrameCmd() {
    Q_EMIT getOneFrame();
    QString message = tr("Getting a single frame");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}
void Graphical_UI::deleteLastFrameCmd() {
    Q_EMIT deleteLastFrame();
    QString message = tr("Deleting the last node from the graph");
    statusBar()->showMessage(message);
    //infoLabel->setText(message);
}

void Graphical_UI::toggleMappingPriv(bool mapping_on) {
    Q_EMIT toggleMapping(mapping_on);
}

void Graphical_UI::bagRecording(bool pause_on) {
    Q_EMIT toggleBagRecording();
    if(pause_on) {
        QString message = tr("Recording Bagfile.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    } else {
        QString message = tr("Stopped Recording.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    }
}
void Graphical_UI::triggerCloudFiltering() {
  Q_EMIT occupancyFilterClouds();
}
void Graphical_UI::setOctoMapResolution() {
  this->setParam("octomap_resolution");
}
void Graphical_UI::toggleOnlineVoxelMapping(bool online) {
  ParameterServer::instance()->set("octomap_online_creation", online);
}
void Graphical_UI::toggleCloudStorage(bool storage) {
  ParameterServer::instance()->set("store_pointclouds", storage);
}
void Graphical_UI::toggleLandmarkOptimization(bool landmarks) {
  ParameterServer::instance()->set("optimize_landmarks", landmarks);
}

void Graphical_UI::pause(bool pause_on) {
    Q_EMIT togglePause();
    if(pause_on) {
        QString message = tr("Processing.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    } else {
        QString message = tr("Stopped processing.");
        statusBar()->showMessage(message);
        //infoLabel->setText(message);
    }
}

void Graphical_UI::help() {
    QMessageBox::about(this, tr("Help Menu"), /**menuHelpText +*/ *mouseHelpText );
}
void Graphical_UI::about() {
    QMessageBox::about(this, tr("About RGBDSLAM"), *infoText + *licenseText);
}

void Graphical_UI::set2DStream(bool is_on) {
    if(is_on){ 
        visual_image_label->show();
        depth_image_label->show(); 
        feature_flow_image_label->show(); 
        feature_image_label->show(); 
        QList<int> list;
        list.append(1);//upper part on
        list.append(1);//lower part on
        vsplitter->setSizes(list);
    } else { 
        visual_image_label->hide(); 
        depth_image_label->hide(); 
        feature_flow_image_label->hide(); 
        feature_image_label->hide(); 
        QList<int> list;
        list.append(1);//upper part on
        list.append(0);//lower part off
        vsplitter->setSizes(list);
    } 
}

/*
void Graphical_UI::set3DDisplay(bool is_on) {
    if(!ParameterServer::instance()->get<bool>("use_glwidget")) return;
    if(is_on){ glviewer->show(); } 
    else { glviewer->hide(); } 
}
*/

void Graphical_UI::createMenus() {
    //these are the menus created here
    QMenu *dataMenu;
    QMenu *octoMapMenu;
    QMenu *actionMenu;
    QMenu *viewMenu3d;
    QMenu *viewMenu2d;
    QMenu *settingsMenu;
    QMenu *helpMenu;

    //Graph Menu
    dataMenu = menuBar()->addMenu(tr("&I/O"));

    QAction *openPCDFilesAct = new QAction(tr("&Open PCD files"), this);
    openPCDFilesAct->setShortcuts(QKeySequence::Open);
    openPCDFilesAct->setStatusTip(tr("Open one or more pcd files to process"));
    openPCDFilesAct->setIcon(QIcon::fromTheme("document-open"));//doesn't work for gnome
    connect(openPCDFilesAct, SIGNAL(triggered()), this, SLOT(openPCDFilesDialog()));
    dataMenu->addAction(openPCDFilesAct);
    this->addAction(openPCDFilesAct);

    QAction *openBagFileAct = new QAction(tr("Open ROS &bag file"), this);
    openBagFileAct->setStatusTip(tr("Open a bag file to process"));
    openBagFileAct->setShortcut(QString("Ctrl+B"));
    openBagFileAct->setIcon(QIcon::fromTheme("document-open"));//doesn't work for gnome
    connect(openBagFileAct, SIGNAL(triggered()), this, SLOT(openBagFileDialog()));
    dataMenu->addAction(openBagFileAct);
    this->addAction(openBagFileAct);


    QAction *quickSaveAct = new QAction(tr("&Save"), this);
    quickSaveAct->setShortcuts(QKeySequence::Save);
    quickSaveAct->setStatusTip(tr("Save all stored point clouds with common coordinate frame to a pcd file"));
    quickSaveAct->setIcon(QIcon::fromTheme("document-save"));//doesn't work for gnome
    connect(quickSaveAct, SIGNAL(triggered()), this, SLOT(quickSaveAll()));
    dataMenu->addAction(quickSaveAct);
    this->addAction(quickSaveAct);

    QAction *saveFeaturesAct = new QAction(tr("Save &Feature Map"), this);
    saveFeaturesAct->setShortcut(QString("Ctrl+F"));
    saveFeaturesAct->setStatusTip(tr("Save all feature positions and descriptions in a common coordinate frame to a yaml or xml file"));
    saveFeaturesAct->setIcon(QIcon::fromTheme("document-save"));//doesn't work for gnome
    connect(saveFeaturesAct, SIGNAL(triggered()), this, SLOT(saveFeatures()));
    dataMenu->addAction(saveFeaturesAct);
    this->addAction(saveFeaturesAct);

    QAction *saveOctoAct = new QAction(tr("Save Octomap"), this);
    saveOctoAct->setStatusTip(tr("Save computed OctoMap"));
    saveOctoAct->setIcon(QIcon::fromTheme("document-save-as"));//doesn't work for gnome
    connect(saveOctoAct, SIGNAL(triggered()), this, SLOT(saveOctomap()));
    dataMenu->addAction(saveOctoAct);
    this->addAction(saveOctoAct);

    QAction *saveAct = new QAction(tr("&Save as Point Cloud ..."), this);
    saveAct->setShortcuts(QKeySequence::SaveAs);
    saveAct->setStatusTip(tr("Save all stored point clouds with common coordinate frame"));
    saveAct->setIcon(QIcon::fromTheme("document-save-as"));//doesn't work for gnome
    connect(saveAct, SIGNAL(triggered()), this, SLOT(saveAll()));
    dataMenu->addAction(saveAct);
    this->addAction(saveAct);

    QAction *saveIndiAct = new QAction(tr("&Save Point Cloud Node-Wise..."), this);
    saveIndiAct->setShortcut(QString("Ctrl+N"));
    saveIndiAct->setStatusTip(tr("Save stored point clouds in individual files"));
    saveAct->setIcon(QIcon::fromTheme("document-save-all"));//doesn't work for gnome
    connect(saveIndiAct, SIGNAL(triggered()), this, SLOT(saveIndividual()));
    dataMenu->addAction(saveIndiAct);
    this->addAction(saveIndiAct);

    QAction *saveG2OGraphAct = new QAction(tr("Save &G2O Graph"), this);
    saveG2OGraphAct->setStatusTip(tr("Save G2O graph (e.g. for use with the g2o viewer or external optimization)"));
    saveG2OGraphAct->setIcon(QIcon::fromTheme("document-save"));//doesn't work for gnome
    connect(saveG2OGraphAct, SIGNAL(triggered()), this, SLOT(saveG2OGraphDialog()));
    dataMenu->addAction(saveG2OGraphAct);
    this->addAction(saveG2OGraphAct);

    QAction *saveBagAct = new QAction(tr("Save Clouds to &Bag"), this);
    saveBagAct->setShortcut(QString("Ctrl+Shift+B"));
    saveBagAct->setStatusTip(tr("Save clouds and transforms to bagfile"));
    connect(saveBagAct, SIGNAL(triggered()), this, SLOT(saveBagDialog()));
    dataMenu->addAction(saveBagAct);
    this->addAction(saveBagAct);

    QAction *saveTrajectoryAct = new QAction(tr("Save Trajectory &Estimate"), this);
    saveTrajectoryAct->setShortcut(QString("Ctrl+E"));
    saveTrajectoryAct->setStatusTip(tr("Save trajectory estimate (and ground truth trajectory if available) for external evaluation."));
    connect(saveTrajectoryAct, SIGNAL(triggered()), this, SLOT(saveTrajectoryDialog()));
    dataMenu->addAction(saveTrajectoryAct);
    this->addAction(saveTrajectoryAct);

    QAction *sendAct = new QAction(tr("&Send Model"), this);
    sendAct->setShortcut(QString("Ctrl+M"));
    sendAct->setStatusTip(tr("Send out all stored point clouds with corrected transform"));
    sendAct->setIcon(QIcon::fromTheme("document-send"));//doesn't work for gnome
    connect(sendAct, SIGNAL(triggered()), this, SLOT(sendAll()));
    dataMenu->addAction(sendAct);
    this->addAction(sendAct);

    dataMenu->addSeparator();

    QAction *toggleLandmarksAct = new QAction(tr("Toggle &Landmark Optimization"), this);
    toggleLandmarksAct->setShortcut(QString("L"));
    toggleLandmarksAct->setCheckable(true);
    toggleLandmarksAct->setChecked(ParameterServer::instance()->get<bool>("optimize_landmarks"));
    toggleLandmarksAct->setStatusTip(tr("Toggle whether pose graph includes landmark positions"));
    connect(toggleLandmarksAct, SIGNAL(toggled(bool)), this, SLOT(toggleLandmarkOptimization(bool)));
    dataMenu->addAction(toggleLandmarksAct);
    this->addAction(toggleLandmarksAct);

    QAction *toggleMappingAct = new QAction(tr("Toggle &Mapping"), this);
    toggleMappingAct->setShortcut(QString("M"));
    toggleMappingAct->setCheckable(true);
    toggleMappingAct->setChecked(true);
    toggleMappingAct->setStatusTip(tr("Toggle between SLAM and Localization"));
    toggleMappingAct->setIcon(QIcon::fromTheme("media-playback-start"));//doesn't work for gnome
    connect(toggleMappingAct, SIGNAL(toggled(bool)), this, SLOT(toggleMappingPriv(bool)));
    dataMenu->addAction(toggleMappingAct);
    this->addAction(toggleMappingAct);

    dataMenu->addSeparator();

    QAction *clearDisplayAct;
    clearDisplayAct = new QAction(tr("&Clear 3D Display"), this);
    clearDisplayAct->setStatusTip(tr("Clears 3D viewer cloud data. Point Clouds are still retained, e.g. for mapping purposes."));
    clearDisplayAct->setIcon(QIcon::fromTheme("edit-delete"));//doesn't work (for gnome
    connect(clearDisplayAct, SIGNAL(triggered()), glviewer, SLOT(reset()));
    dataMenu->addAction(clearDisplayAct);
    this->addAction(clearDisplayAct);

    QAction *clearCloudsAct = new QAction(tr("Clear Cloud Storage"), this);
    clearCloudsAct->setStatusTip(tr("Remove Point Clouds from Memory"));
    clearDisplayAct->setIcon(QIcon::fromTheme("edit-delete"));//doesn't work (for gnome
    connect(clearCloudsAct, SIGNAL(triggered()), this, SIGNAL(clearClouds()));
    dataMenu->addAction(clearCloudsAct);
    this->addAction(clearCloudsAct);

    dataMenu->addSeparator();

    QAction *optimizeAct = new QAction(tr("Optimize Trajectory &Estimate"), this);
    optimizeAct->setShortcut(QString("O"));
    optimizeAct->setStatusTip(tr("Compute optimized pose graph with g2o"));
    connect(optimizeAct, SIGNAL(triggered()), this, SLOT(optimizeGraphTrig()));
    dataMenu->addAction(optimizeAct);
    this->addAction(optimizeAct);

    dataMenu->addSeparator();

    QAction *exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcut(QString("Ctrl+Q")); //doesn't work: QKeySequence::Quit);
    exitAct->setStatusTip(tr("Exit the application"));
    exitAct->setIcon(QIcon::fromTheme("application-exit"));//doesn't work for gnome
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));
    dataMenu->addAction(exitAct);
    this->addAction(exitAct);

    octoMapMenu = menuBar()->addMenu(tr("&OctoMap"));
    octoMapMenu->addAction(saveOctoAct);

    /* Separation of computation and saving of octomap not yet Implemented
    QAction *computeOctoAct = new QAction(tr("Compute Octomap"), this);
    //computeOctoAct->setShortcuts(QKeySequence::SaveAs);
    computeOctoAct->setStatusTip(tr("Create OctoMap from stored point clouds"));
    connect(computeOctoAct, SIGNAL(triggered()), this, SLOT(computeOctomap()));
    octoMapMenu->addAction(computeOctoAct);
    this->addAction(computeOctoAct);
    */


    QAction *occupancyFilterAct = new QAction(tr("Point Cloud Occupancy Filter"), this);
    //occupancyFilterAct->setShortcuts(QKeySequence::SaveAs);
    occupancyFilterAct->setStatusTip(tr("Remove points from the cloud that fall into unoccupied voxels of the OctoMap"));
    connect(occupancyFilterAct, SIGNAL(triggered()), this, SLOT(triggerCloudFiltering()));
    octoMapMenu->addAction(occupancyFilterAct);
    this->addAction(occupancyFilterAct);
    QAction *setOctoMapResolutionAct = new QAction(tr("Octomap Resolution"), this);
    //setOctoMapResolutionAct->setShortcuts(QKeySequence::SaveAs);
    setOctoMapResolutionAct->setStatusTip(tr("Change the octomap resolution. Clears previously created maps on next update."));
    connect(setOctoMapResolutionAct, SIGNAL(triggered()), this, SLOT(setOctoMapResolution()));
    octoMapMenu->addAction(setOctoMapResolutionAct);
    this->addAction(setOctoMapResolutionAct);

    QAction *toggleOnlineVoxelMappingAct = new QAction(tr("&Online OctoMapping"), this);
    toggleOnlineVoxelMappingAct->setCheckable(true);
    toggleOnlineVoxelMappingAct->setChecked(ParameterServer::instance()->get<bool>("octomap_online_creation"));
    toggleOnlineVoxelMappingAct->setStatusTip(tr("Toggle Online/Offline OctoMapping. Make sure to set a low octomap_resolution and/or high cloud_creation_skip_step for online mapping"));
    connect(toggleOnlineVoxelMappingAct, SIGNAL(toggled(bool)), this, SLOT(toggleOnlineVoxelMapping(bool)));
    octoMapMenu->addAction(toggleOnlineVoxelMappingAct);
    this->addAction(toggleOnlineVoxelMappingAct);


    //Processing Menu
    actionMenu = menuBar()->addMenu(tr("&Processing"));

    QAction *newAct;
    newAct = new QAction(tr("&Reset"), this);
    newAct->setShortcut(QString("Ctrl+R"));
    newAct->setStatusTip(tr("Reset the graph, clear all data collected"));
    newAct->setIcon(QIcon::fromTheme("edit-delete"));//doesn't work (for gnome
    connect(newAct, SIGNAL(triggered()), this, SLOT(resetCmd()));
    actionMenu->addAction(newAct);
    this->addAction(newAct);

    QAction *pauseAct = new QAction(tr("&Process"), this);
    pauseAct->setShortcut(QString(" "));
    pauseAct->setCheckable(true);
    pauseAct->setChecked(!ParameterServer::instance()->get<bool>("start_paused"));
    pauseAct->setStatusTip(tr("Start/stop processing of frames"));
    pauseAct->setIcon(QIcon::fromTheme("media-playback-start"));//doesn't work for gnome
    connect(pauseAct, SIGNAL(toggled(bool)), this, SLOT(pause(bool)));
    actionMenu->addAction(pauseAct);
    this->addAction(pauseAct);

    QAction *oneFrameAct = new QAction(tr("Capture One& Frame"), this);
    oneFrameAct->setShortcuts(QKeySequence::InsertParagraphSeparator);
    oneFrameAct->setStatusTip(tr("Process one frame only"));
    connect(oneFrameAct, SIGNAL(triggered()), this, SLOT(getOneFrameCmd()));
    actionMenu->addAction(oneFrameAct);
    this->addAction(oneFrameAct);

    QAction *videoStreamAct = new QAction(tr("Capture Screencast"), this);
    videoStreamAct->setCheckable(true);
    videoStreamAct->setChecked(!ParameterServer::instance()->get<std::string>("screencast_path_prefix").empty());
    connect(videoStreamAct, SIGNAL(toggled(bool)), this, SLOT(toggleScreencast(bool)));
    actionMenu->addAction(videoStreamAct);
    this->addAction(videoStreamAct);

    QAction *delFrameAct = new QAction(tr("&Delete Last Node"), this);
    delFrameAct->setShortcut(QString("Backspace"));
    delFrameAct->setStatusTip(tr("Remove last node from graph"));
    delFrameAct->setIcon(QIcon::fromTheme("edit-undo"));//doesn't work for gnome
    connect(delFrameAct, SIGNAL(triggered()), this, SLOT(deleteLastFrameCmd()));
    actionMenu->addAction(delFrameAct);
    this->addAction(delFrameAct);

    QAction *bagRecordingAct = new QAction(tr("&Bagfile Recording"), this);
    bagRecordingAct->setShortcut(QString("R"));
    bagRecordingAct->setCheckable(true);
    bagRecordingAct->setChecked(false);
    bagRecordingAct->setStatusTip(tr("Start/stop recording of frames to bagfile"));
    bagRecordingAct->setIcon(QIcon::fromTheme("media-record"));//doesn't work for gnome
    connect(bagRecordingAct, SIGNAL(toggled(bool)), this, SLOT(bagRecording(bool)));
    actionMenu->addAction(bagRecordingAct);
    this->addAction(bagRecordingAct);



    /*
    QAction *showErrorAct = new QAction(tr("Show Edge Errors"), this);
    showErrorAct->setShortcut(QString("Ctrl+Shift+E"));
    showErrorAct->setStatusTip(tr(""));
    connect(showErrorAct, SIGNAL(triggered()), this, SLOT(showEdgeErrors()));
    actionMenu->addAction(showErrorAct);
    this->addAction(showErrorAct);
    */

    QAction *pruneAct = new QAction(tr("Set Ma&ximum Edge Error"), this);
    pruneAct->setShortcut(QString("Ctrl+X"));
    pruneAct->setStatusTip(tr("Set the Maximum Allowed for Edges"));
    connect(pruneAct, SIGNAL(triggered()), this, SLOT(pruneEdgesWithHighError()));
    actionMenu->addAction(pruneAct);
    this->addAction(pruneAct);

    QAction *psOutputAct = new QAction(tr("&Write PDF File"), this);
    psOutputAct->setShortcut(QString("W"));
    psOutputAct->setStatusTip(tr("Write 3D Scene to a PDF File. Warning: Meant for Pose Graphs not for the clouds"));
    psOutputAct->setIcon(QIcon::fromTheme("application-pdf"));//doesn't work for gnome
    connect(psOutputAct, SIGNAL(triggered()), this, SLOT(saveVectorGraphic()));
    actionMenu->addAction(psOutputAct);
    this->addAction(psOutputAct);

    QAction *saveImages = new QAction(tr("&Write All Images to file."), this);
    saveImages->setStatusTip(tr("Write All images shown in the gui to appropriate files"));
    saveImages->setIcon(QIcon::fromTheme("application-pdf"));//doesn't work for gnome
    connect(saveImages, SIGNAL(triggered()), this, SLOT(saveAllImages()));
    actionMenu->addAction(saveImages);
    this->addAction(saveImages);
    QAction *toggleCloudStorageAct = new QAction(tr("&Store Point Clouds"), this);
    QList<QKeySequence> tcs_shortcuts;
    tcs_shortcuts.append(QString("Ctrl+P"));
    toggleCloudStorageAct->setShortcuts(tcs_shortcuts);
    toggleCloudStorageAct->setCheckable(true);
    toggleCloudStorageAct->setChecked(ParameterServer::instance()->get<bool>("store_pointclouds"));
    toggleCloudStorageAct->setStatusTip(tr("Toggle storing of point clouds (for later sending, map creation)"));
    toggleCloudStorageAct->setIcon(QIcon::fromTheme("server-database"));//doesn't work for gnome
    connect(toggleCloudStorageAct, SIGNAL(toggled(bool)), this, SLOT(toggleCloudStorage(bool)));
    actionMenu->addAction(toggleCloudStorageAct);
    this->addAction(toggleCloudStorageAct);


    //View Menus ###############################################################
    //View Menus ###############################################################

    if(ParameterServer::instance()->get<bool>("use_glwidget"))
    {
      viewMenu3d = menuBar()->addMenu(tr("&3D View"));


      /* Crashes the program even if neither 2d nor 3d widget are activated *
      QAction *toggleFullscreenAct = new QAction(tr("&Fullscreen"), this);
      //QList<QKeySequence> shortcuts;
      //shortcuts.append(QString("F"));
      //toggleFullscreenAct->setShortcuts(shortcuts);
      toggleFullscreenAct->setCheckable(true);
      toggleFullscreenAct->setChecked(false);
      toggleFullscreenAct->setStatusTip(tr("Toggle Fullscreen"));
      toggleFullscreenAct->setIcon(QIcon::fromTheme("view-fullscreen"));//doesn't work for gnome
      connect(toggleFullscreenAct, SIGNAL(toggled(bool)), this, SLOT(toggleFullscreen(bool)));
      viewMenu3d->addAction(toggleFullscreenAct);
      this->addAction(toggleFullscreenAct);
      */
      QAction *toggleGLViewerAct = new QAction(tr("Toggle &3D Display"), this);
      toggleGLViewerAct->setShortcut(QString("3"));
      toggleGLViewerAct->setCheckable(true);
      toggleGLViewerAct->setChecked(true);
      toggleGLViewerAct->setStatusTip(tr("Turn off the OpenGL Display"));
      connect(toggleGLViewerAct, SIGNAL(toggled(bool)), glviewer, SLOT(setVisible(bool)));
      viewMenu3d->addAction(toggleGLViewerAct);
      this->addAction(toggleGLViewerAct);

      QAction *toggleTriangulationAct = new QAction(tr("&Toggle Triangulation"), this);
      toggleTriangulationAct->setShortcut(QString("T"));
      toggleTriangulationAct->setStatusTip(tr("Switch between surface, wireframe and point cloud"));
      connect(toggleTriangulationAct, SIGNAL(triggered(bool)), glviewer, SLOT(toggleTriangulation()));
      viewMenu3d->addAction(toggleTriangulationAct);
      this->addAction(toggleTriangulationAct);

      QAction *toggleFollowAct = new QAction(tr("Follow &Camera"), this);
      toggleFollowAct->setShortcut(QString("Shift+F"));
      toggleFollowAct->setCheckable(true);
      toggleFollowAct->setChecked(true);
      toggleFollowAct->setStatusTip(tr("Always use viewpoint of last frame (except zoom)"));
      connect(toggleFollowAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleFollowMode(bool)));
      viewMenu3d->addAction(toggleFollowAct);
      this->addAction(toggleFollowAct);

      QAction *toggleShowGrid = new QAction(tr("Show Grid"), this);
      toggleShowGrid->setCheckable(true);
      toggleShowGrid->setChecked(false);
      toggleShowGrid->setStatusTip(tr("Display XY plane grid"));
      connect(toggleShowGrid, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowGrid(bool)));
      viewMenu3d->addAction(toggleShowGrid);
      this->addAction(toggleShowGrid);

      QAction *toggleShowTFs = new QAction(tr("Show Pose TFs"), this);
      toggleShowTFs->setCheckable(true);
      toggleShowTFs->setChecked(false);
      toggleShowTFs->setStatusTip(tr("Display pose transformations at axes"));
      connect(toggleShowTFs, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowTFs(bool)));
      viewMenu3d->addAction(toggleShowTFs);
      this->addAction(toggleShowTFs);

      QAction *toggleShowIDsAct = new QAction(tr("Show Pose IDs"), this);
      toggleShowIDsAct->setShortcut(QString("I"));
      toggleShowIDsAct->setCheckable(true);
      toggleShowIDsAct->setChecked(false);
      toggleShowIDsAct->setStatusTip(tr("Display pose ids at axes"));
      connect(toggleShowIDsAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowIDs(bool)));
      viewMenu3d->addAction(toggleShowIDsAct);
      this->addAction(toggleShowIDsAct);

      QAction *toggleShowPosesAct = new QAction(tr("Show &Poses of Graph"), this);
      toggleShowPosesAct->setShortcut(QString("P"));
      toggleShowPosesAct->setCheckable(true);
      toggleShowPosesAct->setChecked(ParameterServer::instance()->get<bool>("show_axis"));
      toggleShowPosesAct->setStatusTip(tr("Display poses as axes"));
      connect(toggleShowPosesAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowPoses(bool)));
      viewMenu3d->addAction(toggleShowPosesAct);
      this->addAction(toggleShowPosesAct);

      QAction *toggleShowEdgesAct = new QAction(tr("Show &Edges of Graph"), this);
      toggleShowEdgesAct->setShortcut(QString("E"));
      toggleShowEdgesAct->setCheckable(true);
      toggleShowEdgesAct->setChecked(ParameterServer::instance()->get<bool>("show_axis"));
      toggleShowEdgesAct->setStatusTip(tr("Display edges of pose graph as lines"));
      connect(toggleShowEdgesAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowEdges(bool)));
      viewMenu3d->addAction(toggleShowEdgesAct);
      this->addAction(toggleShowEdgesAct);

      QAction *toggleStereoAct = new QAction(tr("Stere&o View"), this);
      //toggleStereoAct->setShortcut(QString(""));
      toggleStereoAct->setCheckable(true);
      toggleStereoAct->setChecked(false);
      toggleStereoAct->setStatusTip(tr("Split screen view with slightly shifted Camera"));
      connect(toggleStereoAct, SIGNAL(toggled(bool)), glviewer, SLOT(toggleStereo(bool)));
      viewMenu3d->addAction(toggleStereoAct);
      this->addAction(toggleStereoAct);

      QAction *toggleShowFeatures = new QAction(tr("Show &Feature Locations"), this);
      toggleShowFeatures->setCheckable(true);
      toggleShowFeatures->setChecked(false);
      toggleShowFeatures->setStatusTip(tr("Toggle whether feature locations should be rendered"));
      connect(toggleShowFeatures, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowFeatures(bool)));
      viewMenu3d->addAction(toggleShowFeatures);
      this->addAction(toggleShowFeatures);

      QAction *toggleOctoMapDisplay = new QAction(tr("Show &Octomap"), this);
      toggleOctoMapDisplay->setShortcut(QString("Ctrl+Shift+O"));
      toggleOctoMapDisplay->setCheckable(true);
      toggleOctoMapDisplay->setChecked(true);
      toggleOctoMapDisplay->setStatusTip(tr("Toggle whether octomap should be displayed"));
      connect(toggleOctoMapDisplay, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowOctoMap(bool)));
      viewMenu3d->addAction(toggleOctoMapDisplay);
      this->addAction(toggleOctoMapDisplay);

      QAction *toggleCloudDisplay = new QAction(tr("Show &Clouds"), this);
      toggleCloudDisplay->setShortcut(QString("C"));
      toggleCloudDisplay->setCheckable(true);
      toggleCloudDisplay->setChecked(true);
      toggleCloudDisplay->setStatusTip(tr("Toggle whether point clouds should be rendered"));
      connect(toggleCloudDisplay, SIGNAL(toggled(bool)), glviewer, SLOT(toggleShowClouds(bool)));
      viewMenu3d->addAction(toggleCloudDisplay);
      this->addAction(toggleCloudDisplay);

      QAction *toggleBGColor = new QAction(tr("Toggle Background"), this);
      toggleBGColor->setShortcut(QString("B"));
      toggleBGColor->setCheckable(true);
      toggleBGColor->setChecked(true);
      toggleBGColor->setStatusTip(tr("Toggle whether background should be white or black"));
      connect(toggleBGColor, SIGNAL(toggled(bool)), glviewer, SLOT(toggleBackgroundColor(bool)));
      viewMenu3d->addAction(toggleBGColor);
      this->addAction(toggleBGColor);

      QAction *setRotationGridAct = new QAction(tr("Set Rotation &Grid"), this);
      setRotationGridAct->setShortcut(QString("G"));
      setRotationGridAct->setStatusTip(tr("Discretize Rotation in Viewer"));
      connect(setRotationGridAct, SIGNAL(triggered()), this, SLOT(setRotationGrid()));
      viewMenu3d->addAction(setRotationGridAct);
      this->addAction(setRotationGridAct);

      QAction *setShiftAct = new QAction(tr("Set Stereo Offset"), this);
      setShiftAct->setShortcut(QString("<"));
      setShiftAct->setStatusTip(tr("Set the distance between the virtual cameras for stereo view"));
      connect(setShiftAct, SIGNAL(triggered()), this, SLOT(setStereoShift()));
      viewMenu3d->addAction(setShiftAct);
      this->addAction(setShiftAct);

    }



    //2D View Menu ###############################################################
    viewMenu2d = menuBar()->addMenu(tr("&2D View"));

    QAction *toggleStreamAct = new QAction(tr("Toggle &2D Stream"), this);
    toggleStreamAct->setShortcut(QString("2"));
    toggleStreamAct->setCheckable(true);
    toggleStreamAct->setChecked(true);
    toggleStreamAct->setStatusTip(tr("Turn off the Image Stream"));
    connect(toggleStreamAct, SIGNAL(toggled(bool)), this, SLOT(set2DStream(bool)));
    viewMenu2d->addAction(toggleStreamAct);
    this->addAction(toggleStreamAct);


    QAction *setLabelVisibility1 = new QAction(tr("Show Visual Image"), this);
    setLabelVisibility1->setStatusTip(tr("Show/Hide visual (color/monochrome) image."));
    setLabelVisibility1->setCheckable(true);
    setLabelVisibility1->setChecked(true);
    connect(setLabelVisibility1, SIGNAL(toggled(bool)), visual_image_label, SLOT(setVisible(bool)));
    viewMenu2d->addAction(setLabelVisibility1);
    this->addAction(setLabelVisibility1);

    QAction *setLabelVisibility2 = new QAction(tr("Show Depth Image"), this);
    setLabelVisibility2->setStatusTip(tr("Show/Hide depth image."));
    setLabelVisibility2->setCheckable(true);
    setLabelVisibility2->setChecked(true);
    connect(setLabelVisibility2, SIGNAL(toggled(bool)), depth_image_label, SLOT(setVisible(bool)));
    viewMenu2d->addAction(setLabelVisibility2);
    this->addAction(setLabelVisibility2);

    QAction *setLabelVisibility3 = new QAction(tr("Show Keypoint Image"), this);
    setLabelVisibility3->setStatusTip(tr("Show/Hide image with keypoints."));
    setLabelVisibility3->setCheckable(true);
    setLabelVisibility3->setChecked(false);
    connect(setLabelVisibility3, SIGNAL(toggled(bool)), feature_image_label, SLOT(setVisible(bool)));
    viewMenu2d->addAction(setLabelVisibility3);
    this->addAction(setLabelVisibility3);

    QAction *setLabelVisibility4 = new QAction(tr("Show Visual Flow Image"), this);
    setLabelVisibility4->setStatusTip(tr("Show/Hide image with sparse feature flow (arrows)."));
    setLabelVisibility4->setCheckable(true);
    setLabelVisibility4->setChecked(true);
    connect(setLabelVisibility4, SIGNAL(toggled(bool)), feature_flow_image_label, SLOT(setVisible(bool)));
    viewMenu2d->addAction(setLabelVisibility4);
    this->addAction(setLabelVisibility4);

    //Settings Menu
    settingsMenu = menuBar()->addMenu(tr("&Settings"));

    QAction *reloadAct;
    reloadAct = new QAction(tr("&Reload Config"), this);
    reloadAct->setStatusTip(tr("Reload Configuration from Parameter Server."));
    reloadAct->setIcon(QIcon::fromTheme("reload"));//doesn't work (for gnome
    connect(reloadAct, SIGNAL(triggered()), this, SLOT(reloadConfig()));
    settingsMenu->addAction(reloadAct);
    this->addAction(reloadAct);

    QAction *optionAct = new QAction(tr("&View Current Settings"), this);
    optionAct->setShortcut(QString("?"));
    optionAct->setStatusTip(tr("Display the currently active options"));
    connect(optionAct, SIGNAL(triggered()), this, SLOT(showOptions()));
    settingsMenu->addAction(optionAct);
    this->addAction(optionAct);

    QAction *setAct = new QAction(tr("Set internal &Parameter"), this);
    setAct->setStatusTip(tr("Change a parameter (This will also change the value on the ROS Parameter server)"));
    connect(setAct, SIGNAL(triggered()), this, SLOT(setParam()));
    settingsMenu->addAction(setAct);
    this->addAction(setAct);


    //Help Menu
    helpMenu = menuBar()->addMenu(tr("&Help"));

    QAction *helpAct = new QAction(tr("&Usage Help"), this);
    helpAct->setShortcuts(QKeySequence::HelpContents);
    helpAct->setStatusTip(tr("Show usage information"));
    helpAct->setIcon(QIcon::fromTheme("help-contents"));//doesn't work for gnome
    connect(helpAct, SIGNAL(triggered()), this, SLOT(help()));
    helpMenu->addAction(helpAct);
    this->addAction(helpAct);

    QAction *aboutAct = new QAction(tr("&About RGBDSLAM"), this);
    aboutAct->setShortcut(QString("Ctrl+A"));
    aboutAct->setStatusTip(tr("Show information about RGBDSLAM"));
    aboutAct->setIcon(QIcon::fromTheme("help-about"));//doesn't work for gnome
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
    helpMenu->addAction(aboutAct);
    this->addAction(aboutAct);

}

GLViewer* Graphical_UI::getGLViewer() { 
  return glviewer; 
}

void Graphical_UI::showOptions(){
  QScrollArea* scrollarea = new QScrollArea();
  scrollarea->setMinimumWidth(600);
  QWidget* scrollarea_content = new QWidget(scrollarea);
  scrollarea_content->setMinimumWidth(500);
  QGridLayout *gridLayout = new QGridLayout(scrollarea_content);
  //gridLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
  //gridLayout->setLabelAlignment(Qt::AlignLeft);
  gridLayout->setVerticalSpacing(10);
  gridLayout->setContentsMargins(10,10,10,10);
  QLabel* intro = new QLabel("<h3>This is an read-only view of the current settings. To modify RGBDSLAM's settings use the ROS parameter server functionality (i.e. set options either in a launch-file or as a command line parameter).</h3>");
  intro->setAlignment(Qt::AlignJustify);
  intro->setWordWrap(true);
  intro->setMinimumWidth(560);
  intro->setMinimumHeight(60);
  gridLayout->addWidget(intro,0,0,1,-1, Qt::AlignJustify);

  QLabel namevalue_header("Parameter Name and Value");
  gridLayout->addWidget(&namevalue_header, 1, 0);
  QLabel description_header("Parameter Description");
  gridLayout->addWidget(&description_header, 1, 1);

  //Iterate through parameters
  std::map<std::string, boost::any>&  config = ParameterServer::instance()->getConfigData();
  std::map<std::string, boost::any>::const_iterator itr = config.begin();
  for (int row=2; itr != config.end(); ++itr, row+=3) {
    //Name and Description
    QLabel* name_lbl = new QLabel(QString("<b>") + itr->first.c_str() + QString("</b>"), scrollarea_content);
    name_lbl->setTextFormat(Qt::RichText);
    name_lbl->setAlignment(Qt::AlignLeft);
    gridLayout->addWidget(name_lbl, row, 0);
    QLabel* description = new QLabel(ParameterServer::instance()->getDescription(itr->first).c_str());
    description->setAlignment(Qt::AlignJustify);
    description->setWordWrap(true);
    gridLayout->addWidget(description, row+1, 0, 1,-1);

    //Value
    QString val_txt;
    if (itr->second.type() == typeid(std::string)) {
      val_txt = QString::fromStdString(boost::any_cast<std::string>(itr->second));
    } else if (itr->second.type() == typeid(int)) {
      val_txt = QString::number(boost::any_cast<int>(itr->second));
    } else if (itr->second.type() == typeid(double)) {
      val_txt = QString::number(boost::any_cast<double>(itr->second));
    } else if (itr->second.type() == typeid(bool)) {
      val_txt = boost::any_cast<bool>(itr->second)? "True" : "False";
    }
    QLabel* val_lbl = new QLabel(QString("<tt>") + val_txt + QString("</tt>"), scrollarea_content);
    val_lbl->setTextFormat(Qt::RichText);
    val_lbl->setLineWidth(1);
    val_lbl->setFrameStyle(QFrame::Panel|QFrame::Sunken);
    //val_lbl->setAlignment(Qt::AlignTop|Qt::AlignLeft);
    gridLayout->addWidget(val_lbl, row, 1);

    //Separator
    QFrame* line = new QFrame(scrollarea_content);
    line->setGeometry(QRect(500, 150, 118, 3));
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    gridLayout->addWidget(line, row+2, 0, 1,-1);
  }


  scrollarea_content->setLayout(gridLayout);
  scrollarea->setWidget(scrollarea_content);
  scrollarea->show();
  scrollarea->raise();
  scrollarea->activateWindow();
}


void Graphical_UI::showBusy(int id, const char* message, int max){
  bool contained = progressbars.contains(id);
  QProgressBar* busydialog = NULL;
  if(!contained) {
    busydialog = new QProgressBar();
    busydialog->resize(100, 10);
    progressbars[id] = busydialog;
    statusBar()->insertPermanentWidget(progressbars.size(), busydialog);
  } else {
    busydialog = progressbars[id];
  }
  busydialog->show();
  busydialog->setMinimum(0);
  busydialog->setMaximum(max);
  busydialog->setFormat(QString(message) + " %p%");
}

void Graphical_UI::setBusy(int id, const char* message, int val){
  if(progressbars.contains(id)) {
    QProgressBar* busydialog = progressbars[id];
    busydialog->show();
    if(val > busydialog->maximum()){
      statusBar()->removeWidget(busydialog);
      busydialog->hide();
      statusBar()->showMessage(message);
    } else {
      busydialog->setValue(val);
      busydialog->setFormat(QString(message) + " %p%");
      busydialog->update();
    }
  } else 
    statusBar()->showMessage("Error: Set Value for non-existing progressbar");
}

void Graphical_UI::saveAllImages() {
	QString tmp="~";
    QString file_basename = QFileDialog::getSaveFileName(this, "Save all images to file", tmp, tr("PNG (*.png)"));
    file_basename.remove(".png", Qt::CaseInsensitive);

    QString depth_file =file_basename+"-depth.png";
    QString feature_file =file_basename+"-feature.png";
    QString flow_file =file_basename+"-flow.png";
    QString visual_file =file_basename+"-visual.png";
    QString vector_file =file_basename+"-points.ps";
    std::cout << visual_file.toStdString() << std::endl;
    depth_image.save(depth_file);
    feature_image.save(feature_file);
    feature_flow_image.save(flow_file);
    visual_image.save(visual_file);

    //glviewer->drawToPS(vector_file);
    QString message = tr("Saving all images.");
    statusBar()->showMessage(message);
}
