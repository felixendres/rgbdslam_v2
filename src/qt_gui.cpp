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

///Apply consistent format for image labels
static void formatImageLabel(QLabel* label){
    label->setWordWrap(true);//for infotext in rightmost label
    label->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
    label->setMinimumSize(4,100);
    label->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Expanding);

    if(ParameterServer::instance()->get<bool>("scalable_2d_display")) {
      label->setScaledContents(true);
    }
}

void Graphical_UI::initTexts(){
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
                "    <li><i>Ctrl + Wheel:</i> change 3D point size.</li>"
                "    <li><i>Double click (on background):</i> reset camera position to latest pose.</li>"
                "    <li><i>Ctrl + Double click (on background):</i> reset camera position to first pose (only works if follow mode is off).</li>"
                "    <li><i>Double click on object:</i> set pivot to clicked point (only works if follow mode is off).</li>"
                "<ul></p>")); 
}

void Graphical_UI::setup(){
    initTexts();    

    ParameterServer* ps = ParameterServer::instance();

    // create widgets for image and map display
    std::string visual_topic = ps->get<std::string>("topic_image_mono");
    QString vl("Waiting for visual image on topic<br/><i>\""); vl += visual_topic.c_str(); vl += "\"</i>";
    visual_image_label = new QLabel(vl);
    formatImageLabel(visual_image_label);

    std::string depth_topic = ps->get<std::string>("topic_image_depth");
    QString dl("Waiting for depth image on topic<br/><i>\""); dl += depth_topic.c_str(); 
    dl += "\"</i><br/>";

    depth_image_label = new QLabel(dl);
    formatImageLabel(depth_image_label);

    feature_image_label = new QLabel(tr("<i>Waiting for feature image...</i>"));
    formatImageLabel(feature_image_label);
    
    feature_flow_image_label = new QLabel(*mouseHelpText);
    formatImageLabel(feature_flow_image_label);

    QSplitter* hsplitter = new QSplitter(Qt::Horizontal);
    hsplitter->addWidget(visual_image_label);
    hsplitter->addWidget(depth_image_label);
    hsplitter->addWidget(feature_image_label);
    hsplitter->addWidget(feature_flow_image_label);

    // setup the layout:
    // use a splitter as main widget
    vsplitter = new QSplitter(Qt::Vertical);
    setCentralWidget(vsplitter);

    // add glviewer as top item to splitter
    if(ps->get<bool>("use_glwidget")) {
      glviewer = new GLViewer(this);//displays the cloud in 3d
      vsplitter->addWidget(glviewer);
    }

    vsplitter->addWidget(hsplitter);

    createMenus();

    setupStatusbar();

    setMinimumSize(600, 290);
    resize(1000, 700);
}

void Graphical_UI::setupStatusbar(){
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

}




void Graphical_UI::setLabelToImage(QLabel* label, QImage image){
  if(label->isVisible()){
    label->setMaximumHeight(image.size().height());
    label->setMinimumHeight(image.size().height()/2);
    //label->setMinimumSize(QSize(image.size().width()/2, image.size().height()/2));
    label->setAlignment(Qt::AlignCenter);
    label->setPixmap(QPixmap::fromImage(image));
    label->repaint();
  }
}

void Graphical_UI::setFeatureImage(QImage qimage){
  setLabelToImage(feature_image_label, qimage);
}

void Graphical_UI::setFeatureFlowImage(QImage qimage){
  setLabelToImage(feature_flow_image_label, qimage);
}

void Graphical_UI::setVisualImage(QImage qimage){
  setLabelToImage(visual_image_label, qimage);
}

void Graphical_UI::setDepthImage(QImage qimage){
  setLabelToImage(depth_image_label, qimage);
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
}
void Graphical_UI::sendAll() {
    Q_EMIT sendAllClouds();
    QString message = tr("Sending Whole Model");
    statusBar()->showMessage(message);
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
}

void Graphical_UI::getOneFrameCmd() {
    Q_EMIT getOneFrame();
    QString message = tr("Getting a single frame");
    statusBar()->showMessage(message);
}
void Graphical_UI::deleteLastFrameCmd() {
    Q_EMIT deleteLastFrame();
    QString message = tr("Deleting the last node from the graph");
    statusBar()->showMessage(message);
}

void Graphical_UI::toggleMappingPriv(bool mapping_on) {
    Q_EMIT toggleMapping(mapping_on);
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
    } else {
        QString message = tr("Stopped processing.");
        statusBar()->showMessage(message);
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
QAction* Graphical_UI::newAction(QMenu* menu, const char* title, const char* statustip, QIcon icon){
    QAction *new_action = new QAction(tr(title), this);
    new_action->setStatusTip(tr(statustip));
    new_action->setIcon(icon);
    this->addAction(new_action);
    menu->addAction(new_action);
    return new_action;
}

QAction* Graphical_UI::newMenuItem(QMenu* menu, const char* title, QObject* receiver, const char* callback, const char* statustip, const char* key_seq_str, QIcon icon){
    QAction *new_action = newAction(menu, title, statustip, icon);
    new_action->setShortcut(QString(key_seq_str));
    connect(new_action, SIGNAL(triggered()), receiver, callback);
    return new_action;
}
QAction* Graphical_UI::newMenuItem(QMenu* menu, const char* title, const char* callback_method_of_this_class, const char* statustip, QKeySequence::StandardKey std_key, QIcon icon){
    QAction *new_action = newAction(menu, title, statustip, icon);
    new_action->setShortcuts(std_key);
    connect(new_action, SIGNAL(triggered()), this, callback_method_of_this_class);
    return new_action;
}
QAction* Graphical_UI::newMenuItem(QMenu* menu, const char* title, const char* callback_method_of_this_class, const char* statustip, const char* key_seq_str, QIcon icon){
    QAction *new_action = newAction(menu, title, statustip, icon);
    new_action->setShortcut(QString(key_seq_str));
    connect(new_action, SIGNAL(triggered()), this, callback_method_of_this_class);
    return new_action;
}
QAction* Graphical_UI::newMenuItem(QMenu* menu, const char* title, QObject* receiver, const char* callback, const char* statustip, bool checked, const char* key_seq_str,QIcon icon){
    QAction *new_action = newAction(menu, title, statustip, icon);
    new_action->setShortcut(QString(key_seq_str));
    new_action->setCheckable(true);
    new_action->setChecked(checked);
    connect(new_action, SIGNAL(toggled(bool)), receiver, callback);
    return new_action;
}

void Graphical_UI::createLoadMenu() {
    QMenu* lm = menuBar()->addMenu(tr("&Load"));
    newMenuItem(lm,
                "&Open PCD files", 
                SLOT(openPCDFilesDialog()),
                "Open one or more pcd files to process",
                QKeySequence::Open, 
                QIcon::fromTheme("document-open"));
    
    newMenuItem(lm,
                "Open ROS &bag file",
                SLOT(openBagFileDialog()),
                "Open a bag file to process",
                "Ctrl+B",
                QIcon::fromTheme("document-open"));
}

QAction* Graphical_UI::createSaveMenu() {//octomap Menu
    QMenu* sm = menuBar()->addMenu(tr("&Save"));
    newMenuItem(sm,
                "&Save",                                
                SLOT(quickSaveAll()),
                "Save all stored point clouds with common coordinate frame to a pcd file",
                QKeySequence::Save,
                QIcon::fromTheme("document-save"));
    

    newMenuItem(sm,
                "Save &Feature Map...",
                SLOT(saveFeatures()),
                "Save all feature positions and descriptions in a common coordinate frame to a yaml or xml file",
                "Ctrl+F",
                QIcon::fromTheme("document-save"));

    QAction* oa = newMenuItem(sm, "Save Octomap...",
                              SLOT(saveOctomap()),
                              "Save computed OctoMap",
                              "",
                              QIcon::fromTheme("document-save-as"));

    newMenuItem(sm, "&Save as Point Cloud ...",
                SLOT(saveAll()),
                "Save all stored point clouds with common coordinate frame",
                QKeySequence::SaveAs,
                QIcon::fromTheme("document-save-as"));

    newMenuItem(sm, "&Save Point Cloud Node-Wise...", 
                SLOT(saveIndividual()),
                "Save stored point clouds in individual files", 
                "Ctrl+N", 
                QIcon::fromTheme("document-save-all"));

    newMenuItem(sm, "Save &G2O Graph...",
                SLOT(saveG2OGraphDialog()),
                "Save G2O graph (e.g. for use with the g2o viewer or external optimization)",
                "",
                QIcon::fromTheme("document-save"));
                              
    newMenuItem(sm, "Save Clouds to &Bag...",
                SLOT(saveBagDialog()),
                "Save clouds and transforms to bagfile",
                "Ctrl+Shift+B");
                
    newMenuItem(sm, "Save Trajectory &Estimate...",
                SLOT(saveTrajectoryDialog()),
                "Save trajectory estimate (and ground truth trajectory if available) for external evaluation.",
                "Ctrl+E");
                
    sm->addSeparator();
    newMenuItem(sm, "&Send Model",
                SLOT(sendAll()),
                "Send out all stored point clouds with corrected transform",
                "Ctrl+M",
                QIcon::fromTheme("document-send"));

    sm->addSeparator();

    newMenuItem(sm, "Save &3D as PDF File...",
                SLOT(saveVectorGraphic()),
                "Write 3D Scene to a PDF File. Warning: Meant for Pose Graphs not for the clouds or octomaps!",
                "",
                QIcon::fromTheme("application-pdf"));

    newMenuItem(sm, "Save Input &Images to Files...",
                SLOT(saveAllImages()),
                "Write All images shown in the gui to appropriate files",
                "",
                QIcon::fromTheme("image-x-generic"));
    
    newMenuItem(sm, "Capture Screencast...",
                this,
                SLOT(toggleScreencast(bool)),
                "Dump Screen as Video",
                !ParameterServer::instance()->get<std::string>("screencast_path_prefix").empty());
                
    return oa;
}
void Graphical_UI::createProcessingMenu() {
    QMenu* pm = menuBar()->addMenu(tr("&Processing"));
    newMenuItem(pm, "&Reset",
                SLOT(resetCmd()),
                "Reset the graph, clear all data collected",
                "Ctrl+R",
                QIcon::fromTheme("edit-delete"));

    newMenuItem(pm, "&Process",
                this,
                SLOT(pause(bool)),
                "Start/stop processing of frames",
                !ParameterServer::instance()->get<bool>("start_paused"),
                " ",
                QIcon::fromTheme("media-playback-start"));

    newMenuItem(pm, "Capture One& Frame",
                SLOT(getOneFrameCmd()),
                "Process one frame only",
                QKeySequence::InsertParagraphSeparator);
                
                              
    /* Crashes
    newMenuItem(pm, "&Delete Last Node",
                SLOT(deleteLastFrameCmd()),
                "Remove last node from graph",
                "Backspace",
                QIcon::fromTheme("edit-undo"));
                */
    pm->addSeparator();
                              
    newMenuItem(pm, "Clear Cloud Storage",
                SIGNAL(clearClouds()),
                "Remove Point Clouds from Memory",
                "",
                QIcon::fromTheme("edit-delete"));

    newMenuItem(pm, "Optimize Trajectory &Estimate",
                SLOT(optimizeGraphTrig()),
                "Compute optimized pose graph with g2o",
                "O");
                
    pm->addSeparator();

    newMenuItem(pm, "E&xit",
                SLOT(close()),
                "Exit the application",
                "Ctrl+Q",
                QIcon::fromTheme("application-exit"));//doesn't work for gnome
}

void Graphical_UI::createMenus() {


    createProcessingMenu();
    createLoadMenu();

    QAction* octoMapAction =  createSaveMenu();

    QMenu *om = menuBar()->addMenu(tr("&OctoMap"));
    newMenuItem(om, "Point Cloud Occupancy Filter",
                SLOT(triggerCloudFiltering()),
                "Remove points from the cloud that fall into unoccupied voxels of the OctoMap");
                
    newMenuItem(om, "Octomap Resolution",
                SLOT(setOctoMapResolution()),
                "Change the octomap resolution. Clears previously created maps on next update.");

    newMenuItem(om, "&Online OctoMapping",
                this,
                SLOT(toggleOnlineVoxelMapping(bool)),
                "Toggle Online/Offline OctoMapping. Make sure to set a low octomap_resolution and/or high cloud_creation_skip_step for online mapping",
                ParameterServer::instance()->get<bool>("octomap_online_creation"));




    /* Separation of computation and saving of octomap not yet Implemented
    QAction *computeOctoAct = new QAction(tr("Compute Octomap"), this);
    //computeOctoAct->setShortcuts(QKeySequence::SaveAs);
    computeOctoAct->setStatusTip(tr("Create OctoMap from stored point clouds"));
    connect(computeOctoAct, SIGNAL(triggered()), this, SLOT(computeOctomap()));
    octoMapMenu->addAction(computeOctoAct);
    this->addAction(computeOctoAct);
    */



    /*
    QAction *showErrorAct = new QAction(tr("Show Edge Errors"), this);
    showErrorAct->setShortcut(QString("Ctrl+Shift+E"));
    showErrorAct->setStatusTip(tr(""));
    connect(showErrorAct, SIGNAL(triggered()), this, SLOT(showEdgeErrors()));
    actionMenu->addAction(showErrorAct);
    this->addAction(showErrorAct);
    */


    //View Menus ###############################################################
    //View Menus ###############################################################

    if(ParameterServer::instance()->get<bool>("use_glwidget"))
    {
      QMenu* v3 = menuBar()->addMenu(tr("&3D View"));

      newMenuItem(v3, "&Clear 3D Display",
                  glviewer,
                  SLOT(reset()),
                  "Clears 3D viewer cloud data. Point Clouds are still retained, e.g. for mapping purposes.",
                  "",
                  QIcon::fromTheme("edit-delete"));
                                  

      newMenuItem(v3, "Toggle &3D Display",
                  glviewer,
                  SLOT(setVisible(bool)),
                  "Turn off the OpenGL Display",
                  true,
                  "3");

      newMenuItem(v3, "&Toggle Triangulation",
                  glviewer,
                  SLOT(toggleTriangulation()),
                  "Switch between surface, wireframe and point cloud",
                  "T");
                  
      newMenuItem(v3, "Follow &Camera",
                  glviewer,
                  SLOT(toggleFollowMode(bool)),
                  "Always use viewpoint of last frame (except zoom)",
                  true,
                  "Shift+F");

      newMenuItem(v3, "Show Grid",
                  glviewer,
                  SLOT(toggleShowGrid(bool)),
                  "Display XY plane grid",
                  false);
      newMenuItem(v3, "Show Pose TFs",
                  glviewer,
                  SLOT(toggleShowTFs(bool)),
                  "Display pose transformations at axes",
                  false);
      newMenuItem(v3, "Show Pose IDs",
                  glviewer,
                  SLOT(toggleShowIDs(bool)),
                  "Display pose ids at axes", 
                  false,
                  "I");

      newMenuItem(v3, "Show &Poses of Graph",
                  glviewer,
                  SLOT(toggleShowPoses(bool)),
                  "Display poses as axes",
                  ParameterServer::instance()->get<bool>("show_axis"),
                  "P");

      newMenuItem(v3, "Show &Edges of Graph",
                  glviewer,
                  SLOT(toggleShowEdges(bool)),
                  "Display edges of pose graph as lines",
                  ParameterServer::instance()->get<bool>("show_axis"),
                  "E");

      newMenuItem(v3, "Stere&o View",
                  glviewer,
                  SLOT(toggleStereo(bool)),
                  "Split screen view with slightly shifted Camera",
                  false);
      newMenuItem(v3, "Show &Feature Locations",
                  glviewer,
                  SLOT(toggleShowFeatures(bool)),
                  "Toggle whether feature locations should be rendered",
                  false);
      newMenuItem(v3, "Show &Octomap",
                  glviewer,
                  SLOT(toggleShowOctoMap(bool)),
                  "Toggle whether octomap should be displayed",
                  true,
                  "Ctrl+Shift+O");

      newMenuItem(v3, "Show &Clouds",
                  glviewer,
                  SLOT(toggleShowClouds(bool)),
                  "Toggle whether point clouds should be rendered",
                  true,
                  "C");

      newMenuItem(v3, "Toggle Background",
                  glviewer,
                  SLOT(toggleBackgroundColor(bool)),
                  "Toggle whether background should be white or black",
                  true,
                  "B");

      newMenuItem(v3, "Set Rotation &Grid",
                  this,
                  SLOT(setRotationGrid()),
                  "Discretize Rotation in Viewer",
                  "G");
                  
      newMenuItem(v3, "Set Stereo Offset",
                  this,
                  SLOT(setStereoShift()),
                  "Set the distance between the virtual cameras for stereo view",
                  "<");
                  
    }



    //2D View Menu ###############################################################
    QMenu *v2 = menuBar()->addMenu(tr("&2D View"));

    newMenuItem(v2, "Toggle &2D Stream",
                this,
                SLOT(set2DStream(bool)),
                "Turn the Image Stream on/off", 
                true,
                "2");

    newMenuItem(v2, "Show Visual Image",
                visual_image_label,
                SLOT(setVisible(bool)),
                "Show/Hide visual (color/monochrome) image.", 
                true);

    newMenuItem(v2, "Show Depth Image",
                depth_image_label,
                SLOT(setVisible(bool)),
                "Show/Hide depth image.",
                true);

    newMenuItem(v2, "Show Keypoint Image",
                feature_image_label,
                SLOT(setVisible(bool)),
                "Show/Hide image with keypoints.",
                false);

    newMenuItem(v2, "Show Visual Flow Image",
                feature_flow_image_label,
                SLOT(setVisible(bool)),
                "Show/Hide image with sparse feature flow (arrows).",
                true);

    //Settings Menu
    QMenu *st = menuBar()->addMenu(tr("&Settings"));

    newMenuItem(st, "&Reload Config",
                SLOT(reloadConfig()),
                "Reload Configuration from Parameter Server.",
                "",
                QIcon::fromTheme("reload"));

    newMenuItem(st, "&View Current Settings",
                SLOT(showOptions()),
                "Display the currently active options",
                "?");
                
    newMenuItem(st, "Set internal &Parameter",
                SLOT(setParam()),
                "Change a parameter (This will also change the value on the ROS Parameter server)");
                
    newMenuItem(st, "Set Ma&ximum Edge Error",
                SLOT(pruneEdgesWithHighError()),
                "Set the maximum allowed for edges (reoptimize to see the effect)",
                "Ctrl+X");
                
    //Help Menu
    QMenu *hm= menuBar()->addMenu(tr("&Help"));

    newMenuItem(hm, "&Usage Help",
                SLOT(help()),
                "Show usage information",
                QKeySequence::HelpContents,
                QIcon::fromTheme("help-contents"));

    newMenuItem(hm, "&About RGBDSLAM",
                SLOT(about()),
                "Show information about RGBDSLAM",
                "Ctrl+A",
                QIcon::fromTheme("help-about"));

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
  } else {
    statusBar()->showMessage(QString("Error: Set Value for non-existing progressbar ")+QString::number(id));
    showBusy(id, message, 0);
    setBusy(id, message, val);
  }
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
    depth_image_label->pixmap()->save(depth_file);
    feature_image_label->pixmap()->save(feature_file);
    feature_flow_image_label->pixmap()->save(flow_file);
    visual_image_label->pixmap()->save(visual_file);

    //glviewer->drawToPS(vector_file);
    QString message = tr("Saving all images.");
    statusBar()->showMessage(message);
}
