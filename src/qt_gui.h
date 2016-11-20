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
*
* there is an alternative: RosUI for headless use of the rgbdslam
*/
#ifndef QTCV_H
#define QTCV_H

#include <QMainWindow>
#include <QGridLayout>
#include "parameter_server.h"
#include <QMatrix4x4>
#include <QMap>

class QAction;
class QLabel;
class QMenu;
class GLViewer;
class QSplitter;
class QProgressBar;

//TODO:
//Choice between Binary and ASCII outputfiles
//Buttons for start/stop
//GUI/Commandline options for switching on/off the individual visualizations

//!Constructs a QT GUI for easy control of RGBD-SLAM
/** Small GUI Class to visualize and control rgbdslam
* See Help->About for a short description */
class Graphical_UI: public QMainWindow
{
    Q_OBJECT

public:
    Graphical_UI();
    Graphical_UI(QString title);
    GLViewer* getGLViewer();
Q_SIGNALS:
    ///User selected to reset the graph
    void reset(); 
    ///User selected to start or resume processing
    void togglePause();
    ///User wants the next frame to be processed
    void getOneFrame();
    ///User wants the last node to be removed from the graph
    void deleteLastFrame();
    void sendAllClouds(); ///< Signifies the sending of the whole model
    void saveBagfile(QString filename); ///< Signifies the sending of the whole model
    ///User wants the current world model to be saved to a pcd-file or ply file
    void saveAllClouds(QString filename);
    void openPCDFiles(QStringList filenamelist);
    void openBagFile(QString filename);
    ///User wants the g2o graph saved 
    void saveG2OGraph(QString filename);
    void saveAllFeatures(QString filename);
    void saveTrajectory(QString filename);
    void saveOctomapSig(QString filename);
    void computeOctomapSig(QString filename);
    ///User wants the current world model to be saved to one pcd-file per node
    void saveIndividualClouds(QString file_basename);
    void evaluation();
    void optimizeGraph();
    void occupancyFilterClouds();
    void printEdgeErrors(QString);
    void pruneEdgesWithErrorAbove(float);
    void toggleMapping(bool);
    void clearClouds();
     
public Q_SLOTS:
    void setVisualImage(QImage);
    void setFeatureFlowImage(QImage);
    void setFeatureImage(QImage);
    void setDepthImage(QImage);
    void sendFinished(); ///< Call to display, that sending finished
    void showOptions();
    void showBusy(int id, const char* message, int max);
    void setBusy(int id, const char* message, int val);
    void set2DStream(bool is_on);
    //save depth color feature and correspondences image
    void saveAllImages();

private Q_SLOTS:
    void saveVectorGraphic();
    void resetCmd();
    void reloadConfig();
    void sendAll();
    void setParam();///< Show a combobox to select a parameter, then call this->setParam(QString)
    void setStereoShift();
    void setRotationGrid();
    void saveAll();
    void saveOctomap();
    void computeOctomap();
    void saveIndividual();
    void quickSaveAll();
    void saveFeatures();
    void pause(bool);
    void about();
    void help();
    void setInfo(QString);
    void setInfo2(QString);
    void setStatus(QString);
    void getOneFrameCmd();
    void deleteLastFrameCmd();
//    void set3DDisplay(bool is_on);
    void saveTrajectoryDialog();
    void openBagFileDialog();
    void saveBagDialog();
    void openPCDFilesDialog();
    void saveG2OGraphDialog();
    void optimizeGraphTrig();
    void showEdgeErrors();
    void pruneEdgesWithHighError();
    void toggleFullscreen(bool);
    void toggleCloudStorage(bool);
    void toggleOnlineVoxelMapping(bool);
    void toggleLandmarkOptimization(bool);
    void toggleMappingPriv(bool);
    void toggleScreencast(bool);
    //Display a Dialog to change the value of the mentioned parameter
    void setParam(QString param_name);
    void setOctoMapResolution();
    void triggerCloudFiltering();
private:
    //Helper to add a new checkable item to a menu. Assumes callback is method of this class
    QAction* newMenuItem(QMenu* menu, const char* title, QObject* receiver, const char* callback, const char* statustip, bool checked, const char* key_seq = "" ,QIcon icon = QIcon());
    //Helper to add a new item to a menu. Assumes callback is method of this class
    QAction* newMenuItem(QMenu* menu, const char* title, const char* callback_method_of_this_class, const char* statustip,               const char* key_seq = "", QIcon icon = QIcon());
    //Helper to add a new item to a menu. Assumes callback is method of this class
    QAction* newMenuItem(QMenu* menu, const char* title, const char* callback_method_of_this_class, const char* statustip, QKeySequence::StandardKey key_seq,      QIcon icon = QIcon());
    //Helper to add a new item to a menu. 
    QAction* newMenuItem(QMenu* menu, const char* title, QObject* receiver, const char* callback,   const char* statustip,               const char* key_seq = "", QIcon icon = QIcon());
    //Helper for newMenuItem
    QAction* newAction(QMenu* menu, const char* title, const char* statustip, QIcon icon);
    void setLabelToImage(QLabel* which_label, QImage new_image);
    void setup();
    void setupStatusbar();
    void initTexts();
    //!Menus and Menu elements are defined here
    void createMenus();
    //create "save" menu, return saveOctomap to also put in octoMapMenu
    QAction* createSaveMenu();
    void createProcessingMenu();
    void createLoadMenu();
    //QString *menuHelpText;
    QString *mouseHelpText;
    QString *infoText;
    QString *licenseText;
    QSplitter* vsplitter;
    QLabel *infoLabel;
    QLabel *infoLabel2;
    QLabel *tmpLabel;
    QLabel *visual_image_label;
    QLabel *feature_flow_image_label;
    QLabel *depth_image_label;
    QLabel *feature_image_label;
    QLabel *stats_image_label;
    //QLabel *transform_label;
    QGridLayout* gridlayout;
    QString filename;
    GLViewer* glviewer;
    bool pause_on;
    QMap<int, QProgressBar*> progressbars;
};

#endif
