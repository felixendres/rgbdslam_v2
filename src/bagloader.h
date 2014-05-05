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


#ifndef OPENNI_LISTENER_H
#define OPENNI_LISTENER_H
#include "ros/ros.h"
#include "parameter_server.h"
#include <tf/transform_broadcaster.h>
//#include <pcl_tf/transforms.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <qtconcurrentrun.h>
#include <QImage> //for cvMat2QImage not listet here but defined in cpp file
#include <QStringList> 
#include <QMatrix4x4> 
#include <rosbag/bag.h>

//forward-declare to avoid including tf
///\cond
namespace tf{
  class TransformListener;
}
///\endcond


//!Handles most of the ROS-based communication

/** The purpose of this class is to listen to 
 * synchronized image pairs from the kinect
 * convert them to opencv, process them, convert
 * them to a qt image and send them to the mainwindow
 * defined in qtcv.h/.cpp
 */
class BagLoader : public QObject {
  ///QT Stuff, to communicate with the gui
  Q_OBJECT
  Q_SIGNALS:
    void setPointCloud(pointcloud_type * pc, QMatrix4x4 transformation);
    ///Set the info label on the right side in the statusbar of the GUI
    void setGUIInfo(QString message);
    void setGUIInfo2(QString message);
    ///Set the temporary status-message in the GUI
    void setGUIStatus(QString message);
    void bagFinished();

  public Q_SLOTS:
    ///Switch between processing or ignoring new incoming data
    void togglePause();
    void loadBagFileAsync(QString);
    void loadBagFileAsync(std::string);
    void clearPointCloud(pointcloud_type*);

  public:
    //!Ctor: setup synced listening to ros topics (kinect/stereo data) and prepare the feature handling
    /*!Constructor: The listener needs to know the node handle and the GraphManager instance.
     * Which topics to listen to and which feature detector/extractor to use is queried from the parameter 
     * server.
     */
    BagLoader();

    //!Delete tflistener, shutdown ros publishers
    ~BagLoader();
    void callback(const sensor_msgs::PointCloud2ConstPtr& point_cloud);

  protected:
    void loadBag(std::string filename);
    //!Retrieve the transform between the lens and the base-link at capturing time
    void sendWithTransformation(pointcloud_type* cloud);

    rosbag::Bag bag;
    
    //ros::Publisher pc_pub; 
    /*unsigned int callback_counter_;*/
    bool pause_;
    tf::TransformListener* tflistener_; //!this being a pointer saves the include (using the above forward declaration)
    ros::Publisher tf_pub_;
    int data_id_;
};

#endif
