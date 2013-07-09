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
//#include <pcl_tf/transforms.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "graph_manager.h"
#include <qtconcurrentrun.h>
#include <QImage> //for cvMat2QImage not listet here but defined in cpp file
#include <rosbag/bag.h>

//forward-declare to avoid including tf
///\cond
namespace tf{
  class TransformListener;
}
///\endcond

//The policy merges kinect messages with approximately equal timestamp into one callback 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                        sensor_msgs::Image, 
                                                        sensor_msgs::PointCloud2> KinectSyncPolicy;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                        sensor_msgs::PointCloud2> StereoSyncPolicy;

//The policy merges kinect messages with approximately equal timestamp into one callback 
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, 
                                                        sensor_msgs::Image,
                                                        sensor_msgs::CameraInfo> NoCloudSyncPolicy;


/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function 
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg) { 
    this->signalMessage(msg); //"this->" is required as of ros groovy
  }
};

//!Handles most of the ROS-based communication

/** The purpose of this class is to listen to 
 * synchronized image pairs from the kinect
 * convert them to opencv, process them, convert
 * them to a qt image and send them to the mainwindow
 * defined in qtcv.h/.cpp
 */
class OpenNIListener : public QObject {
  ///QT Stuff, to communicate with the gui
  Q_OBJECT
  Q_SIGNALS:
    ///Connect to this signal to get up-to-date optical images from the listener
    void newVisualImage(QImage);
    ///Connect to this signal to get up-to-date featureFlow visualizations from the listener
    void newFeatureFlowImage(QImage);
    ///Connect to this signal to get up-to-date depth images from the listener
    void newDepthImage(QImage);
    ///Connect to this signal to get the transformation matrix from the last frame as QString
    void newTransformationMatrix(QString);
    //void pauseStatus(bool is_paused);
    ///Set the info label on the right side in the statusbar of the GUI
    void setGUIInfo(QString message);
    ///Set the temporary status-message in the GUI
    void setGUIStatus(QString message);
    void bagFinished();

  public Q_SLOTS:
    ///Switch between processing or ignoring new incoming data
    void togglePause();
    void toggleBagRecording();
    ///Process a single incomming frame. Useful in pause-mode for getting one snapshot at a time
    void getOneFrame();

  public:
    //!Ctor: setup synced listening to ros topics (kinect/stereo data) and prepare the feature handling
    /*!Constructor: The listener needs to know the node handle and the GraphManager instance.
     * Which topics to listen to and which feature detector/extractor to use is queried from the parameter 
     * server.
     */
    OpenNIListener(GraphManager* g_mgr);

    //!Delete tflistener, shutdown ros publishers
    ~OpenNIListener();
    //! Listen to kinect data, construct nodes and feed the graph manager with it.
    /*! For each dataset from the kinect, do some data conversion,
     *  construct a node, hand it to the graph manager and
     *  do some visualization of the result in the GUI and RVIZ.
     */
    void kinectCallback (const sensor_msgs::ImageConstPtr& visual_img,
                         const sensor_msgs::ImageConstPtr& depth_img, 
                         const sensor_msgs::PointCloud2ConstPtr& point_cloud);
    //! For this callback the point cloud is not required. 
    void noCloudCallback (const sensor_msgs::ImageConstPtr& visual_img_msg,
                          const sensor_msgs::ImageConstPtr& depth_img_msg,
                          const sensor_msgs::CameraInfoConstPtr& cam_info_msg) ;
    //! No depth image but pointcloud, e.g., for stereo cameras
    void stereoCallback(const sensor_msgs::ImageConstPtr& visual_img_msg, const sensor_msgs::PointCloud2ConstPtr& point_cloud);

    void loadBag(const std::string &filename);
  protected:
    //! Create a QImage from one image. 
    ///The QImage stores its data in the rgba_buffers_ indexed by idx (reused/overwritten each call)
    QImage cvMat2QImage(const cv::Mat& image, unsigned int idx); 
    //! Create a QImage from several one-channel images.
    ///The QImage stores its data in the rgba_buffers_ indexed by idx (reused/overwritten each call)
    QImage cvMat2QImage(const cv::Mat& channel1, const cv::Mat& channel2, const cv::Mat& channel3, unsigned int idx);
    //!Retrieve the transform between the lens and the base-link at capturing time
    void retrieveTransformations(std_msgs::Header depth_header, Node* node_ptr);

    //!Call processNode either regularly or as background thread
    void callProcessing(cv::Mat gray_img, Node* node_ptr);
    //!processNode is called by cameraCallback in a separate thread and after finishing visualizes the results
    void processNode(Node* new_node);

    //! common processing 
    void cameraCallback(cv::Mat visual_img, 
                        pointcloud_type::Ptr point_cloud, 
                        cv::Mat depth_mono8_img);
    //! as cameraCallback, but create Node without cloud
    void noCloudCameraCallback(cv::Mat visual_img, 
                               cv::Mat depth, 
                               cv::Mat depth_mono8_img,
                               std_msgs::Header depth_header,
                               const sensor_msgs::CameraInfoConstPtr& cam_info);
    ///The GraphManager uses the Node objects to do the actual SLAM
    ///Public, s.t. the qt signals can be connected to by the holder of the OpenNIListener
    GraphManager* graph_mgr_;
    
    //Variables
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> extractor_;

    message_filters::Synchronizer<StereoSyncPolicy>* stereo_sync_;
    message_filters::Synchronizer<KinectSyncPolicy>* kinect_sync_;
    message_filters::Synchronizer<NoCloudSyncPolicy>* no_cloud_sync_;
    message_filters::Subscriber<sensor_msgs::Image> *visua_sub_;      
    message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;      
    message_filters::Subscriber<sensor_msgs::CameraInfo> *cinfo_sub_;      
    message_filters::Subscriber<sensor_msgs::PointCloud2> *cloud_sub_;

    BagSubscriber<sensor_msgs::Image>* rgb_img_sub_;
    BagSubscriber<sensor_msgs::Image>* depth_img_sub_;
    BagSubscriber<sensor_msgs::CameraInfo>* cam_info_sub_;

    cv::Mat depth_mono8_img_;
    ///The depth mono img is stored here for visualization purposes
    cv::Mat visualization_depth_mono8_img_;
    ///The visual img is stored here for visualization purposes here
    cv::Mat visualization_img_;
    std::vector<cv::Mat> rgba_buffers_;
    
    rosbag::Bag bag;
    bool save_bag_file;
    
    //ros::Publisher pc_pub; 
    /*unsigned int callback_counter_;*/
    bool pause_;
    bool getOneFrame_;
    bool first_frame_;
    QFuture<void> future_;
    QMutex bagfile_mutex;
    tf::TransformListener* tflistener_; //!this being a pointer saves the include (using the above forward declaration)
    tf::TransformBroadcaster tf_br_;
    ros::Publisher tf_pub_;
    int data_id_;
    std::string image_encoding_;
};

#endif
