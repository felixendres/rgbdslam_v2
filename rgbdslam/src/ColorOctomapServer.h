#ifndef COLOR_OCTOMAP_SERVER_RGBDSLAM
#define COLOR_OCTOMAP_SERVER_RGBDSLAM

#include <octomap_server/OctomapServer.h>
#include <octomap/ColorOcTree.h>
#include <octomap_ros/OctomapROS.h>
#include "parameter_server.h"
#include <qtconcurrentrun.h>


  class ColorOctomapServer: public octomap_server::OctomapServer {
  public:
    ColorOctomapServer();
    virtual ~ColorOctomapServer();
    void reset();
    bool save(const char* filename) const;
    virtual void insertCloudCallback(const pointcloud_type::ConstPtr cloud, double max_range = -1.0);
    virtual void insertCloudCallbackCommon(const pointcloud_type::ConstPtr cloud,
                                           const tf::Transform& trans, double max_range);

  protected:
    octomap::OctomapROS<octomap::ColorOcTree> m_octoMap;
    mutable QFuture<void> rendering;  //Mutable is a hack, otherwise waitforfinished cannot be called in const function
  };

#endif

