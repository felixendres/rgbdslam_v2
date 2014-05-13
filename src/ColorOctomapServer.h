#ifndef COLOR_OCTOMAP_SERVER_RGBDSLAM
#define COLOR_OCTOMAP_SERVER_RGBDSLAM

#include "parameter_server.h"
//#include <octomap/ColorVoxelMap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <qtconcurrentrun.h>
#include <memory>
#include <boost/shared_ptr.hpp>
#include "renderable.h"

  class ColorOctomapServer : public Renderable {
  public:
    ColorOctomapServer();
    virtual ~ColorOctomapServer();
    void reset();
    bool save(const char* filename) const;
    ///Raycas cloud into the octomap
    /// @param cloud pointcloud in arbitrary frame (specified in the clouds header)
    virtual void insertCloudCallback(const pointcloud_type::ConstPtr cloud, double max_range = -1.0);

    ///Raycast cloud into the octomap
    /// @param cloud pointcloud in map frame
    /// @param origin sensor location in map frame
    virtual void insertCloudCallbackCommon(boost::shared_ptr<octomap::Pointcloud> cloud,
                                           pointcloud_type::ConstPtr colors,
                                           const octomap::point3d& origin, double max_range = -1.0);

    ///Filter cloud by occupancy of voxels, e.g. remove points in free space
    void occupancyFilter(pointcloud_type::ConstPtr input, 
                         pointcloud_type::Ptr output, 
                         double occupancy_threshold);

    virtual void render();
  protected:
    octomap::ColorOcTree m_octoMap;
    //octomap::OctomapROS<octomap::ColorOcTree> m_octoMap;
    mutable QFuture<void> rendering;  //Mutable is a hack, otherwise waitforfinished cannot be called in const function
  };

#endif

