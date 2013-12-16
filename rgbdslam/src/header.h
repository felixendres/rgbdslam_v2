#include <std_msgs/Header.h>
#include <pcl/PCLHeader.h>
#ifndef RGBDSLAM_MYHEADER
#define RGBDSLAM_MYHEADER

///A class to facilitate parallel use of pcl and ros header
class myHeader {
  public:
    myHeader();///<Allows same usage as ros/pcl header
    myHeader(pcl::PCLHeader h); ///<Allow implicit conversion from pcl header
    myHeader(std_msgs::Header rh);///<Allow implicit conversion from ros header
    myHeader(uint32_t aseq, ros::Time astamp, std::string aframe_id); ///<convenience 

    std_msgs::Header toRosHeader();
    void fromRosHeader(std_msgs::Header rh);

    operator pcl::PCLHeader(); ///<Allow implicit conversion to pcl header
    operator std_msgs::Header(); ///<Allow implicit conversion to ros header
    
    uint32_t seq;///<Allows direct access as for pcl/ros header
    ros::Time stamp; ///<Allows direct access as for pcl/ros header
    std::string frame_id;///<Allows direct access as for pcl/ros header
};
#endif
