#include "header.h"
#include <pcl_conversions/pcl_conversions.h>

myHeader::myHeader()
{
  seq = 0;
}

myHeader::myHeader(pcl::PCLHeader h) {
  std_msgs::Header rh = pcl_conversions::fromPCL(h);
  fromRosHeader(rh);
}

myHeader::myHeader(std_msgs::Header rh){
  fromRosHeader(rh);
}

myHeader::myHeader(uint32_t aseq, ros::Time astamp, std::string aframe_id)
{
  seq = aseq;
  stamp = astamp;
  frame_id = aframe_id;
}

std_msgs::Header myHeader::toRosHeader()
{
  std_msgs::Header rh;
  rh.seq = seq;
  rh.stamp = stamp;
  rh.frame_id = frame_id;
  return rh;
}


void myHeader::fromRosHeader(std_msgs::Header rh){
  seq = rh.seq;
  stamp = rh.stamp;
  frame_id = rh.frame_id;
}


myHeader::operator pcl::PCLHeader()
{
  return pcl_conversions::toPCL(this->toRosHeader());
}
myHeader::operator std_msgs::Header()
{
  return this->toRosHeader();
}
