#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# Cython Extension Copyright (c) 2014, Felix Endres, ALU
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This scripts reads a bag file containing RGBD data, adds the corresponding
PointCloud2 messages, and saves it again into a bag file. Optional arguments
allow to select only a portion of the original bag file.
"""

import argparse
import sys
import os
import struct

cdef compute_rgb_cloud(double centerX, double centerY, double depthFocalLength, double depthscale, int height, int width, int downsamplingx, int downsamplingy, cv_rgb_image_color, cv_depth_image):
  cdef int u, v, size
  size = 0
  cdef double ptx, pty, ptz, d
  buffer = []

  for v in range(0,height,downsamplingx):
      for u in range(0,width,downsamplingy):
          d = cv_depth_image[v,u] 
          if d == d: #nan-test
            size +=1
            d *= depthscale
            rgb = cv_rgb_image_color[v,u]
            ptx = (u - centerX) * d / depthFocalLength;
            pty = (v - centerY) * d / depthFocalLength;
            ptz = d;
            buffer.append(struct.pack('ffffBBBBIII',
                ptx,pty,ptz,1.0,
                rgb[0],rgb[1],rgb[2],0,
                0,0,0))
  return ("".join(buffer), size)

cdef compute_depth_cloud(double centerX, double centerY, double depthFocalLength, double depthscale, int height, int width, int downsamplingx, int downsamplingy, cv_depth_image):
  cdef int u, v, size
  size = 0
  cdef double ptx, pty, ptz, d
  buffer = []

  for v in range(0,height,downsamplingx):
      for u in range(0,width,downsamplingy):
          d = cv_depth_image[v,u]
          if d == d: #nan-test
            size +=1
            d *= depthscale
            ptx = (u - centerX) * d / depthFocalLength;
            pty = (v - centerY) * d / depthFocalLength;
            ptz = d;
            buffer.append(struct.pack('ffff',ptx,pty,ptz,1.0))
  return ("".join(buffer), size)

def main():
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This scripts reads a bag file containing RGBD data, 
    adds the corresponding PointCloud2 messages, and saves it again into a bag file. 
    Optional arguments allow to select only a portion of the original bag file.  
    ''')
    parser.add_argument('--start', help='skip the first N seconds of  input bag file (default: 0.0)',default=0.00)
    parser.add_argument('--duration', help='only process N seconds of input bag file (default: off)')
    parser.add_argument('--nth', help='only process every N-th frame of input bag file (default: 15)',default=15)
    parser.add_argument('--skip', help='skip N blocks in the beginning (default: 1)', default=1)
    parser.add_argument('--downsamplex', help='use every nth column of rgb/d images (default: 1)', default=1)
    parser.add_argument('--downsampley', help='use every nth row of rgb/d images (default: 1)', default=1)
    parser.add_argument('--compress', help='compress output bag file', action='store_true')
    parser.add_argument('--depthscale', help='scale depth value', default=1.0)
    parser.add_argument('--depthcloud', help='generate non-colored cloud', action='store_true')
    parser.add_argument('--depthimage', help='store original depth image', action='store_true')
    parser.add_argument('--colorimage', help='store original color image', action='store_true')
    parser.add_argument('--nothingelse', help='only store the relevant information for rgbdslam in output bag', action='store_true')
    parser.add_argument('inputbag', help='input bag file')
    parser.add_argument('outputbag', nargs='?',help='output bag file')
    args = parser.parse_args()

    import roslib; roslib.load_manifest('rgbdslam')
    import rospy
    import rosbag
    import sensor_msgs.msg
    import cv
    from cv_bridge import CvBridge, CvBridgeError
    import tf
    
    if not args.outputbag:
        args.outputbag = os.path.splitext(args.inputbag)[0] + "-points.bag"
      
    print "Processing bag file:"
    print "  in:",args.inputbag
    print "  out:",args.outputbag
    print "  starting from: %s seconds"%(args.start)
        
    if args.duration:
        print "  duration: %s seconds"%(args.duration)
        
    print "  saving every %s-th frame"%(args.nth)
    args.skip = float(args.skip)
    print "  skipping %s blocks"%(args.skip)
    print "  scaling depth by %f"%(float(args.depthscale))

    inbag = rosbag.Bag(args.inputbag,'r')
    if args.compress:
        param_compression = rosbag.bag.Compression.BZ2
    else:
        param_compression = rosbag.bag.Compression.NONE
        
    outbag = rosbag.Bag(args.outputbag, 'w', compression=param_compression)
    
    depth_camera_info = None
    rgb_camera_info = None
    depth_image = None
    rgb_image_color = None
    cortex = None

    #nan = float('nan')
    bridge = CvBridge()
    frame = 0 
    transforms = dict()
    
    time_start = None
    for topic, msg, t in inbag.read_messages():
        if time_start==None:
            time_start=t
        if t - time_start < rospy.Duration.from_sec(float(args.start)):
            continue
        if args.duration and (t - time_start > rospy.Duration.from_sec(float(args.start) + float(args.duration))):
            break
        print "t=%f\r"%(t-time_start).to_sec(),
        if topic == "/tf":
            for transform in msg.transforms:
              if (transform.header.frame_id != "/world") and (transform.header.frame_id != "/kinect"):
                transforms[ (transform.header.frame_id,transform.child_frame_id) ] = transform
            # store messages
            msg = tf.msg.tfMessage()
            msg.transforms = list( transforms.itervalues() ) 
            outbag.write("/tf",msg,t)
            transforms = dict()
            continue
        if topic == "/imu":
            imu = msg
            continue
        if topic == "/camera/depth/camera_info":
            depth_camera_info = msg
            continue
        if topic == "/camera/rgb/camera_info":
            rgb_camera_info = msg
            continue
        if topic == "/camera/rgb/image_color" and rgb_camera_info:
            rgb_image_color = msg
            continue
        if topic == "/camera/depth/image" and depth_camera_info and rgb_image_color and rgb_camera_info:# and imu:
            depth_image = msg
            # now process frame
            
            if depth_image.header.stamp - rgb_image_color.header.stamp > rospy.Duration.from_sec(1/30.0):
                continue
            
            frame += 1
            if frame % float(args.nth) ==0:
                if args.skip > 0:
                    args.skip -= 1
                else:
#                    outbag.write("/imu",imu,t)
                    if args.depthimage:
                      outbag.write("/camera/depth/camera_info",depth_camera_info,t)
                      outbag.write("/camera/depth/image",depth_image,t)
                    if args.colorimage:
                      outbag.write("/camera/rgb/camera_info",rgb_camera_info,t)
                      outbag.write("/camera/rgb/image_color",rgb_image_color,t)

                    # generate monochrome image from color image
                    cv_rgb_image_mono = bridge.imgmsg_to_cv(rgb_image_color, "mono8")
                    rgb_image_mono = bridge.cv_to_imgmsg(cv_rgb_image_mono)
                    rgb_image_mono.header = rgb_image_color.header
                    if args.colorimage:
                      outbag.write("/camera/rgb/image_mono",rgb_image_mono,t)
    
                    # generate depth and colored point cloud
                    cv_depth_image = bridge.imgmsg_to_cv(depth_image, "passthrough")
                    cv_rgb_image_color = bridge.imgmsg_to_cv(rgb_image_color, "bgr8")
    #               
                    if args.depthcloud:
                        centerX = depth_camera_info.K[2]
                        centerY = depth_camera_info.K[5]
                        depthFocalLength = depth_camera_info.K[0]
                        depth_points = sensor_msgs.msg.PointCloud2()
                        depth_points.header = depth_image.header
                        depth_points.width = depth_image.width
                        depth_points.height  = depth_image.height
                        depth_points.fields.append(sensor_msgs.msg.PointField(
                            name = "x",offset = 0,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                        depth_points.fields.append(sensor_msgs.msg.PointField(
                            name = "y",offset = 4,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                        depth_points.fields.append(sensor_msgs.msg.PointField(
                            name = "z",offset = 8,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                        depth_points.point_step = 16 
                        depth_points.row_step = depth_points.point_step * depth_points.width

                        depth_points.data, size = compute_depth_cloud(centerX, centerY, depthFocalLength, float(args.depthscale), depth_image.height, depth_image.width, int(args.downsamplex), int(args.downsampley), cv_depth_image)
                        depth_points.width = size
                        depth_points.height  = 1
                        outbag.write("/camera/depth/points", depth_points, t)
                    
                    centerX = depth_camera_info.K[2]
                    centerY = depth_camera_info.K[5]
                    depthFocalLength = depth_camera_info.K[0]
                    rgb_points = sensor_msgs.msg.PointCloud2()
                    rgb_points.header = rgb_image_color.header
                    rgb_points.fields.append(sensor_msgs.msg.PointField(
                        name = "x",offset = 0,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                    rgb_points.fields.append(sensor_msgs.msg.PointField(
                        name = "y",offset = 4,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                    rgb_points.fields.append(sensor_msgs.msg.PointField(
                        name = "z",offset = 8,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                    rgb_points.fields.append(sensor_msgs.msg.PointField(
                        name = "rgb",offset = 16,datatype = sensor_msgs.msg.PointField.FLOAT32,count = 1 ))
                    rgb_points.point_step = 32 
                    rgb_points.row_step = rgb_points.point_step * rgb_points.width

                    rgb_points.data, size = compute_rgb_cloud(centerX, centerY, depthFocalLength, float(args.depthscale), depth_image.height, depth_image.width, int(args.downsamplex), int(args.downsampley), cv_rgb_image_color, cv_depth_image)
                    rgb_points.width = size
                    rgb_points.height  = 1
                    outbag.write("/camera/rgb/points", rgb_points, t)                
            # consume the images
#            imu = None
            depth_image = None
            rgb_image_color = None
            continue
        if topic not in ["/tf","/imu",
                         "/camera/depth/camera_info","/camera/rgb/camera_info",
                         "/camera/rgb/image_color","/camera/depth/image"]:
          if not args.nothingelse:
            # anything else: pass thru
            outbag.write(topic,msg,t)
                
    outbag.close()
    print

if __name__ == '__main__':
  main()
