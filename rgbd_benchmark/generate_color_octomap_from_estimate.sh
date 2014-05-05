#!/bin/bash

if  test ! -d "$1" || test ! -f "$2"; then  
  echo "'$1' is not a directory or $1 does not exist" 
  echo "Usage: $0 <directory-where-tgz-was-extracted-to> <trajectory-file>"
  echo "e.g., $0 some/path/to/rgbd_dataset_freiburg1_360/ other/path/to/SURF/rgbd_dataset_freiburg1_360/rgbd_dataset_freiburg1_360.bagafter1_optimization_estimate.txt"
  exit
fi
DIR=`readlink -f $1`
FILE=`readlink -f $2`
pushd $DIR > /dev/null

if rosrun rgbd_benchmark_tools generate_registered_pointcloud.py --pcd_format rgb.txt depth.txt $FILE --downsample 1 --nth 1 registered_cloud; then
  export ROS_MASTER_URI=http://localhost:11314
  roscore -p11314 &
  sleep 3
  rosparam set /color_octomap_server1/data_directory $DIR
  roslaunch octomap_server color_octomap_mapping_from_pcd.launch  
  #octovis $DIR/color_server_map*.cot
  kill -TERM $! #Kill Roscore
fi

popd > /dev/null

