#!/bin/bash
FILE=`readlink -f $0`
DIR=`dirname $FILE`
if pushd $DIR; then
  #svn export https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools 
  #read -p "Press Enter to Start Downloading of (huge amounts of) Benchmark Data."
  wget -c -P benchmark_data -i download_benchmark_data.urls
fi
popd > /dev/null
