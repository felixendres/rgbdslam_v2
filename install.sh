#This script installs rgbdslam_v2 and g2o in ~/Code
#This script can be downloaded individually, as it 
#does not require rest of the rgbdslam repository.
#It will download all required code in the process.

#If you have a fast machine with 4GB RAM or more, increase the 
#two occurences of "-j2" below to parallelize the compilation more

#Prepare System
if test ! -d /opt/ros/kinetic; then
  echo This script assumes ROS kinetic to be installed
  echo The directory /opt/ros/kinetic was not found
  exit 1
fi

SUBDIR=~/Code
echo "This script puts all code into '$SUBDIR'. Edit this script to change the location."
echo "Press enter to continue, Ctrl-C to cancel"
read
mkdir -p $SUBDIR

source /opt/ros/kinetic/setup.bash

echo
echo "Removing packages known to conflict (password required for apt-get)"
echo
sudo apt-get purge ros-kinetic-libg2o libqglviewer-dev

echo
echo "Updating ROS dependency database"
echo
rosdep update

echo "Install dependences for g2o"
sudo apt-get install libsuitesparse-dev libeigen3-dev
echo

echo
echo "Downloading, building and installing g2o"
echo
G2O_REPO_DIR=$SUBDIR/g2ofork
git clone -b c++03 https://github.com/felixendres/g2o.git $G2O_REPO_DIR
mkdir $G2O_REPO_DIR/build
cd $G2O_REPO_DIR/build
cmake .. -DCMAKE_INSTALL_PREFIX=$G2O_REPO_DIR/install -DG2O_BUILD_EXAMPLES=OFF
nice make -j2 install

echo
echo "Preparing catkin workspace for rgbdslam_v2"
echo
WORKSPACE=$SUBDIR/rgbdslam_catkin_ws
mkdir -p $WORKSPACE/src
cd $WORKSPACE/src
catkin_init_workspace
catkin_make -C $WORKSPACE
source $WORKSPACE/devel/setup.bash

echo
echo "Downloading rgbdslam_v2"
echo
#Get and build rgbdslam_v2
export G2O_DIR=$G2O_REPO_DIR/install
git clone -b kinetic https://github.com/felixendres/rgbdslam_v2.git $WORKSPACE/src/rgbdslam

#Install missing dependencies
rosdep install rgbdslam
echo
echo "Building rgbdslam_v2"
echo
nice catkin_make -C $WORKSPACE -j2

