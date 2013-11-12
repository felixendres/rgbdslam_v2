#########################################################
### CONFIG ##############################################
#########################################################
# 1, if SiftGPU should be used. For additional information
# see README
set(USE_SIFT_GPU        1)
set(ENV{SIFT_GPU_MODE}		2) #CUDA = 1, GLSL = 2
set(ENV{SIFT_GPU_CUDA_PATH}	/usr/local/cuda)	
	
set(USE_GICP_BIN		0)
set(USE_GICP_CODE		0)
set(USE_PCL_ICP		  1)
set(USE_GL2PS       0) #For printing the 3D Window to PostScript - requires installation of additional library: libgl2ps-dev
IF ("${ROS_PARALLEL_JOBS}" EQUAL "")
  set(ROS_PARALLEL_JOBS 1)
endif ("${ROS_PARALLEL_JOBS}" EQUAL "")
#########################################################
#########################################################
#########################################################

set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} -fopenmp)

cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)
#add_definitions(-DROSCONSOLE_SEVERITY_INFO)
IF (${USE_GL2PS})
  add_definitions(-DGL2PS)
ENDIF (${USE_GL2PS})
set(CMAKE_CXX_FLAGS "-O3")
#is used for compiling the libraries 
set(USE_GICP 0)
IF (${USE_GICP_BIN} OR ${USE_GICP_CODE})
	set(USE_GICP 1)
ENDIF (${USE_GICP_BIN} OR ${USE_GICP_CODE})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library) 
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)


##############################################################################
# Required 3rd party
##############################################################################
#############################
# Octomap ###################
#############################
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

#############################
# Qt ########################
#############################
# http://qtnode.net/wiki/Qt4_with_cmake
find_package(Qt4 REQUIRED)
SET(QT_USE_QTXML TRUE)
SET(QT_USE_QTOPENGL TRUE)
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})

# This is necessary as all ui files etc will get dumped in the bottom of then binary directory. 
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${QT_QTOPENGL_INCLUDE_DIR} external/gicp/ann_1.1.2/include/ANN)

#get_directory_property(clean ADDITIONAL_MAKE_CLEAN_FILES)
#SET(clean external/siftgpu/linux/bin/libsiftgpu.so)
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES "${clean}")
 	
LINK_DIRECTORIES(${LIBRARY_OUTPUT_PATH})
SET(${CMAKE_LIBRARY_PATH} ${LIBRARY_OUTPUT_PATH})
#SET(CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE TRUE)

find_package(libg2o REQUIRED)
ADD_DEFINITIONS(-DDO_FEATURE_OPTIMIZATION)


#Only possible when not subscribing to point cloud topic. Possibly dangerous anyway, but halves storage space per point cloud
#ADD_DEFINITIONS(-DRGB_IS_4TH_DIM) 
#ADD_DEFINITIONS(-DDO_FEATURE_OPTIMIZATION)
ADD_DEFINITIONS(-DHEMACLOUDS) 


#############################
# SiftGPU ###################
#############################
IF (${USE_SIFT_GPU})
	SET(sift_gpu_build_path ${CMAKE_SOURCE_DIR}/external/siftgpu/linux)

	include_directories(${CMAKE_CURRENT_BINARY_DIR} external/siftgpu/src)
	
	MESSAGE("\n------------------------------------------------------------------\n")
	MESSAGE("Compiling SiftGPU...")
	EXECUTE_PROCESS(
		COMMAND make ${ROS_PARALLEL_JOBS} siftgpu 
		WORKING_DIRECTORY ${sift_gpu_build_path}
		RESULT_VARIABLE sift_gpu_return
	)
	MESSAGE("\n------------------------------------------------------------------\n")
	
	#on error
	IF (NOT ${sift_gpu_return} EQUAL 0)
		MESSAGE(FATAL_ERROR "SiftGPU cannot be compiled. Returned: ${sift_gpu_return}")
	ENDIF (NOT ${sift_gpu_return} EQUAL 0)
	
	FILE(COPY external/siftgpu/linux/bin/libsiftgpu.so DESTINATION ../lib/)
	
	add_definitions(-DUSE_SIFT_GPU)
	add_definitions(-DSIFT_GPU_MODE=$ENV{SIFT_GPU_MODE})
ENDIF (${USE_SIFT_GPU})



#############################
# GICP ######################
#############################
IF (${USE_GICP})
	SET(gicp_build_path ${CMAKE_SOURCE_DIR}/external/gicp/ann_1.1.2)
  FILE(MAKE_DIRECTORY ${gicp_build_path}/lib/)
  MESSAGE("\n------------------------------------------------------------------\n")
	MESSAGE("Compiling GICP...")
	EXECUTE_PROCESS(
	    COMMAND make ${ROS_PARALLEL_JOBS} linux-g++
	    WORKING_DIRECTORY ${gicp_build_path}
	    RESULT_VARIABLE gicp_return
    )
    MESSAGE("\n------------------------------------------------------------------\n")
	
	#on error
	IF (NOT ${gicp_return} EQUAL 0)
		MESSAGE(FATAL_ERROR "GICP cannot be compiled. Returned: ${gicp_return}")
	ENDIF (NOT ${gicp_return} EQUAL 0)
	
	FILE(COPY ${gicp_build_path}/lib/libANN.a DESTINATION ${CMAKE_SOURCE_DIR}/lib/)
ENDIF (${USE_GICP})

IF (${USE_GICP_BIN})
	add_definitions(-DUSE_ICP_BIN)
ENDIF (${USE_GICP_BIN})

IF (${USE_GICP_CODE})
	add_definitions(-DUSE_ICP_CODE) 
ENDIF (${USE_GICP_CODE})
IF (${USE_PCL_ICP})
  add_definitions(-DUSE_PCL_ICP) 
ENDIF (${USE_PCL_ICP})



#############################
# OpenCV ####################
#############################
find_package(OpenCV)
#include_directories(${OpenCV_INCLUDE_DIRS} external/ /usr/include/suitesparse/)
include_directories(${OpenCV_INCLUDE_DIRS} external/ /usr/include/suitesparse/)

#############################
# Eigen ####################
#############################
find_package(Eigen)
IF(Eigen_FOUND)
  include_directories(${EIGEN_INCLUDE_DIRS})
  add_definitions(${EIGEN_DEFINITIONS})
ELSE(Eigen_FOUND)
  MESSAGE("Eigen package was not found. This is OK FOR ROS ELECTRIC, bad for fuerte\n")
ENDIF(Eigen_FOUND)

##############################################################################
# Some QT Magic (MOC Creation)
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/local/*.hpp src/qtros.h src/openni_listener.h src/qt_gui.h src/graph_manager.h src/glviewer.h src/ros_service_ui.h)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})  

##############################################################################
# Sources to Compile
##############################################################################
SET(ADDITIONAL_SOURCES src/gicp-fallback.cpp src/main.cpp src/qtros.cpp  src/openni_listener.cpp src/qt_gui.cpp src/flow.cpp src/node.cpp src/graph_manager.cpp src/graph_mgr_io.cpp src/glviewer.cpp src/parameter_server.cpp src/ros_service_ui.cpp src/misc.cpp src/landmark.cpp src/loop_closing.cpp src/ColorOctomapServer.cpp src/scoped_timer.cpp src/icp.cpp src/matching_result.cpp)
SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/transformation_estimation_euclidean.cpp src/transformation_estimation.cpp src/graph_manager2.cpp)

IF (${USE_SIFT_GPU})
 	SET(ADDITIONAL_SOURCES ${ADDITIONAL_SOURCES} src/sift_gpu_wrapper.cpp)
ENDIF (${USE_SIFT_GPU})

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS ${ADDITIONAL_SOURCES})

##############################################################################
# Binaries
##############################################################################
rosbuild_add_executable(rgbdslam ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

#set libs
#SET(G2O_LIBS cholmod g2o_core g2o_stuff g2o_types_slam3d g2o_solver_cholmod g2o_solver_pcg g2o_solver_csparse cxsparse g2o_incremental)
SET(G2O_LIBS ${libg2o_LIBRARIES} cholmod cxsparse)
#SET(LIBS_LINK GL GLU -lgl2ps ${G2O_LIBS} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS})
SET(LIBS_LINK GL GLU ${G2O_LIBS} ${QT_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${OpenCV_LIBS} -lboost_signals -lrt -loctomap -loctomap_server -loctomath)

IF (${USE_SIFT_GPU})
 	SET(LIBS_LINK ${LIBS_LINK} siftgpu)
ENDIF (${USE_SIFT_GPU})
IF (${USE_GL2PS})
 	SET(LIBS_LINK ${LIBS_LINK} -lgl2ps)
ENDIF (${USE_GL2PS})
IF (${USE_GICP})
 	SET(LIBS_LINK ${LIBS_LINK} gicp ANN gsl gslcblas)
ENDIF (${USE_GICP})
#link libraries
target_link_libraries(rgbdslam ${LIBS_LINK})

 
IF (${USE_GICP})
  set(ROS_COMPILE_FLAGS ${ROS_COMPILE_FLAGS} -fpermissive)
  rosbuild_add_library(gicp external/gicp/bfgs_funcs.cpp external/gicp/gicp.cpp external/gicp/optimize.cpp external/gicp/scan.cpp external/gicp/transform.cpp)
ENDIF (${USE_GICP})

