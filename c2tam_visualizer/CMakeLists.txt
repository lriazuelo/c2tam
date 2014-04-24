#Set the project name here once. 
SET (PROJECT_NAME C2TAMvisualizer)

PROJECT(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_add_boost_directories()

SET (CMAKE_VERBOSE_MAKEFILE ON)

set(CMAKE_C_COMPILER g++)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall  -O2 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall  -O2 -march=native")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L MY_CUSTOM_LINK_PATH")

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

SET(CTAM_visualizer_MOC_HDRS  include/GLWindow2.h
	include/GLWindowMenu.h
	include/Visualizer.h
	include/ATANCamera.h
	include/KeyFrame.h
	include/MapPoint.h
	include/MapObject.h
	include/Map.h
	include/SmallBlurryImage.h
	include/ShiTomasi.h
	include/HomographyInit.h
	include/Bundle.h
	include/PatchFinder.h
	include/Relocaliser.h
	include/MiniPatch.h
	include/MapViewer.h
)

set(CTAM_visualizer_SRCS
	src/main.cc
	src/GLWindow2.cc
	src/GLWindowMenu.cc
	src/ATANCamera.cc
	src/KeyFrame.cc
	src/MapPoint.cc
	src/Map.cc
	src/SmallBlurryImage.cc
	src/ShiTomasi.cc
	src/HomographyInit.cc
	src/Bundle.cc
	src/PatchFinder.cc
	src/Relocaliser.cc
	src/MiniPatch.cc
	src/MapViewer.cc
	src/Visualizer.cc
)


include_directories(${CMAKE_CURRENT_BINARY_DIR}
        ../c2tam_tracking/EXTERNAL/gvars3
        ../c2tam_tracking/EXTERNAL/libcvd
        ../c2tam_tracking/EXTERNAL/TooNcvs
        ../c2tam_tracking/EXTERNAL
)


link_directories(${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../c2tam_tracking/EXTERNAL/gvars3
    ${PROJECT_SOURCE_DIR}/../c2tam_tracking/EXTERNAL/libcvd
)

rosbuild_add_executable(C2TAMvisualizer
                 ${CTAM_visualizer_SRCS}
                 ${CTAM_visualizer_MOC_SRCS}
)

target_link_libraries(C2TAMvisualizer 
    	lapack 
    	GVars3
    	cvd 
	pthread
	glut
	GLU
	GL
)




