#Set the project name here once. 
SET (PROJECT_NAME C2TAMtracking)

PROJECT(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()
rosbuild_add_boost_directories()

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()


SET (CMAKE_VERBOSE_MAKEFILE ON)


set(CMAKE_C_COMPILER g++)
set(CMAKE_CXX_COMPILER g++)
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT  -Wall  -O2 -march=nocona -msse3")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT  -Wall  -O2 -march=nocona -msse3")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall  -O2 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall  -march=native")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L MY_CUSTOM_LINK_PATH")

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

SET(C2TAM_tracking_MOC_HDRS  include/GLWindow2.h
	include/GLWindowMenu.h
	include/System.h
	include/ATANCamera.h
	include/KeyFrame.h
	include/AddKeyFrame.h
	include/MapPoint.h
	include/Map.h
	include/SmallBlurryImage.h
	include/ShiTomasi.h
	include/HomographyInit.h
	include/Bundle.h
	include/PatchFinder.h
	include/Relocaliser.h
	include/MiniPatch.h
	include/MapViewer.h
	include/Tracker.h
	include/VisualizationManager.h
)

set(C2TAM_tracking_SRCS
	src/main.cc
	src/GLWindow2.cc
	src/GLWindowMenu.cc
	src/System.cc
	src/ATANCamera.cc
	src/KeyFrame.cc
	src/AddKeyFrame.cc
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
	src/Tracker.cc
	src/VisualizationManager.cc
)


include_directories(${CMAKE_CURRENT_BINARY_DIR}
	EXTERNAL/gvars3
	EXTERNAL/libcvd
        EXTERNAL/TooNcvs
)


link_directories(${PROJECT_SOURCE_DIR}/EXTERNAL/gvars3
    ${PROJECT_SOURCE_DIR}/EXTERNAL/libcvd
)

rosbuild_add_executable(C2TAMtracking
                 ${C2TAM_tracking_SRCS}
                 ${C2TAM_tracking_MOC_SRCS}
)

target_link_libraries(C2TAMtracking 
    	lapack
        blas
    	GVars3
    	cvd 
	pthread
	glut
        GLU
	GL
	boost_signals
)


add_custom_target(GVars3 DEPENDS ${PROJECT_SOURCE_DIR}/EXTERNAL/gvars3/libGVars3.a)
add_custom_target(cvd DEPENDS ${PROJECT_SOURCE_DIR}/EXTERNAL/libcvd/libcvd.a)

add_dependencies(C2TAMtracking GVars3 cvd )

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/EXTERNAL/gvars3/libGVars3.a 
	COMMAND sh install-external.sh
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/EXTERNAL/gvars3
	COMMENT Compilando gvars3
)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/EXTERNAL/libcvd/libcvd.a 
	COMMAND sh install-external.sh
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/EXTERNAL/libcvd
	COMMENT Compilando cvd
)
