#Set the project name here once. 
SET (PROJECT_NAME C2TAMmapping)

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
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall  -O2 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall  -O2 -march=native")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L MY_CUSTOM_LINK_PATH")

FIND_PACKAGE(Eigen3)
IF(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE ${EIGEN3_INCLUDE_DIR} CACHE PATH "Directory of Eigen3")
ELSE(EIGEN3_FOUND)
  SET(G2O_EIGEN3_INCLUDE "" CACHE PATH "Directory of Eigen3")
ENDIF(EIGEN3_FOUND)


#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

SET(CTAM_mapping_MOC_HDRS  include/GLWindow2.h
	include/GLWindowMenu.h
	include/System.h
	include/ATANCamera.h
	include/KeyFrame.h
	include/MapPoint.h
	include/Map.h
	include/SmallBlurryImage.h
	include/ShiTomasi.h
	include/HomographyInit.h
	include/MapMaker.h
	include/Bundle.h
	include/PatchFinder.h
	include/Relocaliser.h
	include/MiniPatch.h
	include/MapViewer.h
)

set(CTAM_mapping_SRCS
	src/main.cc
	src/GLWindow2.cc
	src/GLWindowMenu.cc
	src/System.cc
	src/ATANCamera.cc
	src/KeyFrame.cc
	src/MapPoint.cc
	src/Map.cc
	src/SmallBlurryImage.cc
	src/ShiTomasi.cc
	src/HomographyInit.cc
	src/MapMaker.cc
	src/Bundle.cc
	src/PatchFinder.cc
	src/Relocaliser.cc
	src/MiniPatch.cc
	src/MapViewer.cc
)


include_directories(${CMAKE_CURRENT_BINARY_DIR}
        ../c2tam_tracking/EXTERNAL/gvars3
        ../c2tam_tracking/EXTERNAL/libcvd
        ../c2tam_tracking/EXTERNAL/TooNcvs
        ../c2tam_tracking/EXTERNAL
        EXTERNAL/g2o
	EXTERNAL/g2o/g2o 
	EXTERNAL/g2o/g2o/core
        EXTERNAL/g2o/build
        EXTERNAL/g2o/g2o/EXTERNAL/eigen3
	/usr/include/suitesparse
       

)


link_directories(${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/../c2tam_tracking/EXTERNAL/gvars3
    ${PROJECT_SOURCE_DIR}/../c2tam_tracking/EXTERNAL/libcvd
    ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib
)

rosbuild_add_executable(C2TAMmapping
                 ${CTAM_mapping_SRCS}
                 ${CTAM_mapping_MOC_SRCS}
)

target_link_libraries(C2TAMmapping 
    	lapack
	blas 
    	GVars3
    	cvd 
	pthread
	glut
	GLU
	GL
	g2o_core 
	g2o_solver_cholmod 
	g2o_stuff
	cholmod
	g2o_types_sba

)

add_custom_target(g2o_core DEPENDS ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_core.so)

add_dependencies(C2TAMmapping g2o_core )

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_core.so 
	COMMAND sh install-external.sh
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o
	COMMENT Compilando g2o
)



