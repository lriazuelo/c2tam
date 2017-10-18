cmake_minimum_required(VERSION 2.8.3)
project(c2tam_visualizer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/c2tam/${PROJECT_NAME}/cmake_modules")
set(CMAKE_BUILD_TYPE None)

set(CMAKE_C_COMPILER g++)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall  -O2 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall -O2 -march=native -fpermissive")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L MY_CUSTOM_LINK_PATH")


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs std_msgs cv_bridge image_transport c2tam_msgs c2tam_srvs)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CVD REQUIRED)
find_package(GVars3 REQUIRED)
find_package(TooN REQUIRED)

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


include_directories(
	include
   	${catkin_INCLUDE_DIRS}
	${CVD_INCLUDE_DIR}
        ${GVars3_INCLUDE_DIR}
        ${TooN_INCLUDE_DIR}

)


link_directories(${catkin_LIBRARY_DIRS}
)


add_executable(C2TAMvisualizer
                 ${CTAM_visualizer_SRCS}
                 ${CTAM_visualizer_MOC_SRCS}
)

target_link_libraries(C2TAMvisualizer 
    	lapack 
	pthread
	glut
        GLU
	GL
	${catkin_LIBRARIES}
	${CVD_LIBRARY}
	${GVars3_LIBRARIES}
	${TooN_LIBRARIES}
)


add_dependencies(C2TAMtracking c2tam_msgs_gencpp)
add_dependencies(C2TAMtracking c2tam_srvs_gencpp)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES  ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs nav_msgs std_msgs cv_bridge image_transport c2tam_msgs c2tam_srvs
  DEPENDS SuiteSparse CVD GVars3 TooN
)


