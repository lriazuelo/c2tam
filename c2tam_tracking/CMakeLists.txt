
cmake_minimum_required(VERSION 2.8.3)
project(c2tam_tracking)


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs std_msgs cv_bridge image_transport tf pcl_ros pcl_conversions message_filters c2tam_msgs c2tam_srvs)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)

set(CMAKE_C_COMPILER g++)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall  -O2 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall -O2 -march=native -fpermissive")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L MY_CUSTOM_LINK_PATH")


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


include_directories(
	include
	EXTERNAL/gvars3
	EXTERNAL/libcvd
        EXTERNAL/TooNcvs
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${OpenGL_INCLUDE_DIRS}

)


link_directories(${PROJECT_SOURCE_DIR}/EXTERNAL/gvars3
    ${PROJECT_SOURCE_DIR}/EXTERNAL/libcvd
)

add_executable(C2TAMtracking
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
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
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

add_dependencies(C2TAMtracking c2tam_msgs_gencpp)
add_dependencies(C2TAMtracking c2tam_srvs_gencpp)



catkin_package(
#  DEPENDS opencv2
  INCLUDE_DIRS include ${OPENGL_INCLUDE_DIR}  ${OPENCV_INCLUDE_DIR}
  LIBRARIES ${OpenCV_LIBS}

)












