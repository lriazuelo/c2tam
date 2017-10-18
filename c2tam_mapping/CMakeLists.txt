cmake_minimum_required(VERSION 2.8.3)
project(c2tam_mapping)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/c2tam/${PROJECT_NAME}/cmake_modules")
set(CMAKE_BUILD_TYPE None)

set(CMAKE_C_COMPILER g++)
set(CMAKE_CXX_COMPILER g++)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall -O2 -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I MY_CUSTOM_INCLUDE_PATH -D_LINUX -D_REENTRANT -D_SEQUENCE_ -Wall -O2 -march=native -fpermissive")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -L MY_CUSTOM_LINK_PATH")


find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs std_msgs cv_bridge tf tf_conversions image_transport pcl_ros c2tam_msgs c2tam_srvs)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CVD REQUIRED)
find_package(GVars3 REQUIRED)
find_package(TooN REQUIRED)



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


include_directories(
	include
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o
	${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/g2o 
	${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/g2o/core
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/build
        ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/g2o/EXTERNAL/eigen3
	/usr/include/suitesparse
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${OpenGL_INCLUDE_DIRS}
	${CVD_INCLUDE_DIR}
        ${GVars3_INCLUDE_DIR}
        ${TooN_INCLUDE_DIR}
       

)


link_directories(${catkin_LIBRARY_DIRS}
    ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib
)

add_executable(C2TAMmapping
                 ${CTAM_mapping_SRCS}
                 ${CTAM_mapping_MOC_SRCS}
)

target_link_libraries(C2TAMmapping 
    	lapack
	blas 
	pthread
	glut
	GLU
	GL
	g2o_core 
	g2o_solver_cholmod 
	g2o_stuff
	cholmod
	g2o_types_sba
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	${CVD_LIBRARY}
	${GVars3_LIBRARIES}
	${TooN_LIBRARIES}

)

add_custom_target(g2o_core DEPENDS ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_core.so)

add_dependencies(C2TAMmapping g2o_core )

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o/lib/libg2o_core.so 
	COMMAND sh install-external.sh
	WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o
	COMMENT Compilando g2o
)

add_dependencies(C2TAMtracking c2tam_msgs_gencpp)
add_dependencies(C2TAMtracking c2tam_srvs_gencpp)

catkin_package(
  INCLUDE_DIRS include ${OPENGL_INCLUDE_DIR}  ${OPENCV_INCLUDE_DIR}
  LIBRARIES ${OpenCV_LIBS} g2o_core g2o_solver_cholmod g2o_stuff cholmod g2o_types_sba 
  DEPENDS CVD GVars3 TooN

)




