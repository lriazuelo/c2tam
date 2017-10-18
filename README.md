Install
=====================================

mkdir catkin_c2tam
mkdir catkin_c2tam/src
cd catkin_c2tam/src
catkin_init_workspace
cd ..
catkin_make

gedit .bashrc
#source /opt/ros/indigo/setup.bash
source ~/catkin_c2tam/devel/setup.bash

cd ~/catkin_c2tam/src/
mkdir c2tam
cd c2tam
git clone -b ros-indigo https://lriazuelo@bitbucket.org/lriazuelo/c2tam.git .


roscd c2tam_tracking/EXTERNAL/gvars3/
sh install-external.sh 
cd ..
cd libcvd/
sh install-external.sh 

roscd c2tam_mapping/EXTERNAL/g2o/
sh install-external.sh

catkin_make --pkg c2tam_msgs c2tam_srvs c2tam_tracking c2tam_mapping c2tam_visualizer


Launch
=====================================

roscore 

roslaunch c2tam_mapping server.launch 

roslaunch c2tam_tracking client.launch > info.log

roslaunch c2tam_visualizer visualizer.launch

rosservice call /c2tam_vslam/start true 0





