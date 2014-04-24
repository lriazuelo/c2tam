C2TAM: A Cloud framework for cooperative tracking and mapping
===================================

- Luis Riazuelo <riazuelo@unizar.es>  
- Javier Civera  
- Jose Maria Martinez Montiel

Robotics, Perception and Real-Time Group  
Aragón Institute of Engineering Research (I3A)  
University of Zaragoza, Spain  
<http://robots.unizar.es>  
<https://i3a.unizar.es>  

Overview
-----------------------------------

The c2tam repository contains a ROS stack of a cloud framework for a cooperative visual tracking and mapping. The softwares is tested under ros-fuerte distribution.

This software is based on PTAM "Parallel Tracking and Mapping" (http://www.robots.ox.ac.uk/~gk/PTAM/) library.

Installing
-----------------------------------

Download the stack from the repository:

    git clone https://github.com/lriazuelo/c2tam.git

Add the downloaded folder to your`$ROS_PACKAGE_PATH`.

Install any dependencies of the stack

	rosdep install c2tam

Install manually the dependencies

	sudo apt-get install libsuitesparse-dev libeigen3-dev libxmu-dev libxi-dev

Compile the packages on this order:

	rosmake c2tam_msgs
	rosmake c2tam_srvs
	rosmake c2tam_tracking
	rosmake c2tam_mapping
	rosmake c2tam_visualizer

Quick usage
-----------------------------------

Camera calibration

 * Edit camera.cfg file into c2tam_mapping and c2tam_tracking with the calibration of your camera sensor.
 * Modify camera_parameters.yaml file into c2tam_tracking/launch folder according with your camera sensor.

Connect your RGB-D camera and launch the <http://www.ros.org/wiki/openni_camera> Kinect sensor driver

	roslaunch openni_camera openni_node.launch

Next, start the C2TAM mapping node

	roslaunch c2tam_mapping server.launch

Once the mapping server is running, start the C2TAM tracking node

	roslaunch c2tam_tracking client.launch

Finally, launch the C2TAM visualizer node

	roslaunch c2tam_visualizer visualizer.launch

When all the nodes are running, we are ready to start the application by calling the service:

	rosservice call /c2tam_vslam/start true 0

The visual SLAM starts to build a map from freely hand-held camera image sequence. The c2tam_visualizer node has two displays. On the first one, it can be seen the real image gathered by the camera with the tracked map features backprojected. On the second one, you will see a virtual reconstruction of a 3D world displaying the keyframe locations, and all the 3D map points.

References
-----------------------------------

If you use this software in your reasearch, please cite the following journal paper:

L. RiazueloCorresponding, Javier Civera, J.M.M. Montiel
*C2TAM: A Cloud framework for cooperative tracking and mapping*.
Robotics and Autonomous Systems, Volume 62, Issue 4, Pages 401–413, April 2014

More info
-----------------------------------

Documentation:

 * Web site: https://sites.google.com/site/c2tamvisualslam/

Videos:

 * SLAM In the Cloud using C2TAM : https://www.youtube.com/watch?v=kE5wmFoCV5E
 * Cooperative SLAM In the Cloud using C2TAM : https://www.youtube.com/watch?v=giMDnKhkg-0

