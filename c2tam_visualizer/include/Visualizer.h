/*
*
* This file is part of the c2tam project
*
* c2tam is free software: you can redistribute it and/or modify 
* it under the terms of the GNU General Public License as published by 
* the Free Software Foundation, either version 3 of the License, or 
* (at your option) any later version.
*
* c2tam is distributed in the hope that it will be useful, 
* but WITHOUT ANY WARRANTY; without even the implied warranty of 
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License 
* along with c2tam. If not, see http://www.gnu.org/licenses/.
*
*/

#ifndef __FINDER_H
#define __FINDER_H

#include <math.h>
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/thread.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>

#include "GLWindow2.h"
#include "OpenGL.h"

#include <gvars3/instances.h>
#include <stdlib.h>
#include <cvd/image_io.h>
#include <sys/time.h>

#include "MapViewer.h"
#include "c2tam_msgs/MapInfo.h"
#include "c2tam_msgs/RgbdInfo.h"
#include "c2tam_msgs/DenseKf.h"
#include "c2tam_msgs/DenseInfo.h"
#include "c2tam_srvs/LoadDrawingModel.h"
#include "c2tam_msgs/ObjectTransform.h"
#include "c2tam_msgs/ObjectTransformArray.h"

#include <geometry_msgs/PoseStamped.h>


class Map;
class MapViewer;

class Visualizer
{
    ros::NodeHandle nh_;

    CVD::ImageRef *imageSize;
    GLWindow2 *mGLWindowVirtual;

    Map *mpMap;
    MapViewer *mpMapViewer;

    ros::Timer *timerWait; 

    ros::Subscriber map_info_sub;
    ros::Subscriber dense_info_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber obj_trans_sub;

    ros::ServiceServer load_drawing_model_ ;

    pthread_mutex_t mapMutex;

    SE3<> se3CamFromW;

    bool should_continue;
    bool drawVirtualDense;

    int node_visualizer;

    double cam_cx;
    double cam_cy;
    double cam_fx;
    double cam_fy;

  public:

    Visualizer();

    void Run();

    static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);

    void drawVirtual(GLWindow2* w);

    void mapCb(const c2tam_msgs::MapInfo::ConstPtr& map_msg);

    void denseCb(const c2tam_msgs::DenseInfo::ConstPtr& dense_msg);

    void poseCb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    void objTransCb(const c2tam_msgs::ObjectTransformArray::ConstPtr& dense_msg);


    bool LoadDrawingModel(c2tam_srvs::LoadDrawingModel::Request  &req,
		           c2tam_srvs::LoadDrawingModel::Response &res );

    void waitCallback(const ros::TimerEvent&);

  private:
 
};

#endif
