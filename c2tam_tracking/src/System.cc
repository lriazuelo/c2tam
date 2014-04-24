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

#include "System.h"
#include "OpenGL.h"
#include <gvars3/instances.h>
#include <stdlib.h>
#include "ATANCamera.h"
#include "Tracker.h"
#include "MapViewer.h"

#include<iostream>
#include<fstream>

#include <cvd/image_io.h>
#include <sys/time.h>


using namespace CVD;
using namespace std;
using namespace GVars3;

namespace enc = sensor_msgs::image_encodings;

typedef message_filters::Subscriber<sensor_msgs::Image> image_sub_type;      
typedef message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub_type;  

#ifdef _SEQUENCE_
  System::System(std::string path, double fps)
#else
  System::System()
#endif
{

  imageSize = new CVD::ImageRef(640,480);

  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);

  mimFrameBW.resize(*imageSize);
  mimFrameRGB.resize(*imageSize);

  // First, check if the camera is calibrated.
  // If not, we need to run the calibration widget.
  Vector<NUMTRACKERCAMPARAMETERS> vTest;

  vTest = GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("Camera.Parameters", ATANCamera::mvDefaultParams, HIDDEN);
  mpCamera = new ATANCamera("Camera");

  Vector<2> v2;
  if(v2==v2) ;
  if(vTest == ATANCamera::mvDefaultParams)
  {
    cout << endl;
    cout << "! Camera.Parameters is not set, need to run the CameraCalibrator tool" << endl;
    cout << "  and/or put the Camera.Parameters= line into the appropriate .cfg file." << endl;
    exit(1);
  }

  mpMap = new Map;

  tflistener_ = new tf::TransformListener(nh_);

  // Don't start until map is loaded
  nh_.param("C2TAMtracking/mode", modeTracking, 2); //0 //3 
  nh_.param("C2TAMtracking/freqVisual", freqVisual, 15);
  nh_.param("C2TAMtracking/freqObj", freqObj, 150);

  cntImg = 0; // Counter of visual freq
  cntImgO = 0; // Counter of visual freq


  //nh_.param("C2TAMtracking/stat", printStat, 0); 
  printStat = 0;

  loadMutex = PTHREAD_MUTEX_INITIALIZER;
  firstUnlock = false;
  if(modeTracking==1 || modeTracking==3){
    pthread_mutex_lock(&loadMutex); 
    firstUnlock = true;
    pthread_mutex_unlock(&loadMutex); 
  }

  detectObject = true;

  if(modeTracking !=3){
    mpTracker = new Tracker(*imageSize, *mpCamera, *mpMap, nh_, &idTracker, &detectObject);
  }

  mpMapViewer = new MapViewer(*mpMap);  

  GUI.ParseLine("GLWindow.AddMenu Menu Menu");
  GUI.ParseLine("Menu.ShowMenu Root");
  GUI.ParseLine("Menu.AddMenuButton Root Reset Reset Root");
  GUI.ParseLine("Menu.AddMenuButton Root Spacebar PokeTracker Root");
  GUI.ParseLine("DrawAR=0");
  GUI.ParseLine("DrawMap=0");
  GUI.ParseLine("Menu.AddMenuToggle Root \"View Map\" DrawMap Root");
  GUI.ParseLine("Menu.AddMenuToggle Root \"Draw AR\" DrawAR Root");

  mbDone = false;
  drawVirtualDense = false;

  frameAdded = false;

  start_vslam = false;

  sec=0;

};

bool System::startCB(c2tam_srvs::RunVslam::Request  &req, c2tam_srvs::RunVslam::Response &res )
{
ROS_INFO("startCB");
  if(req.run && req.mode == 0){
    start_vslam = true;
  }
  return true;
}

bool System::stopCB(c2tam_srvs::RunVslam::Request  &req, c2tam_srvs::RunVslam::Response &res )
{
  if(req.run && req.mode == 0){
    start_vslam = false;
  }
  return true;
}


bool System::GetMapObjectsCb(c2tam_srvs::GetMapObjects::Request  &req, c2tam_srvs::GetMapObjects::Response &res ){

  return true;
}


void System::updaterCallback(const c2tam_msgs::DataPoints::ConstPtr& msg)
{

  mpTracker->dataPointsCallback(msg);

  if(modeTracking==1 && firstUnlock){
    pthread_mutex_lock(&loadMutex); 
    firstUnlock = false;
    pthread_mutex_unlock(&loadMutex);

  }
}


void System::printTransform(const char* name, const tf::Transform t) {
  ROS_INFO("%s [%f.%f]: Translation %f %f %f",name, t.getOrigin().x(),t.getOrigin().y(),t.getOrigin().z());
  ROS_INFO("%s: Rotation %f %f %f  %f",t.getRotation().getX(), t.getRotation().getY(),t.getRotation().getZ(),t.getRotation().getW());
}



void System::imageDCallback(const sensor_msgs::ImageConstPtr& image_ptr, const sensor_msgs::ImageConstPtr& depth_ptr)
{
  struct timeval	tc;
  double t1,t2,t3,t4,t5,t6,t7,t8,t9,t10;

  if (start_vslam){

    tf::Transform camera_tf;

    pthread_mutex_lock(&loadMutex);

    if(!firstUnlock){ //NEW
      gettimeofday(&tc, NULL);
      t1=tc.tv_sec+tc.tv_usec/1e6; 

      pthread_mutex_unlock(&loadMutex); //NEW

      int bit_depth    = enc::bitDepth(image_ptr->encoding);
      int num_channels = enc::numChannels(image_ptr->encoding);

      int type = bit_depth == 8 ? CV_8U : CV_16U;

      cv::Mat color =  cv_bridge::toCvCopy(image_ptr)->image;
      cv::Mat gray =  cv_bridge::toCvCopy(image_ptr)->image;

      cv::cvtColor(color, gray, CV_RGB2GRAY);

      cv::GaussianBlur(gray,gray,  cv::Size(5,5), 0);

      memcpy(mimFrameBW.data(),gray.data,640*480*sizeof(uchar));

      mpCamera->SetImageSize(mimFrameBW.size());


      static bool bFirstFrame = true;
      if(bFirstFrame) {  bFirstFrame = false; }

      gettimeofday(&tc, NULL);
      t2=tc.tv_sec+tc.tv_usec/1e6;
    
      ////////  MAIN FUNCTION ///////
   
      cntImg++;
      cntImgO++;
      current_cam = mpTracker->TrackFrame(mimFrameBW,&frameAdded,&initMap);

      gettimeofday(&tc, NULL);
      t3=tc.tv_sec+tc.tv_usec/1e6;


      if(initMap){
        mpTracker->initColor_ptr =  cv_bridge::toCvCopy(image_ptr);
        mpTracker->initDepth_ptr =  cv_bridge::toCvCopy(depth_ptr);

        mpTracker->TrackForInitialMap_RGBD(mimFrameBW);

        gettimeofday(&tc, NULL);
        t4=tc.tv_sec+tc.tv_usec/1e6;
      }
      else{
        gettimeofday(&tc, NULL);
        t4=tc.tv_sec+tc.tv_usec/1e6; 
      }
      if (!mpTracker->notPose){

        if(frameAdded){

          mpTracker->mpAddKeyFrame->color_ptr =  cv_bridge::toCvCopy(image_ptr);
          mpTracker->mpAddKeyFrame->depth_ptr =  cv_bridge::toCvCopy(depth_ptr);

          gettimeofday(&tc, NULL);
          t5=tc.tv_sec+tc.tv_usec/1e6; 

          mpTracker->AddNewKeyFrame();

          gettimeofday(&tc, NULL);
          t6=tc.tv_sec+tc.tv_usec/1e6; 

          gettimeofday(&tc, NULL);
          t7=tc.tv_sec+tc.tv_usec/1e6; 
        }
        else{
          gettimeofday(&tc, NULL);
          t5=tc.tv_sec+tc.tv_usec/1e6; 
          gettimeofday(&tc, NULL);
          t6=tc.tv_sec+tc.tv_usec/1e6; 
          gettimeofday(&tc, NULL);
          t7=tc.tv_sec+tc.tv_usec/1e6; 
        }

        gettimeofday(&tc, NULL);
        t8=tc.tv_sec+tc.tv_usec/1e6;

        cam_pose.header.stamp = image_ptr->header.stamp; //ros::Time::now();  
        cam_pose.header.frame_id = "vslam_odom";
        cam_pose.pose.position.x = current_cam.get_translation()[0];// / mpMap->denseScale;
        cam_pose.pose.position.y = current_cam.get_translation()[1];// / mpMap->denseScale;
        cam_pose.pose.position.z = current_cam.get_translation()[2];// / mpMap->denseScale;

        camera_tf.setOrigin(tf::Vector3(cam_pose.pose.position.x,cam_pose.pose.position.y,cam_pose.pose.position.z));
        //camera_tf.setRotation();

        Vector<3> cam_rot = current_cam.get_rotation().ln();
        double theta = norm(cam_rot);
        normalize(cam_rot);
        //cam_rot = cam_rot / theta;
    
        if (std::isnan(cam_rot[0]))
          cam_pose.pose.orientation.x = 0;
        else
          cam_pose.pose.orientation.x = cam_rot[0] * sin(theta/2);

        if (std::isnan(cam_rot[1]))
          cam_pose.pose.orientation.y = 0;
        else
          cam_pose.pose.orientation.y = cam_rot[1] * sin(theta/2);

        if (std::isnan(cam_rot[2]))
          cam_pose.pose.orientation.z = 0;
        else
          cam_pose.pose.orientation.z = cam_rot[2] * sin(theta/2);

        cam_pose.pose.orientation.w = cos(theta/2);


        camera_tf.setRotation( tf::Quaternion(cam_pose.pose.orientation.x, cam_pose.pose.orientation.y, cam_pose.pose.orientation.z, 
				cam_pose.pose.orientation.w) );
        //camera_tf.setRotation( tf::Quaternion(0, 0, 0) );

        camera_pub.publish(cam_pose);

        cam_loc.header.stamp = image_ptr->header.stamp; //ros::Time::now();  
        cam_loc.header.frame_id = "vslam_odom";
        cam_loc.pose.position.x = current_cam.inverse().get_translation()[0] / mpMap->denseScale;
        cam_loc.pose.position.y = current_cam.inverse().get_translation()[1] / mpMap->denseScale;
        cam_loc.pose.position.z = current_cam.inverse().get_translation()[2] / mpMap->denseScale;

        Vector<3> cam_loc_rot = current_cam.inverse().get_rotation().ln();
        theta = norm(cam_loc_rot);
        normalize(cam_loc_rot);
        //cam_rot = cam_rot / theta;
    
        if (std::isnan(cam_loc_rot[0]))
          cam_loc.pose.orientation.x = 0;
        else
          cam_loc.pose.orientation.x = cam_loc_rot[0] * sin(theta/2);

        if (std::isnan(cam_loc_rot[1]))
          cam_loc.pose.orientation.y = 0;
        else
          cam_loc.pose.orientation.y = cam_loc_rot[1] * sin(theta/2);

        if (std::isnan(cam_loc_rot[2]))
          cam_loc.pose.orientation.z = 0;
        else
          cam_loc.pose.orientation.z = cam_loc_rot[2] * sin(theta/2);

        cam_loc.pose.orientation.w = cos(theta/2);

        camera_loc_pub.publish(cam_loc);

        gettimeofday(&tc, NULL);
        t9=tc.tv_sec+tc.tv_usec/1e6;

      }
      else{
        gettimeofday(&tc, NULL);
        t5=tc.tv_sec+tc.tv_usec/1e6;
        gettimeofday(&tc, NULL);
        t6=tc.tv_sec+tc.tv_usec/1e6;
        gettimeofday(&tc, NULL);
        t7=tc.tv_sec+tc.tv_usec/1e6;
        gettimeofday(&tc, NULL);
        t8=tc.tv_sec+tc.tv_usec/1e6;
        gettimeofday(&tc, NULL);
        t9=tc.tv_sec+tc.tv_usec/1e6;

      }

      if (cntImg == freqVisual){
        sendReal(gray);
        cntImg = 0;
      }

      gettimeofday(&tc, NULL);
      t10=tc.tv_sec+tc.tv_usec/1e6;

    } 
    else{
      if (modeTracking == 3){

        int bit_depth    = enc::bitDepth(image_ptr->encoding);
        int num_channels = enc::numChannels(image_ptr->encoding);

        int type = bit_depth == 8 ? CV_8U : CV_16U;

        const cv::Mat color(image_ptr->height, image_ptr->width, CV_MAKETYPE(type, num_channels),
        const_cast<uint8_t*>(&image_ptr->data[0]), image_ptr->step);

        cv::Mat gray(image_ptr->height, image_ptr->width, CV_MAKETYPE(type, 1),
                  const_cast<uint8_t*>(&image_ptr->data[0]), image_ptr->width * (bit_depth / 8));

        cv::cvtColor(color, gray, CV_RGB2GRAY);

        cv::GaussianBlur(gray,gray,  cv::Size(5,5), 0);
 
        memcpy(mimFrameBW.data(),gray.data,640*480*sizeof(uchar));

        c2tam_srvs::SearchForMap srv;
        srv.request.img = sensor_msgs::Image(*image_ptr);
        if (clientSearchMap->call(srv)){
          if(srv.response.id >= 0){
            modeTracking = 1;
            InitLoadMode(srv.response.id);
          } 
        }
        else{
         // ROS_ERROR("Failed to call service search_for_map");
        }
        usleep(300000);
      } 
      pthread_mutex_unlock(&loadMutex);
    }
  }
}





void System::InitLoadMode(int id){

  idTracker = id;

  mpTracker = new Tracker(*imageSize, *mpCamera, *mpMap, nh_, &idTracker, &detectObject);

  ostringstream nameServiceDP;
  nameServiceDP << "/map" << idTracker << "/dataPoints";

  data_points_sub = nh_updater_.subscribe(nameServiceDP.str().c_str(), 1000, &System::updaterCallback, this);

  fprintf(stderr,"Open port %d\n",idTracker);
  usleep(500000);

  c2tam_srvs::LoadIdMap srvId;
  srvId.request.id = id;
  if (clientLoadIdMap->call(srvId)){
    ROS_INFO("Map loaded, init recovery");
  }
  else
    ROS_ERROR("Failed to call service load_id_map");


  mpTracker->LoadIdMapClient(srvId.response.mdWiggleScaleDepthNormalized);

}

void System::Run()
{

  // RGB-Depth C2TAM
  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh_, "image_color", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh_, "image_depth", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(30), image_sub, depth_sub);
  sync.registerCallback(boost::bind(&System::imageDCallback, this, _1, _2));

  camera_pub = nh_.advertise<geometry_msgs::PoseStamped>("c2tam/camera_pose", 1000);
  camera_loc_pub = nh_.advertise<geometry_msgs::PoseStamped>("c2tam/camera_loc", 1000);

  nh_updater_.setCallbackQueue(&updater_queue_);

  if (modeTracking != 3){
    ostringstream nameServiceDP;
    nameServiceDP << "/map" << idTracker << "/dataPoints";
    data_points_sub = nh_updater_.subscribe(nameServiceDP.str().c_str(), 1000, &System::updaterCallback, this);
  }

  clientSearchMap = new ros::ServiceClient(nh_.serviceClient<c2tam_srvs::SearchForMap>("/search_for_map"));
  clientLoadIdMap = new ros::ServiceClient(nh_.serviceClient<c2tam_srvs::LoadIdMap>("/load_id_map"));
  visualization_pub = nh_.advertise<sensor_msgs::Image>("vslam/real_visualizer", 100);

  srvStart = nh_.advertiseService("/c2tam_vslam/start", &System::startCB, this);
  srvStop = nh_.advertiseService("/c2tam_vslam/stop", &System::stopCB, this);

  m_visualizer = new VisualizationManager(visualization_pub);

  ros::AsyncSpinner spinner(0, &updater_queue_);
  spinner.start();
  ros::spin();
}



void System::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if(sCommand=="quit" || sCommand == "exit")
    static_cast<System*>(ptr)->mbDone = true;
  if(sCommand=="KeyPress" && sParams == "d"){
    if(static_cast<System*>(ptr)->drawVirtualDense)
      static_cast<System*>(ptr)->drawVirtualDense = false;
    else
      static_cast<System*>(ptr)->drawVirtualDense = true;
  }
}


void System::sendReal(cv::Mat imgMat)
{

  CvScalar color = cvScalar(255, 0, 0);

  cv::Mat img;
  cv::cvtColor(imgMat, img, CV_GRAY2BGR);

  for(vector<TrackerData*>::reverse_iterator
    it = mpTracker->vTrackedFeatures.rbegin();
    it!= mpTracker->vTrackedFeatures.rend();
    it++)
  {
    if(! (*it)->bFound) {
    //  CvScalar color = cvScalar(255*gavLevelColors[(*it)->nSearchLevel][0], 255*gavLevelColors[(*it)->nSearchLevel][1], 255*gavLevelColors[(*it)->nSearchLevel+LEVELS][2]);
    //  cv::circle(img, cvPoint((*it)->v2Image[0], (*it)->v2Image[1]), (int)2, color, 2);
    }
    else{
    //  glColor(gavLevelColors[(*it)->nSearchLevel]);
    //  glVertex((*it)->v2Image);
      CvScalar color = cvScalar(255*gavLevelColors[(*it)->nSearchLevel][0], 255*gavLevelColors[(*it)->nSearchLevel][1], 255*gavLevelColors[(*it)->nSearchLevel][2]);
      cv::circle(img, cvPoint((*it)->v2Image[0], (*it)->v2Image[1]), (int)2, color, 2);
    }
  }

  m_visualizer->show(img,mpTracker->GetMessageForUser());

}


void System::drawVirtual(GLWindow2* w)
{
  w->make_current();
  w->SetupViewport();
  w->SetupVideoOrtho();
  w->SetupVideoRasterPosAndZoom();

  mpMapViewer->DrawMap(mpTracker->GetCurrentPose(), *w, drawVirtualDense);
  w->DrawCaption(mpMapViewer->GetMessageForUser());
  w->swap_buffers();
  w->HandlePendingEvents();
}


void System::drawUpdater(GLWindow2* w)
{
  w->make_current();
  w->SetupViewport();
  w->SetupVideoOrtho();
  w->SetupVideoRasterPosAndZoom();

  mpMapViewer->DrawMapUpdater(mpTracker->GetCurrentPose(), *w, drawVirtualDense);
  w->DrawCaption(mpMapViewer->GetMessageForUser());
  w->swap_buffers();
  w->HandlePendingEvents();
}
