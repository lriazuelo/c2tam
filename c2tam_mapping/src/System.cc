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
#include "MapMaker.h"
#include "MapViewer.h"

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

#include<iostream>
#include<fstream>

#include <cvd/image_io.h>


using namespace CVD;
using namespace std;
using namespace GVars3;


System::System()
{
  sizeVideo = new CVD::ImageRef(640,480);

  mpMap.clear();
  mpMapMaker.clear();
  mpCamera.clear();

  std::string path_node;
  std::string path_file;
  FILE * find_file = popen("rospack find c2tam_mapping", "r");
  char command_find[1000];
  int numChar = fread(command_find, 1, 1000, find_file);
  command_find[numChar-1] = 0;
  path_node = command_find;

  std::string path_file_send;
  char filenameSend[255];
  sprintf(filenameSend,"/.times-send-3.dat");
  path_file_send = path_node + filenameSend;

  //mpSearchObjects = new SearchObjects();

};


//SERVICE  RequestReset

void System::RequestResetProcessCb(c2tam_srvs::RequestReset::Request  &req,
		           c2tam_srvs::RequestReset::Response &res )
{
  ROS_INFO("<ctam_MAPPING> RequestReset...");

  mpMapMaker[req.id]->RequestReset();
  res.reset = true;
}

void System::drawVirtual(GLWindow2* w)
{
 // w->make_current();
  w->SetupViewport();
  w->SetupVideoOrtho();
  w->SetupVideoRasterPosAndZoom();

  mpMapViewer->DrawMapServer(*w);
  w->swap_buffers();
  w->HandlePendingEvents();
}


bool System::RequestResetCb(c2tam_srvs::RequestReset::Request  &req,
		           c2tam_srvs::RequestReset::Response &res )
{
  boost::thread thrdRequestReset(boost::bind(&System::RequestResetProcessCb , this, req, res));
  thrdRequestReset.detach();

  return true;
}

//SERVICE  ResetDone

bool System::ResetDoneCb(c2tam_srvs::RequestReset::Request  &req,
		           c2tam_srvs::RequestReset::Response &res )
{
  ROS_INFO("<ctam_MAPPING> ResetDone...");
  res.reset = mpMapMaker[req.id]->ResetDone();
  return true;
}

//SERVICE  QueueSize



bool System::InitFromRGBDCb(c2tam_srvs::InitFromRGBD::Request  &req,
		           c2tam_srvs::InitFromRGBD::Response &res )
{


  sKeyFrame firstKf;
  firstKf.sMeasurements.clear(); 

  boost::shared_ptr<const sensor_msgs::Image> ros_img_ptr(new sensor_msgs::Image(req.First.img));

  IplImage *im;
  im = bridge_.imgMsgToCv(ros_img_ptr, "mono8");

  ImageRef imSize(im->width,im->height);
  Image<CVD::byte> imFrame(imSize);

  memcpy(imFrame.data(),im->imageData,im->width*im->height*sizeof(uchar));

  SE3 <> s;
  s.get_translation()[0] = req.First.se3CfromW[0];
  s.get_translation()[1] = req.First.se3CfromW[1];
  s.get_translation()[2] = req.First.se3CfromW[2];

  Matrix< 3 > mAux;

  mAux(0,0) = req.First.se3CfromW[3];
  mAux(0,1) = req.First.se3CfromW[4];
  mAux(0,2) = req.First.se3CfromW[5];

  mAux(1,0) = req.First.se3CfromW[6];
  mAux(1,1) = req.First.se3CfromW[7];
  mAux(1,2) = req.First.se3CfromW[8];

  mAux(2,0) = req.First.se3CfromW[9];
  mAux(2,1) = req.First.se3CfromW[10];
  mAux(2,2) = req.First.se3CfromW[11];

  s.get_rotation() = mAux;

  firstKf.imFrame = imFrame;
  firstKf.se3CfromW = s;
  firstKf.bFixed = req.First.bFixed;
  firstKf.bSemantic = false;
  
  firstKf.dSceneDepthMean = req.First.dSceneDepthMean;  
  firstKf.dSceneDepthSigma = req.First.dSceneDepthSigma; 

  vector<c2tam_msgs::Measurement>::const_iterator it;


  vector<c2tam_msgs::RgbdInfo>::const_iterator itDense;
  int u = 0;
  int v = 0;
  for(itDense = req.First.rgbd.begin(); itDense != req.First.rgbd.end(); ++itDense){
    if (v == 640){
      u++;
      v = 0;
    }
    rgbd denseData;
    denseData.r = (*itDense).r;
   denseData.g = (*itDense).g;
    denseData.b = (*itDense).b;
    denseData.z = (*itDense).z;
    denseData.x = (double) ((double) (((double) v - mpMapMaker[req.id]->cam_cx) * denseData.z) / (double) mpMapMaker[req.id]->cam_fx);
    denseData.y = (double) ((double) (((double) u - mpMapMaker[req.id]->cam_cy) * denseData.z) / (double) mpMapMaker[req.id]->cam_fy);
    firstKf.rgbdData.push_back(denseData); 
    v++;
  }

  SE3 <> skf ;
  skf = SE3<>();

  res.mdWiggleScaleDepthNormalized = mpMapMaker[req.id]->InitServFromRGBD
      (
         firstKf, skf);

  res.mse3CamFromWorld.push_back( skf.get_translation()[0]);
  res.mse3CamFromWorld.push_back( skf.get_translation()[1]);
  res.mse3CamFromWorld.push_back( skf.get_translation()[2]);

  Matrix< 3 > mAux_res;
  mAux_res = skf.get_rotation().get_matrix();

  res.mse3CamFromWorld.push_back(mAux_res(0,0));
  res.mse3CamFromWorld.push_back(mAux_res(0,1));
  res.mse3CamFromWorld.push_back(mAux_res(0,2));
  res.mse3CamFromWorld.push_back(mAux_res(1,0));
  res.mse3CamFromWorld.push_back(mAux_res(1,1));
  res.mse3CamFromWorld.push_back(mAux_res(1,2));
  res.mse3CamFromWorld.push_back(mAux_res(2,0));
  res.mse3CamFromWorld.push_back(mAux_res(2,1));
  res.mse3CamFromWorld.push_back(mAux_res(2,2));

  return true;

}

bool System::InitFromStereoEkfCb(c2tam_srvs::InitFromStereoEKF::Request  &req,
		           c2tam_srvs::InitFromStereoEKF::Response &res )
{


  sKeyFrame firstKf;
  firstKf.sMeasurements.clear(); 

  sKeyFrame currentKf;
  currentKf.sMeasurements.clear(); 
 
  boost::shared_ptr<const sensor_msgs::Image> ros_img_ptr(new sensor_msgs::Image(req.First.img));

  IplImage *im;
  // ros converts the image
  im = bridge_.imgMsgToCv(ros_img_ptr, "mono8");

  ImageRef imSize(im->width,im->height);
  Image<CVD::byte> imFrame(imSize);

  memcpy(imFrame.data(),im->imageData,im->width*im->height*sizeof(uchar));

  SE3 <> s;
  s.get_translation()[0] = req.First.se3CfromW[0];
  s.get_translation()[1] = req.First.se3CfromW[1];
  s.get_translation()[2] = req.First.se3CfromW[2];

  Matrix< 3 > mAux;

  mAux(0,0) = req.First.se3CfromW[3];
  mAux(0,1) = req.First.se3CfromW[4];
  mAux(0,2) = req.First.se3CfromW[5];

  mAux(1,0) = req.First.se3CfromW[6];
  mAux(1,1) = req.First.se3CfromW[7];
  mAux(1,2) = req.First.se3CfromW[8];

  mAux(2,0) = req.First.se3CfromW[9];
  mAux(2,1) = req.First.se3CfromW[10];
  mAux(2,2) = req.First.se3CfromW[11];

  s.get_rotation() = mAux;

  firstKf.imFrame = imFrame;
  firstKf.se3CfromW = s;
  firstKf.bFixed = req.First.bFixed;
  firstKf.bSemantic = false;
  
  firstKf.dSceneDepthMean = req.First.dSceneDepthMean;  
  firstKf.dSceneDepthSigma = req.First.dSceneDepthSigma; 

  vector<c2tam_msgs::Measurement>::const_iterator it;


  boost::shared_ptr<const sensor_msgs::Image> ros_img_ptr_c(new sensor_msgs::Image(req.Current.img));

  IplImage *im_c;
  // ros converts the image
  im_c = bridge_.imgMsgToCv(ros_img_ptr_c, "mono8");

  ImageRef imSize_c(im_c->width,im_c->height);
  Image<CVD::byte> imFrame_c(imSize_c);

  memcpy(imFrame_c.data(),im_c->imageData,im_c->width*im_c->height*sizeof(uchar));

  SE3 <> s_c;
  s_c.get_translation()[0] = req.Current.se3CfromW[0];
  s_c.get_translation()[1] = req.Current.se3CfromW[1];
  s_c.get_translation()[2] = req.Current.se3CfromW[2];

  Matrix< 3 > mAux_c;

  mAux_c(0,0) = req.Current.se3CfromW[3];
  mAux_c(0,1) = req.Current.se3CfromW[4];
  mAux_c(0,2) = req.Current.se3CfromW[5];

  mAux_c(1,0) = req.Current.se3CfromW[6];
  mAux_c(1,1) = req.Current.se3CfromW[7];
  mAux_c(1,2) = req.Current.se3CfromW[8];

  mAux_c(2,0) = req.Current.se3CfromW[9];
  mAux_c(2,1) = req.Current.se3CfromW[10];
  mAux_c(2,2) = req.Current.se3CfromW[11];

  s_c.get_rotation() = mAux_c;


  currentKf.imFrame = imFrame_c;
  currentKf.se3CfromW = s_c;
  currentKf.bFixed = req.Current.bFixed;
  currentKf.bSemantic = false;
  currentKf.dSceneDepthMean = req.Current.dSceneDepthMean;  
  currentKf.dSceneDepthSigma = req.Current.dSceneDepthSigma; 


  vector<c2tam_msgs::RgbdInfo>::const_iterator itDense;
  int u = 0;
  int v = 0;
  for(itDense = req.Current.rgbd.begin(); itDense != req.Current.rgbd.end(); ++itDense){
    if (v == 640){
      u++;
      v = 0;
    }
    rgbd denseData;
    denseData.r = (*itDense).r;
   denseData.g = (*itDense).g;
    denseData.b = (*itDense).b;
    denseData.z = (*itDense).z;
    denseData.x = (double) ((double) (((double) v - mpMapMaker[req.id]->cam_cx) * denseData.z) / (double) mpMapMaker[req.id]->cam_fx);
    denseData.y = (double) ((double) (((double) u - mpMapMaker[req.id]->cam_cy) * denseData.z) / (double) mpMapMaker[req.id]->cam_fy);
    currentKf.rgbdData.push_back(denseData); 
    v++;
  }


  vector<c2tam_msgs::Measurement>::const_iterator it_c;


  SE3 <> skf;

  skf.get_translation()[0] = req.mse3CamFromWorld[0];
  skf.get_translation()[1] = req.mse3CamFromWorld[1];
  skf.get_translation()[2] = req.mse3CamFromWorld[2];

  Matrix< 3 > mAuxkf;

  mAuxkf(0,0) = req.mse3CamFromWorld[3];
  mAuxkf(0,1) = req.mse3CamFromWorld[4];
  mAuxkf(0,2) = req.mse3CamFromWorld[5];

  mAuxkf(1,0) = req.mse3CamFromWorld[6];
  mAuxkf(1,1) = req.mse3CamFromWorld[7];
  mAuxkf(1,2) = req.mse3CamFromWorld[8];

  mAuxkf(2,0) = req.mse3CamFromWorld[9];
  mAuxkf(2,1) = req.mse3CamFromWorld[10];
  mAuxkf(2,2) = req.mse3CamFromWorld[11];

  skf.get_rotation() = mAuxkf;

  res.mdWiggleScaleDepthNormalized = mpMapMaker[req.id]->InitServFromStereo_EKF
      (
         firstKf, currentKf, skf
      );

  res.mse3CamFromWorld.push_back( skf.get_translation()[0]);
  res.mse3CamFromWorld.push_back( skf.get_translation()[1]);
  res.mse3CamFromWorld.push_back( skf.get_translation()[2]);

  Matrix< 3 > mAux_res;
  mAux_res = skf.get_rotation().get_matrix();

  res.mse3CamFromWorld.push_back(mAux_res(0,0));
  res.mse3CamFromWorld.push_back(mAux_res(0,1));
  res.mse3CamFromWorld.push_back(mAux_res(0,2));
  res.mse3CamFromWorld.push_back(mAux_res(1,0));
  res.mse3CamFromWorld.push_back(mAux_res(1,1));
  res.mse3CamFromWorld.push_back(mAux_res(1,2));
  res.mse3CamFromWorld.push_back(mAux_res(2,0));
  res.mse3CamFromWorld.push_back(mAux_res(2,1));
  res.mse3CamFromWorld.push_back(mAux_res(2,2));

  return true;
}


bool System::AttemptRecoveryCb(c2tam_srvs::AttemptRecovery::Request  &req,
		           c2tam_srvs::AttemptRecovery::Response &res )
{

  SE3 <> s1;
  SE3 <> s2;
  Vector<6> mv6;
  bool just;

  int indexRecoveryKF = -1;

  boost::shared_ptr<const sensor_msgs::Image> ros_img_ptr(new sensor_msgs::Image(req.img));

  IplImage *im;
  // ros converts the image
  im = bridge_.imgMsgToCv(ros_img_ptr, "mono8");

  ImageRef imSize(im->width,im->height);
  Image<CVD::byte> imFrame(imSize);
  memcpy(imFrame.data(),im->imageData,im->width*im->height*sizeof(uchar));

  res.recovery = mpMapMaker[req.id]->AttemptServiceRecovery(imFrame, just, mv6, s1, s2,&indexRecoveryKF);

  res.mse3StartPos.push_back( s1.get_translation()[0]);
  res.mse3StartPos.push_back( s1.get_translation()[1]);
  res.mse3StartPos.push_back( s1.get_translation()[2]);

  Matrix< 3 > mAux_res;
  mAux_res = s1.get_rotation().get_matrix();

  res.mse3StartPos.push_back(mAux_res(0,0));
  res.mse3StartPos.push_back(mAux_res(0,1));
  res.mse3StartPos.push_back(mAux_res(0,2));
  res.mse3StartPos.push_back(mAux_res(1,0));
  res.mse3StartPos.push_back(mAux_res(1,1));
  res.mse3StartPos.push_back(mAux_res(1,2));
  res.mse3StartPos.push_back(mAux_res(2,0));
  res.mse3StartPos.push_back(mAux_res(2,1));
  res.mse3StartPos.push_back(mAux_res(2,2));

  return true;
}

int System::RecoveryMultiMap(int id, sensor_msgs::Image img,SE3<> &poseInMap)
{

  SE3 <> s1;
  SE3 <> s2;
  Vector<6> mv6;
  bool just;
  double kfDist;

  int numMap = -1;
  int indexRecoveryKF = -1;
  boost::shared_ptr<const sensor_msgs::Image> ros_img_ptr(new sensor_msgs::Image(img));

  IplImage *im;
  // ros converts the image
  im = bridge_.imgMsgToCv(ros_img_ptr, "mono8");
  ImageRef imSize(im->width,im->height);
  Image<CVD::byte> imFrame(imSize);
  memcpy(imFrame.data(),im->imageData,im->width*im->height*sizeof(uchar));

  for (int i=0; i< mpMapMaker.size();i++){
    if (i != id && mpMapMaker[i]->recoveryActive){
      mpMapMaker[i]->mCurrentKF.MakeKeyFrame_Lite(imFrame);
      mpMapMaker[i]->mCurrentKF.MakeKeyFrame_Rest();
      if(mpMapMaker[i]->AttemptServiceRecovery(imFrame, just, mv6, s1, s2,&indexRecoveryKF)){
 
      SE3 <> auxV = mpMapMaker[i]->mse3CamFromWorld;

       mpMapMaker[i]->mbJustRecoveredSoUseCoarse = true;
       mpMapMaker[i]->TrackMap(imFrame);

       SE3 <> pT1 = mpMapMaker[i]->mse3CamFromWorld;
       mpMapMaker[i]->mse3CamFromWorld = auxV;

       mpMapMaker[i]->mbJustRecoveredSoUseCoarse = true;
       mpMapMaker[i]->TrackMap(imFrame);

       SE3 <> pT2 = mpMapMaker[i]->mse3CamFromWorld;
       mpMapMaker[i]->mse3CamFromWorld = auxV;

       mpMapMaker[i]->mbJustRecoveredSoUseCoarse = true;
       mpMapMaker[i]->TrackMap(imFrame);

       SE3 <> pT3 = mpMapMaker[i]->mse3CamFromWorld;
       mpMapMaker[i]->mse3CamFromWorld = auxV;

       mpMapMaker[i]->mbJustRecoveredSoUseCoarse = true;
       mpMapMaker[i]->TrackMap(imFrame);

       SE3 <> pT4 = mpMapMaker[i]->mse3CamFromWorld;
       mpMapMaker[i]->mse3CamFromWorld = auxV;

       if(mpMapMaker[i]->AssessTrackingQuality()){

       int sizeMeans = mpMapMaker[i]->mCurrentKF.mMeasurements.size();
         cv::Mat imagePoints(2,sizeMeans,CV_64F);
         cv::Mat cameraMatrix(3,3,CV_64F);
         cv::Mat distCoeffs(4,1,CV_64F);
         cv::Mat rvec(3,1,CV_64F);
         cv::Mat tvec(3,1,CV_64F);
         vector<int>  inliers_indices;
 
         vector<cv::Point3f> point3d_model_matched;
         vector<cv::Point2f> point2d_matched;
         point3d_model_matched.clear();
         point2d_matched.clear();

         for(meas_it it = mpMapMaker[i]->mCurrentKF.mMeasurements.begin(); it!=mpMapMaker[i]->mCurrentKF.mMeasurements.end(); it++)
         {
           cv::Point3f point3d;
           point3d.x = it->first->v3WorldPos[0];
           point3d.y = it->first->v3WorldPos[1];
           point3d.z = it->first->v3WorldPos[2];
           point3d_model_matched.push_back(point3d);
 
           cv::Point2f point2d;
           point2d.x = it->second.v2RootPos[0];
           point2d.y = it->second.v2RootPos[1];

           point2d_matched.push_back(point2d);
 
         }

         cameraMatrix.at<double>(0,0) = 526.6892;
         cameraMatrix.at<double>(0,1) = 0.0;
         cameraMatrix.at<double>(0,2) = 313.4897;
         cameraMatrix.at<double>(1,0) = 0.0;
         cameraMatrix.at<double>(1,1) = 526.9683;
         cameraMatrix.at<double>(1,2) = 263.5015;
         cameraMatrix.at<double>(2,0) = 0.0;
         cameraMatrix.at<double>(2,1) = 0.0;
         cameraMatrix.at<double>(2,2) = 1.0;

         distCoeffs.at<double>(0,0) = 0.1370;
         distCoeffs.at<double>(1,0) = -0.2156;
         distCoeffs.at<double>(2,0) = 0.0;
         distCoeffs.at<double>(3,0) = 0.0;


         Vector<3> vectPos = mpMapMaker[i]->mCurrentKF.se3CfromW.get_translation();
         Vector<3> vectRot = mpMapMaker[i]->mCurrentKF.se3CfromW.get_rotation().ln();
         tvec.at<double>(0,0) = vectPos[0];
         tvec.at<double>(1,0) = vectPos[1];
         tvec.at<double>(2,0) = vectPos[2];
         rvec.at<double>(0,0) = vectRot[0];
         rvec.at<double>(1,0) = vectRot[1];
         rvec.at<double>(2,0) = vectRot[2];


         cv::solvePnP(point3d_model_matched, point2d_matched, cameraMatrix, distCoeffs, rvec, tvec, true);
         //cv::solvePnPRansac(point3d_model_matched, point2d_matched, cameraMatrix, distCoeffs, rvec, tvec, true, 200, 8.0, 5, inliers_indices);

          if (!mpMapMaker[id]->vctRecovery[i].recovery){
            mpMapMaker[id]->vctRecovery[i].firstKF = true;
            mpMapMaker[id]->vctRecovery[i].recovery = true;
            mpMapMaker[id]->vctRecovery[i].idFirstKF = indexRecoveryKF;
            mpMapMaker[id]->vctRecovery[i].recoveryFirstKF = mpMapMaker[i]->mse3CamFromWorld;

            poseInMap.get_translation()[0] = tvec.at<double>(0,0);
            poseInMap.get_translation()[1] = tvec.at<double>(1,0);
            poseInMap.get_translation()[2] = tvec.at<double>(2,0);
            Vector<3> rotVect;
            rotVect[0] = rvec.at<double>(0,0);
            rotVect[1] = rvec.at<double>(1,0);
            rotVect[2] = rvec.at<double>(2,0);
            SO3<> rotMat(rotVect);
            poseInMap.get_rotation() = rotMat.get_matrix();

            numMap = i;
          }
        }
      }
    }
  }


  Matrix< 3 > mAux_res;
  mAux_res = s1.get_rotation().get_matrix();



  return numMap;
}

int System::SearchMultiMap(sensor_msgs::Image img)
{
  int retId = -1;
  SE3 <> s1;
  SE3 <> s2;
  Vector<6> mv6;
  bool just;

  int indexRecoveryKF = -1;
  boost::shared_ptr<const sensor_msgs::Image> ros_img_ptr(new sensor_msgs::Image(img));

  IplImage *im;
  // ros converts the image
  im = bridge_.imgMsgToCv(ros_img_ptr, "mono8");
  ImageRef imSize(im->width,im->height);
  Image<CVD::byte> imFrame(imSize);
  memcpy(imFrame.data(),im->imageData,im->width*im->height*sizeof(uchar));

  for (int i=0; i< mpMapMaker.size();i++){
    if (mpMapMaker[i]->recoveryActive){

      mpMapMaker[i]->mCurrentKF.MakeKeyFrame_Lite(imFrame);
      mpMapMaker[i]->mCurrentKF.MakeKeyFrame_Rest();
      if(mpMapMaker[i]->AttemptServiceRecovery(imFrame, just, mv6, s1, s2,&indexRecoveryKF)){

        mpMapMaker[i]->mbJustRecoveredSoUseCoarse = true;
        mpMapMaker[i]->TrackMap(imFrame);

        if(mpMapMaker[i]->AssessTrackingQuality()){
          retId = i;
        }
      }
    }
  }

  return retId;
}



bool System::SearchForMapCb(c2tam_srvs::SearchForMap::Request  &req, c2tam_srvs::SearchForMap::Response &res ){

  int returnId = SearchMultiMap(req.img);
  fprintf(stderr,"returnId %d\n",returnId);
  res.id = returnId;

}

bool System::LoadMapCb(c2tam_srvs::LoadMap::Request  &req, c2tam_srvs::LoadMap::Response &res ){

  float cx = cam_cx;
  float cy = cam_cy;
  float fx = cam_fx;
  float fy = cam_fy;


  //ROS_INFO("LoadMapCb");

  ifstream myVpPointsFile;
  ifstream myVpPointsKfFile;
  ifstream myVpPointsTrash;
  ifstream myVpPointsKfTrash;
  ifstream myKeyFrames;
  ifstream myDensePoints;

  std::string path_maps;
  std::string path_load_maps;
  std::string path_maps_d_kf;
  std::string path_maps_vp;
  std::string path_maps_vpANDkf;
  std::string path_maps_vpT;
  std::string path_maps_vpTANDkf;
  std::string path_maps_kf;
  std::string path_dense_points;
  std::string command_decompress;
  std::string command_directory;
  std::string command_remove;
  std::string command_delete;
  std::string command_file;
  std::string name_binary_file;


  FILE * find_file = popen("rospack find c2tam_mapping", "r");
  char command_find[1000];
  int numChar = fread(command_find, 1, 1000, find_file);
  command_find[numChar-1] = 0;

  path_maps = command_find;
  path_load_maps = path_maps + "/maps";

  //OLD LOAD MAP 
  command_directory = "find " + path_load_maps + " -name " + req.name + "_ctam_map";
  // command_directory = "find " + path_load_maps + " -name " + "map." + req.name + "_ctam_map";
  // ROBOEARTH_LOAD  
  //command_directory = "find " + path_load_maps + " -name " +  req.name + "_ctam_map";

  FILE * find_3_file = popen(command_directory.c_str(), "r");
  char command_3_find[1000];
  int numChar_3 = fread(command_3_find, 1, 1000, find_3_file);

  if (numChar_3 != 0 ){
      ROS_INFO("The directory exists");
      //OLD LOAD MAP 
      command_delete = "rm -r " + path_load_maps + "/" + req.name + "_ctam_map";
      //command_delete = "rm -r " + path_load_maps + "/" + "map." + req.name + "_ctam_map";
      // ROBOEARTH_LOAD
      //command_delete = "rm -r " + path_load_maps + "/" + req.name + "_ctam_map";

      FILE * delete_4_file = popen(command_delete.c_str(), "r");
      char command_4_delete[1000];
      int numChar_4 = fread(command_4_delete, 1, 1000, delete_4_file);
      if (numChar_4 != 0 ){
        ROS_INFO("Impossible remove the directory");
       } else{
        ROS_INFO("Directory removed .. create a new one");
      }
  }



  //OLD LOAD MAP  
  command_decompress = "cd " + path_maps + "/maps; tar -xzf " + req.name + ".tar.gz ";
  // ROBOEARTH_LOAD
  //command_decompress = "cd " + path_maps + "/maps; tar -xzf " + "vslam_map" + ".tar.gz ";
  FILE * find_1_file = popen(command_decompress.c_str(), "r");
  char command_1_find[1000];
  int numChar_1 = fread(command_1_find, 1, 1000, find_1_file);
  
if (numChar_1 == 0 ){

  //OLD LOAD MAP   
  path_maps = path_maps + "/maps/" + req.name + "_ctam_map";
  // path_maps = path_maps + "/maps/" + "map." + req.name + "_ctam_map";
  // ROBOEARTH_LOAD
  //path_maps = path_maps + "/maps/" + req.name + "_ctam_map";
  path_maps_d_kf = path_maps + "/kf";

  ROS_INFO("c2tam_mapping maps path %s", path_maps.c_str());

  path_maps_vp = path_maps + "/myVpPoints.dat";
  path_maps_vpANDkf = path_maps + "/myVpPointsKF.dat";
  path_maps_vpT = path_maps + "/myVpPointsTrash.dat";
  path_maps_vpTANDkf = path_maps + "/myVpPointsTrashKF.dat";
  path_maps_kf = path_maps + "/myKeyFrames.dat";
  path_dense_points = path_maps + "/myDensePoints.dat";

  int vpPointsSize;
  int vpKeyFramesSize;
  int mMeasurementsSize;
  int auxInt;
  int indexPoint;
  double auxDouble;
  double auxDouble2;
  int sizeVect;
  std::string auxStr;


  res.id = mpMap.size();

  Map * mapPTAM = new Map;

  MapMaker * mapMakerPTAM = new MapMaker(*mapPTAM, nh_, res.id,req.mode,cx,cy,fx,fy, false);


  mpMap.push_back(mapPTAM);

  mpMapViewer = new MapViewer(*mpMap[0]);  //xxx


  mpMapMaker.push_back(mapMakerPTAM);

  myVpPointsFile.open (path_maps_vp.c_str()); 
  myVpPointsFile.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)

   myVpPointsFile >> auxInt;
   if(auxInt == 1){
     mpMap[res.id]->bGood = true;
   }else{
     mpMap[res.id]->bGood = false;
   }

  myVpPointsFile >> auxDouble;
  mpMapMaker[res.id]->cam_cx = auxDouble;

  myVpPointsFile >> auxDouble;
  mpMapMaker[res.id]->cam_cy = auxDouble;

  myVpPointsFile >> auxDouble;
  mpMapMaker[res.id]->cam_fx = auxDouble;

  myVpPointsFile >> auxDouble;
  mpMapMaker[res.id]->cam_fy = auxDouble;

  myVpPointsFile >> auxDouble;
  mpMap[res.id]->denseScale = auxDouble; 

  myVpPointsFile >> auxDouble;
  mpMapMaker[res.id]->setmdWiggleScaleDepthNormalized(auxDouble); 

  myVpPointsFile >> vpPointsSize;
  mpMapMaker[res.id]->pubPoints.clear();



  for(unsigned int i=0; i< vpPointsSize; i++){

   MapPoint *pNew = new MapPoint();
   myVpPointsFile >> auxInt;
   pNew->indexPointMap = auxInt;

   myVpPointsFile >> auxDouble;
   pNew->v3WorldPos[0] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3WorldPos[1] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3WorldPos[2] = auxDouble;

   myVpPointsFile >> auxInt;
   if(auxInt == 1){
     pNew->bBad = true;
   }else{
     pNew->bBad = false;
   }

   myVpPointsFile >> auxInt;
   pNew->nSourceLevel = auxInt;

   myVpPointsFile >> auxDouble;
   pNew->irCenter.x = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->irCenter.y = auxDouble;

   myVpPointsFile >> auxDouble;
   pNew->v3Center_NC[0] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3Center_NC[1] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3Center_NC[2] = auxDouble;

   myVpPointsFile >> auxDouble;
   pNew->v3OneDownFromCenter_NC[0] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3OneDownFromCenter_NC[1] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3OneDownFromCenter_NC[2] = auxDouble;

   myVpPointsFile >> auxDouble;
   pNew->v3OneRightFromCenter_NC[0] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3OneRightFromCenter_NC[1] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3OneRightFromCenter_NC[2] = auxDouble;

   myVpPointsFile >> auxDouble;
   pNew->v3Normal_NC[0] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3Normal_NC[1] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3Normal_NC[2] = auxDouble;

   myVpPointsFile >> auxDouble;
   pNew->v3PixelDown_W[0] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3PixelDown_W[1] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3PixelDown_W[2] = auxDouble;

   myVpPointsFile >> auxDouble;
   pNew->v3PixelRight_W[0] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3PixelRight_W[1] = auxDouble;
   myVpPointsFile >> auxDouble;
   pNew->v3PixelRight_W[2] = auxDouble;

   myVpPointsFile >> auxDouble;
   pNew->nMEstimatorOutlierCount = auxDouble;

   myVpPointsFile >> auxDouble;
   pNew->nMEstimatorInlierCount = auxDouble;

   myVpPointsFile >> auxDouble;
   pNew->dCreationTime = auxDouble;

   mpMap[res.id]->vpPoints.push_back(pNew);

   mpMapMaker[res.id]->pubPoints.push_back(pNew);
  }
  myVpPointsFile.close();


  myVpPointsTrash.open (path_maps_vpT.c_str()); 
  myVpPointsTrash.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)

  myVpPointsTrash >> vpPointsSize;

  for(unsigned int i=0; i< vpPointsSize; i++){

   MapPoint *pNew = new MapPoint();
   myVpPointsTrash >> auxInt;
   pNew->indexPointMap = auxInt;

   myVpPointsTrash >> auxDouble;
   pNew->v3WorldPos[0] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3WorldPos[1] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3WorldPos[2] = auxDouble;


   myVpPointsTrash >> auxInt;
   if(auxInt == 1){
     pNew->bBad = true;
   }else{
     pNew->bBad = false;
   }

   myVpPointsTrash >> auxInt;
   pNew->nSourceLevel = auxInt;

   myVpPointsTrash >> auxDouble;
   pNew->irCenter.x = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->irCenter.y = auxDouble;

   myVpPointsTrash >> auxDouble;
   pNew->v3Center_NC[0] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3Center_NC[1] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3Center_NC[2] = auxDouble;

   myVpPointsTrash >> auxDouble;
   pNew->v3OneDownFromCenter_NC[0] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3OneDownFromCenter_NC[1] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3OneDownFromCenter_NC[2] = auxDouble;

   myVpPointsTrash >> auxDouble;
   pNew->v3OneRightFromCenter_NC[0] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3OneRightFromCenter_NC[1] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3OneRightFromCenter_NC[2] = auxDouble;

   myVpPointsTrash >> auxDouble;
   pNew->v3Normal_NC[0] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3Normal_NC[1] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3Normal_NC[2] = auxDouble;

   myVpPointsTrash >> auxDouble;
   pNew->v3PixelDown_W[0] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3PixelDown_W[1] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3PixelDown_W[2] = auxDouble;

   myVpPointsTrash >> auxDouble;
   pNew->v3PixelRight_W[0] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3PixelRight_W[1] = auxDouble;
   myVpPointsTrash >> auxDouble;
   pNew->v3PixelRight_W[2] = auxDouble;

   myVpPointsTrash >> auxDouble;
   pNew->nMEstimatorOutlierCount = auxDouble;

   myVpPointsTrash >> auxDouble;
   pNew->nMEstimatorInlierCount = auxDouble;

   myVpPointsTrash >> auxDouble;
   pNew->dCreationTime = auxDouble;
 
   mpMap[res.id]->vpPointsTrash.push_back(pNew);

  }

  myVpPointsTrash.close();


  myKeyFrames.open (path_maps_kf.c_str()); 
  myKeyFrames.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)

  myDensePoints.open (path_dense_points.c_str()); 
  myDensePoints.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)


  myKeyFrames >> vpKeyFramesSize;

  for(unsigned int j=0; j< vpKeyFramesSize; j++)
  {
     KeyFrame *kfNew = new KeyFrame;

     myKeyFrames >> auxInt;
     kfNew->index = auxInt;

     myKeyFrames >> auxDouble;
     kfNew->se3CfromW.get_translation()[0] = auxDouble;
     myKeyFrames >> auxDouble;
     kfNew->se3CfromW.get_translation()[1] = auxDouble;
     myKeyFrames >> auxDouble;
     kfNew->se3CfromW.get_translation()[2] = auxDouble;

     Matrix< 3 > mAux_res;
  
     myKeyFrames >> auxDouble;
     mAux_res(0,0) = auxDouble;
     myKeyFrames >> auxDouble;
     mAux_res(0,1) = auxDouble;
     myKeyFrames >> auxDouble;
     mAux_res(0,2) = auxDouble;

     myKeyFrames >> auxDouble;
     mAux_res(1,0) = auxDouble;
     myKeyFrames >> auxDouble;
     mAux_res(1,1) = auxDouble;
     myKeyFrames >> auxDouble;
     mAux_res(1,2) = auxDouble;

     myKeyFrames >> auxDouble;
     mAux_res(2,0) = auxDouble;
     myKeyFrames >> auxDouble;
     mAux_res(2,1) = auxDouble;
     myKeyFrames >> auxDouble;
     mAux_res(2,2) = auxDouble;

     kfNew->se3CfromW.get_rotation() = mAux_res;

     myKeyFrames >> auxInt;
     if(auxInt == 1){
       kfNew->bFixed = true;
     }else{
       kfNew->bFixed = false;
     }

     myKeyFrames >> auxDouble;
     kfNew->dSceneDepthMean = auxDouble;
     myKeyFrames >> auxDouble;
     kfNew->dSceneDepthSigma = auxDouble;


     myKeyFrames >> mMeasurementsSize;

     for(unsigned int it=0; it< mMeasurementsSize; it++)
     {
       Measurement m;
       myKeyFrames >> indexPoint;

       myKeyFrames >> auxInt;
       m.nLevel = auxInt;

       myKeyFrames >> auxInt;
       if(auxInt == 1){
         m.bSubPix= true;
       }else{
          m.bSubPix = false;
       }

       myKeyFrames >> auxDouble;
       m.v2RootPos[0] = auxDouble;
       myKeyFrames >> auxDouble;
       m.v2RootPos[1] = auxDouble;

       myKeyFrames >> auxInt;
       if (auxInt == 0)
         m.Source = Measurement::SRC_TRACKER;
       else if (auxInt == 1)
         m.Source = Measurement::SRC_REFIND;
       else if (auxInt == 2)
         m.Source = Measurement::SRC_ROOT;
       else if (auxInt == 3)
         m.Source = Measurement::SRC_TRAIL;
       else if (auxInt == 4)
         m.Source = Measurement::SRC_EPIPOLAR;

       kfNew->mMeasurements[mpMap[res.id]->vpPoints[indexPoint]] = m;
     }


     for(int l=0; l<LEVELS; l++)
     {

       ostringstream name_img;
       name_img << path_maps_d_kf <<  "/" << j <<  "_" <<  l <<  ".png";

       kfNew->aLevels[l].im = img_load(name_img.str().c_str());

       myKeyFrames >> auxInt;

       myKeyFrames >> sizeVect;
       for(unsigned int n=0; n < sizeVect; n++)
       {
          myKeyFrames >> auxDouble;
          myKeyFrames >> auxDouble2;
          CVD::ImageRef im_r(auxDouble,auxDouble2);
          kfNew->aLevels[l].vCorners.push_back(im_r);
       }


       myKeyFrames >> sizeVect;
       for(unsigned int n=0; n < sizeVect; n++)
       { 
          myKeyFrames >> auxInt;
          kfNew->aLevels[l].vCornerRowLUT.push_back(auxInt);
       }

       myKeyFrames >> sizeVect;
       for(unsigned int n=0; n < sizeVect; n++)
       { 
          myKeyFrames >> auxDouble;
          myKeyFrames >> auxDouble2;
          CVD::ImageRef im_r(auxDouble,auxDouble2);
          kfNew->aLevels[l].vMaxCorners.push_back(im_r);
       }

       myKeyFrames >> sizeVect;
       for(unsigned int n=0; n < sizeVect; n++)
       {
          Candidate c_;

          myKeyFrames >> auxDouble;
          myKeyFrames >> auxDouble2;
 
          CVD::ImageRef im_r(auxDouble,auxDouble2);
          c_.irLevelPos = im_r;

          myKeyFrames >> auxDouble;
          c_.v2RootPos[0] = auxDouble;
          myKeyFrames >> auxDouble;
          c_.v2RootPos[1] = auxDouble;

          myKeyFrames >> auxDouble;
          c_.dSTScore = auxDouble;
          kfNew->aLevels[l].vCandidates.push_back(c_);
       }

       myKeyFrames >> auxInt;
       if(auxInt == 1){
         kfNew->aLevels[l].bImplaneCornersCached = true;
       }else{
          kfNew->aLevels[l].bImplaneCornersCached = false;
       }

       myKeyFrames >> sizeVect;;
       for(unsigned int n=0; n < sizeVect; n++)
       {
          Vector<2> v_;

          myKeyFrames >> auxDouble;
          v_[0] = auxDouble;
          myKeyFrames >> auxDouble;
          v_[1] = auxDouble;
          kfNew->aLevels[l].vImplaneCorners.push_back(v_);
       }
     }


     kfNew->pSBI = new SmallBlurryImage(*kfNew);
     kfNew->pSBI->MakeJacs();

// DELETE DENSE/*
     myDensePoints >> auxInt;
     if (auxInt > 0){ 
         for (int u=0; u< 480; u++){
           for (int v=0; v< 640; v++){
             rgbd dataDense;
             myDensePoints >> auxInt;
             dataDense.r = auxInt;
             myDensePoints >> auxInt;
             dataDense.g = auxInt;
             myDensePoints >> auxInt;
             dataDense.b = auxInt;

             myDensePoints >> auxInt;
             if(auxInt == -1){
               dataDense.z = std::numeric_limits<float>::quiet_NaN();
               dataDense.x = std::numeric_limits<float>::quiet_NaN();
               dataDense.y = std::numeric_limits<float>::quiet_NaN();
             }
             else{
               myDensePoints >> auxDouble;
               dataDense.z = auxDouble;
               dataDense.x = (double) ((double) (((double) v - mpMapMaker[res.id]->cam_cx) * dataDense.z) / (double) mpMapMaker[res.id]->cam_fx);
               dataDense.y = (double) ((double) (((double) u - mpMapMaker[res.id]->cam_cy) * dataDense.z) / (double) mpMapMaker[res.id]->cam_fy);
             }
             kfNew->rgbdData.push_back(dataDense);
           }
         }
     }

     mpMap[res.id]->vpKeyFrames.push_back(kfNew);
  }
  myKeyFrames.close();



  myVpPointsKfFile.open (path_maps_vpANDkf.c_str()); 
  myVpPointsKfFile.precision (15); 

  for(unsigned int n=0; n< mpMap[res.id]->vpPoints.size();n++)
  {

    myVpPointsKfFile >> auxInt;
    mpMap[res.id]->vpPoints[n]->pPatchSourceKF = mpMap[res.id]->vpKeyFrames[auxInt];

    myVpPointsKfFile >> sizeVect;
    mpMap[res.id]->vpPoints[n]->pMMData = new MapMakerData();

    for(unsigned int r=0; r < sizeVect; r++)
    {
      myVpPointsKfFile >> auxInt;
     
      int a = mpMap[res.id]->vpKeyFrames[auxInt]->index;

      mpMap[res.id]->vpPoints[n]->pMMData->sMeasurementKFs.insert(mpMap[res.id]->vpKeyFrames[auxInt]);
    }

    myVpPointsKfFile >> sizeVect;
    for(unsigned int r=0; r < sizeVect; r++)
    {
      myVpPointsKfFile >> auxInt;
      mpMap[res.id]->vpPoints[n]->pMMData->sNeverRetryKFs.insert(mpMap[res.id]->vpKeyFrames[auxInt]);
    }
  }

  myVpPointsKfFile.close();

  res.mdWiggleScaleDepthNormalized = mpMapMaker[res.id]->getmdWiggleScaleDepthNormalized(); 
  res.queueSize = 0;
  mpMapMaker[res.id]->pubLoadData = true;
  pthread_mutex_unlock(&mpMapMaker[res.id]->loadSaveMutex);
}
else
  fprintf(stderr,"Map not found");

  //  command_remove = "rm -r " + path_load_maps+ "/" + "map." + req.name + "_ctam_map";
  // ROBOEARTH_LOAD
  //command_remove = "rm -r " + path_load_maps+ "/" +  req.name + "_ctam_map";
  //OLD LOAD MAP  
  command_remove = "rm -r " + path_load_maps+ "/" + req.name + "_ctam_map";
  FILE * find_6_file = popen(command_remove.c_str(), "r");
  char command_6_move[1000];
  int numChar_6 = fread(command_6_move, 1, 1000, find_6_file);
  if (numChar_6 == 0 ){
    //ROS_INFO("Remove temp directory");
  }


}

bool System::LoadIdCb(c2tam_srvs::LoadIdMap::Request  &req, c2tam_srvs::LoadIdMap::Response &res ){

  mpMapMaker[req.id]->pubLoadData = true;

  res.mdWiggleScaleDepthNormalized = mpMapMaker[req.id]->getmdWiggleScaleDepthNormalized();

}

bool System::GeneratePlyCb(c2tam_srvs::SaveMap::Request  &req, c2tam_srvs::SaveMap::Response &res ){

/*
  std::string path_save_maps;

  FILE * find_file = popen("rospack find c2tam_mapping", "r");
  char command_find[1000];
  int numChar = fread(command_find, 1, 1000, find_file);
  command_find[numChar-1] = 0;

  path_save_maps = command_find;
  path_save_maps = path_save_maps + "/maps/ply";
  int idMap;

  if (req.id < 0)
    idMap = mpMapMaker.size() - 1;
  else
    idMap = req.id;


  for(unsigned int j=0; j< mpMap[idMap]->vpKeyFrames.size(); j++)
  {
   KeyFrame &k = *mpMap[idMap]->vpKeyFrames[j];
//
//   myKeyFrames << k.index; myKeyFrames << "\t";
//
//   myKeyFrames << k.se3CfromW.get_translation()[0]; myKeyFrames << "\t";
//   myKeyFrames << k.se3CfromW.get_translation()[1]; myKeyFrames << "\t";
//   myKeyFrames << k.se3CfromW.get_translation()[2]; myKeyFrames << "\t";
//
//   Matrix< 3 > mAux;
//   mAux = k.se3CfromW.get_rotation().get_matrix();
// 

    if (mpMap[idMap]->vpKeyFrames[j]->rgbdData.size() > 0){
    ROS_INFO("%d",j);
    int indexP = 0;
    for (int u=0; u< 480; u++){
      for (int v=0; v< 640; v++){
        if(!std::isnan(mMap->vpKeyFrames[i]->rgbdData[indexP].z)){
          Vector<3> xyzPose;

          xyzPose[0] = mMap->vpKeyFrames[i]->rgbdData[indexP].x;
          xyzPose[1] = mMap->vpKeyFrames[i]->rgbdData[indexP].y;
          xyzPose[2] = mMap->vpKeyFrames[i]->rgbdData[indexP].z;
//
//          mMap->vpKeyFrames[i]->se3CfromW.get_translation()[0] = 
//          mMap->vpKeyFrames[i]->se3CfromW.get_translation()[0] / scale;
//
//          mMap->vpKeyFrames[i]->se3CfromW.get_translation()[1] = 
//          mMap->vpKeyFrames[i]->se3CfromW.get_translation()[1] / scale;
//
//          mMap->vpKeyFrames[i]->se3CfromW.get_translation()[2] = 
//          mMap->vpKeyFrames[i]->se3CfromW.get_translation()[2] / scale;
//
          xyzPose = mMap->vpKeyFrames[i]->se3CfromW.inverse() * xyzPose;

          myRgbd << xyzPose[0];// scale;
          myRgbd << " ";
          myRgbd << xyzPose[1];// * scale;
          myRgbd << " ";
          myRgbd << xyzPose[2];// * scale;
          myRgbd << " ";
          myRgbd << mMap->vpKeyFrames[i]->rgbdData[indexP].r;
          myRgbd << " ";
          myRgbd << mMap->vpKeyFrames[i]->rgbdData[indexP].g;
          myRgbd << " ";
          myRgbd << mMap->vpKeyFrames[i]->rgbdData[indexP].b;
          myRgbd << " ";
          myRgbd << 0;
          myRgbd << "\n"; 
          numPoints++;
        }
        indexP++;
      }
    }
  }








  }






  re_msgs::File fileMsg;
  res.vmap = fileMsg;
*/
  return true;
}

bool System::GetOctomapCb(c2tam_srvs::GetOctomap::Request  &req, c2tam_srvs::GetOctomap::Response &res ){

  //ROS_INFO("<ctam_MAPPING> Get Octomap...");

  int idMap;

  geometry_msgs::Pose cam_pose;
  tf::Transform camera_tf;
  geometry_msgs::Transform camera_msg;
  sensor_msgs::PointCloud2 cloudKF;

  c2tam_msgs::CreateOctomap msg_createOctomap;

  //octomap_msgs::GetOctomap srv_octomap;
  //re_msgs::File OctomapMsg;


  if (req.id < 0)
    idMap = mpMapMaker.size() - 1;
  else
    idMap = req.id;


  double scaleComputed = mpMapMaker[idMap]->computeScale();

//@roborgbd  for(unsigned int j=2; j< mpMap[idMap]->vpKeyFrames.size(); j=j+5)
  for(unsigned int j=1; j< mpMap[idMap]->vpKeyFrames.size()-3; j++)
  {
    ROS_INFO("GetOctomapCb KF %d",j);
    KeyFrame &k = *mpMap[idMap]->vpKeyFrames[j];

    if (k.rgbdData.size() > 0 && !k.bSemantic){
/*
    SE3 <> kfse3;
    SE3 <> rot;
    SE3 <> kfse3rot;

    kfse3 = k.se3CfromW;

    rot.get_translation()[0] = 0.0;
    rot.get_translation()[1] = 0.0;
    rot.get_translation()[2] = 0.0;

    Matrix< 3 > mAuxRot;
    mAuxRot(0,0) = 0.0;
    mAuxRot(0,1) = 0.0;
    mAuxRot(0,2) = 1.0;

    mAuxRot(1,0) = 1.0;
    mAuxRot(1,1) = 0.0;
    mAuxRot(1,2) = 0.0;

    mAuxRot(2,0) = 0.0;
    mAuxRot(2,1) = -1.0;
    mAuxRot(2,2) = 0.0;
 
    rot.get_rotation() = mAuxRot;

    kfse3rot = rot * kfse3;




     cam_pose.position.x = kfse3rot.get_translation()[0];
      cam_pose.position.y = kfse3rot.get_translation()[1];
      cam_pose.position.z = kfse3rot.get_translation()[2];



    camera_tf.setOrigin(tf::Vector3(cam_pose.position.x,cam_pose.position.y,cam_pose.position.z));

      //ROS_INFO("Camera x %f",k.se3CfromW.inverse().get_translation()[0]);

      Vector<3> cam_rot = kfse3rot.get_rotation().ln();
      double theta = norm(cam_rot);
      normalize(cam_rot);
    
      if (std::isnan(cam_rot[0]))
        cam_pose.orientation.x = 0;
      else
        cam_pose.orientation.x = cam_rot[0] * sin(theta/2);

      if (std::isnan(cam_rot[1]))
        cam_pose.orientation.y = 0;
      else
        cam_pose.orientation.y = cam_rot[1] * sin(theta/2);

      if (std::isnan(cam_rot[2]))
        cam_pose.orientation.z = 0;
      else
        cam_pose.orientation.z = cam_rot[2] * sin(theta/2);

      cam_pose.orientation.w = cos(theta/2);

      camera_tf.setRotation( tf::Quaternion(cam_pose.orientation.x, cam_pose.orientation.y, cam_pose.orientation.z, 
				cam_pose.orientation.w) );

      camera_tf.setOrigin(tf::Vector3(  cam_pose.position.x / scaleComputed,
      					cam_pose.position.y / scaleComputed,
      					cam_pose.position.z / scaleComputed));

      transformTFToMsg(camera_tf,camera_msg);

      msg_createOctomap.camera.push_back(camera_msg);
*/


      cam_pose.position.x = k.se3CfromW.inverse().get_translation()[0];
      cam_pose.position.y = k.se3CfromW.inverse().get_translation()[1];
      cam_pose.position.z = k.se3CfromW.inverse().get_translation()[2];

      camera_tf.setOrigin(tf::Vector3(cam_pose.position.x,cam_pose.position.y,cam_pose.position.z));

      //ROS_INFO("Camera x %f",k.se3CfromW.inverse().get_translation()[0]);

      Vector<3> cam_rot = k.se3CfromW.inverse().get_rotation().ln();
      double theta = norm(cam_rot);
      normalize(cam_rot);
    
      if (std::isnan(cam_rot[0]))
        cam_pose.orientation.x = 0;
      else
        cam_pose.orientation.x = cam_rot[0] * sin(theta/2);

      if (std::isnan(cam_rot[1]))
        cam_pose.orientation.y = 0;
      else
        cam_pose.orientation.y = cam_rot[1] * sin(theta/2);

      if (std::isnan(cam_rot[2]))
        cam_pose.orientation.z = 0;
      else
        cam_pose.orientation.z = cam_rot[2] * sin(theta/2);

      cam_pose.orientation.w = cos(theta/2);

      camera_tf.setRotation( tf::Quaternion(cam_pose.orientation.x, cam_pose.orientation.y, cam_pose.orientation.z, 
				cam_pose.orientation.w) );

      camera_tf.setOrigin(tf::Vector3(  cam_pose.position.x / scaleComputed,
      					cam_pose.position.y / scaleComputed,
      					cam_pose.position.z / scaleComputed));

      transformTFToMsg(camera_tf,camera_msg);

      msg_createOctomap.camera.push_back(camera_msg);

    }
  }
  create_octo_pub.publish (msg_createOctomap);

/*
  // GET OCTOMAP
  if (octomap_binary_.call(srv_octomap)){
    ROS_INFO("<CTAM_saver> Octomap obtained from octomap_server");
  }

  res.map.name = "octomap.bt";
  res.map.data.resize(srv_octomap.response.map.data.size());
  std::copy(srv_octomap.response.map.data.begin(), srv_octomap.response.map.data.end(), res.map.data.begin()); 
*/
  //res.map = OctomapMsg;


}

bool System::SaveMapCb(c2tam_srvs::SaveMap::Request  &req, c2tam_srvs::SaveMap::Response &res ){

  ROS_INFO("<ctam_MAPPING> SaveMap...");

  int idMap;

  if (req.id < 0)
    idMap = mpMapMaker.size() - 1;
  else
    idMap = req.id;

  ofstream myVpPointsFile;
  ofstream myVpPointsKfFile;
  ofstream myVpPointsTrash;
  ofstream myVpPointsKfTrash;
  ofstream myKeyFrames;
  ofstream myDensePoints;
  ofstream myRgbd;

  std::string path_maps;
  std::string path_save_maps;
  std::string path_maps_vp;
  std::string path_maps_vpANDkf;
  std::string path_maps_vpT;
  std::string path_maps_vpTANDkf;
  std::string path_maps_kf;
  std::string path_maps_ply;
  std::string path_dense_points;
  std::string path_images_kf;
  std::string command_map;
  std::string command_directory;
  std::string command_delete;
  std::string command_compress;
  std::string command_remove;

  FILE * find_file = popen("rospack find c2tam_mapping", "r");
  char command_find[1000];
  int numChar = fread(command_find, 1, 1000, find_file);
  command_find[numChar-1] = 0;

  path_save_maps = command_find;
  path_save_maps = path_save_maps + "/maps";
  //OLD_SAVE 
  command_map = "find " + path_save_maps + " -name " + req.name + ".tar.gz";
  //ROBOEARTH_SAVE 
  //command_map = "find " + path_save_maps + " -name " + "vslam_map.tar.gz";

  FILE * find_2_file = popen(command_map.c_str(), "r");
  char command_2_find[1000];
  int numChar_2 = fread(command_2_find, 1, 1000, find_2_file);

  if (numChar_2 != 0 ){
    command_delete = "rm " + path_save_maps + "/vslam_map.tar.gz";

    FILE * find_8_file = popen(command_delete.c_str(), "r");
    char command_8_find[1000];
    int numChar_8 = fread(command_8_find, 1, 1000, find_8_file);
    if (numChar_8 == 0 ){
      ROS_INFO("File removed");
    }
  }

    //OLD_SAVE 
    command_directory = "find " + path_save_maps + " -name " + req.name + "_ctam_map";
    //ROBOEARTH_SAVE 
    //command_directory = "find " + path_save_maps + " -name " + "map" + "." + req.name + "_ctam_map";

    FILE * find_3_file = popen(command_directory.c_str(), "r");
    char command_3_find[1000];
    int numChar_3 = fread(command_3_find, 1, 1000, find_3_file);

    if (numChar_3 == 0 ){
      //OLD_SAVE 
      path_maps = path_save_maps + "/" + req.name + "_ctam_map";
      //ROBOEARTH_SAVE 
      //path_maps = path_save_maps + "/" + "map" + "." + req.name + "_ctam_map";
      mkdir(path_maps.c_str(),0777); 
      path_maps_kf = path_maps + "/kf";
      mkdir(path_maps_kf.c_str(),0777); 
      ROS_INFO("The directory doesn't exist, we create it");
    }
    else{
      ROS_INFO("The directory exists");
      //OLD_SAVE 
      command_delete = "rm -r " + path_save_maps + "/" + req.name + "_ctam_map";
      //ROBOEARTH_SAVE 
      //command_delete = "rm -r " + path_save_maps + "/" + "map" + "." + req.name + "_ctam_map";

      FILE * find_4_file = popen(command_delete.c_str(), "r");
      char command_4_find[1000];
      int numChar_4 = fread(command_4_find, 1, 1000, find_4_file);
      if (numChar_4 == 0 ){
        ROS_INFO("Directory removed ... we create another directory");
        //OLD_SAVE 
        path_maps = path_save_maps + "/" + req.name + "_ctam_map";
        //ROBOEARTH_SAVE 
        //path_maps = path_save_maps + "/" + "map" + "." + req.name + "_ctam_map";
        mkdir(path_maps.c_str(),0777); 
        path_maps_kf = path_maps + "/kf";
        mkdir(path_maps_kf.c_str(),0777); 
      } else{
        ROS_INFO("Impossible to remove directory");
     //   res.msg="ERROR";
        return true;
      }
    }

  path_maps_vp = path_maps + "/myVpPoints.dat";
  path_maps_vpANDkf = path_maps + "/myVpPointsKF.dat";
  path_maps_vpT = path_maps + "/myVpPointsTrash.dat";
  path_maps_vpTANDkf = path_maps + "/myVpPointsTrashKF.dat";
  path_maps_kf = path_maps + "/myKeyFrames.dat";
  path_dense_points = path_maps + "/myDensePoints.dat";


  myVpPointsFile.open (path_maps_vp.c_str()); 
  myVpPointsFile.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)

  myVpPointsKfFile.open (path_maps_vpANDkf.c_str()); 
  myVpPointsKfFile.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)


   if(mpMap[idMap]->bGood){
     myVpPointsFile << 1; myVpPointsFile << "\n";
   }else{
     myVpPointsFile << 0; myVpPointsFile << "\n";
   }

  myVpPointsFile << mpMapMaker[idMap]->cam_cx; myVpPointsFile << "\n";

  myVpPointsFile << mpMapMaker[idMap]->cam_cy; myVpPointsFile << "\n";

  myVpPointsFile << mpMapMaker[idMap]->cam_fx; myVpPointsFile << "\n";

  myVpPointsFile << mpMapMaker[idMap]->cam_fy; myVpPointsFile << "\n";

  myVpPointsFile << mpMap[idMap]->denseScale; myVpPointsFile << "\n";

  myVpPointsFile << mpMapMaker[idMap]->getmdWiggleScaleDepthNormalized() << "\n"; 

  myVpPointsFile << mpMap[idMap]->vpPoints.size(); myVpPointsFile << "\n";

  for(unsigned int i=0; i< mpMap[idMap]->vpPoints.size(); i++){

   MapPoint &p = *mpMap[idMap]->vpPoints[i];
   myVpPointsFile << p.indexPointMap; myVpPointsFile << "\t";

   myVpPointsFile << p.v3WorldPos[0]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3WorldPos[1]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3WorldPos[2]; myVpPointsFile << "\t";

   if(p.bBad){
     myVpPointsFile << 1; myVpPointsFile << "\t";
   }else{
     myVpPointsFile << 0; myVpPointsFile << "\t";
   }

   myVpPointsFile << p.nSourceLevel; myVpPointsFile << "\t";

   myVpPointsFile << p.irCenter.x; myVpPointsFile << "\t";
   myVpPointsFile << p.irCenter.y; myVpPointsFile << "\t";

   myVpPointsFile << p.v3Center_NC[0]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3Center_NC[1]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3Center_NC[2]; myVpPointsFile << "\t";

   myVpPointsFile << p.v3OneDownFromCenter_NC[0]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3OneDownFromCenter_NC[1]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3OneDownFromCenter_NC[2]; myVpPointsFile << "\t";
 
   myVpPointsFile << p.v3OneRightFromCenter_NC[0]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3OneRightFromCenter_NC[1]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3OneRightFromCenter_NC[2]; myVpPointsFile << "\t";

   myVpPointsFile << p.v3Normal_NC[0]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3Normal_NC[1]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3Normal_NC[2]; myVpPointsFile << "\t";

   myVpPointsFile << p.v3PixelDown_W[0]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3PixelDown_W[1]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3PixelDown_W[2]; myVpPointsFile << "\t";

   myVpPointsFile << p.v3PixelRight_W[0]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3PixelRight_W[1]; myVpPointsFile << "\t";
   myVpPointsFile << p.v3PixelRight_W[2]; myVpPointsFile << "\t";

   myVpPointsFile << p.nMEstimatorOutlierCount; myVpPointsFile << "\t";

   myVpPointsFile << p.nMEstimatorInlierCount; myVpPointsFile << "\t";

   myVpPointsFile << p.dCreationTime; myVpPointsFile << "\n";

   myVpPointsKfFile << p.pPatchSourceKF->index; myVpPointsKfFile << "\t";

   if (p.pMMData->sMeasurementKFs.size() == 0){

     myVpPointsKfFile << 0; myVpPointsKfFile << "\t";
   }
   else{
     myVpPointsKfFile << p.pMMData->sMeasurementKFs.size(); myVpPointsKfFile << "\t";

     for(set<KeyFrame*>::iterator location = p.pMMData->sMeasurementKFs.begin();
        location!=p.pMMData->sMeasurementKFs.end(); location++)
     {
       myVpPointsKfFile << (*location)->index; myVpPointsKfFile << "\t";
     };
   }

   if (p.pMMData->sNeverRetryKFs.size() == 0){

     myVpPointsKfFile << 0; myVpPointsKfFile << "\n";
   }
   else{

     myVpPointsKfFile << p.pMMData->sNeverRetryKFs.size(); 

     for(set<KeyFrame*>::iterator location = p.pMMData->sNeverRetryKFs.begin();
        location!=p.pMMData->sNeverRetryKFs.end(); location++)
     {
       myVpPointsKfFile << "\t"; myVpPointsKfFile << (*location)->index;
     };
     myVpPointsKfFile << "\n";
   }
  }

  myVpPointsFile.close();
  myVpPointsKfFile.close();


  myVpPointsTrash.open (path_maps_vpT.c_str()); 
  myVpPointsTrash.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)

  myVpPointsKfTrash.open (path_maps_vpTANDkf.c_str()); 
  myVpPointsKfTrash.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)

  myVpPointsTrash << mpMap[idMap]->vpPointsTrash.size(); myVpPointsTrash << "\n";

  for(unsigned int i=0; i< mpMap[idMap]->vpPointsTrash.size(); i++){
   MapPoint &p = *mpMap[idMap]->vpPoints[i];
   myVpPointsTrash << p.indexPointMap; myVpPointsTrash << "\t";

   myVpPointsTrash << p.v3WorldPos[0]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3WorldPos[1]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3WorldPos[2]; myVpPointsTrash << "\t";

   if(p.bBad){
     myVpPointsTrash << 1; myVpPointsTrash << "\t";
   }else{
     myVpPointsTrash << 0; myVpPointsTrash << "\t";
   }

   myVpPointsTrash << p.nSourceLevel; myVpPointsTrash << "\t";

   myVpPointsTrash << p.irCenter.x; myVpPointsTrash << "\t";
   myVpPointsTrash << p.irCenter.y; myVpPointsTrash << "\t";


   myVpPointsTrash << p.v3Center_NC[0]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3Center_NC[1]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3Center_NC[2]; myVpPointsTrash << "\t";


   myVpPointsTrash << p.v3OneDownFromCenter_NC[0]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3OneDownFromCenter_NC[1]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3OneDownFromCenter_NC[2]; myVpPointsTrash << "\t";


   myVpPointsTrash << p.v3OneRightFromCenter_NC[0]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3OneRightFromCenter_NC[1]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3OneRightFromCenter_NC[2]; myVpPointsTrash << "\t";


   myVpPointsTrash << p.v3Normal_NC[0]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3Normal_NC[1]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3Normal_NC[2]; myVpPointsTrash << "\t";

   myVpPointsTrash << p.v3PixelDown_W[0]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3PixelDown_W[1]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3PixelDown_W[2]; myVpPointsTrash << "\t";

   myVpPointsTrash << p.v3PixelRight_W[0]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3PixelRight_W[1]; myVpPointsTrash << "\t";
   myVpPointsTrash << p.v3PixelRight_W[2]; myVpPointsTrash << "\t";

   myVpPointsTrash << p.nMEstimatorOutlierCount; myVpPointsTrash << "\t";

   myVpPointsTrash << p.nMEstimatorInlierCount; myVpPointsTrash << "\t";

   myVpPointsTrash << p.dCreationTime; myVpPointsTrash << "\n";

   myVpPointsKfTrash << p.pPatchSourceKF->index; myVpPointsKfTrash << "\t";

   if (p.pMMData->sMeasurementKFs.size() == 0){
     myVpPointsKfTrash << 0; myVpPointsKfTrash << "\t";
   }
   else{
     myVpPointsKfTrash << p.pMMData->sMeasurementKFs.size(); myVpPointsKfTrash << "\t";

     for(set<KeyFrame*>::iterator location = p.pMMData->sMeasurementKFs.begin();
        location!=p.pMMData->sMeasurementKFs.end(); location++)
     {
       myVpPointsKfTrash << (*location)->index; myVpPointsKfTrash << "\t";
     };
   }

   if (p.pMMData->sNeverRetryKFs.size() == 0){
     myVpPointsKfTrash << 0; myVpPointsKfTrash << "\n";
   }
   else{
     myVpPointsKfTrash << p.pMMData->sNeverRetryKFs.size(); 

     for(set<KeyFrame*>::iterator location = p.pMMData->sNeverRetryKFs.begin();
        location!=p.pMMData->sNeverRetryKFs.end(); location++)
     {
       myVpPointsKfTrash << "\t"; myVpPointsKfTrash << (*location)->index;
     };
     myVpPointsKfTrash << "\n";
   }
  }

  myVpPointsTrash.close();
  myVpPointsKfTrash.close();


  myKeyFrames.open (path_maps_kf.c_str()); 
  myKeyFrames.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)

  myDensePoints.open (path_dense_points.c_str()); 
  myDensePoints.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)

  myKeyFrames << mpMap[idMap]->vpKeyFrames.size(); myKeyFrames << "\n";

  for(unsigned int j=0; j< mpMap[idMap]->vpKeyFrames.size(); j++)
  {
   KeyFrame &k = *mpMap[idMap]->vpKeyFrames[j];

   myKeyFrames << k.index; myKeyFrames << "\t";

   myKeyFrames << k.se3CfromW.get_translation()[0]; myKeyFrames << "\t";
   myKeyFrames << k.se3CfromW.get_translation()[1]; myKeyFrames << "\t";
   myKeyFrames << k.se3CfromW.get_translation()[2]; myKeyFrames << "\t";

   Matrix< 3 > mAux;
   mAux = k.se3CfromW.get_rotation().get_matrix();

   myKeyFrames << mAux(0,0); myKeyFrames << "\t";
   myKeyFrames << mAux(0,1); myKeyFrames << "\t";
   myKeyFrames << mAux(0,2); myKeyFrames << "\t";
   myKeyFrames << mAux(1,0); myKeyFrames << "\t";
   myKeyFrames << mAux(1,1); myKeyFrames << "\t";
   myKeyFrames << mAux(1,2); myKeyFrames << "\t";
   myKeyFrames << mAux(2,0); myKeyFrames << "\t";
   myKeyFrames << mAux(2,1); myKeyFrames << "\t";
   myKeyFrames << mAux(2,2); myKeyFrames << "\t";

   if(k.bFixed){
     myKeyFrames << 1; myKeyFrames << "\t";
   }else{
     myKeyFrames << 0; myKeyFrames << "\t";
   }

   myKeyFrames << k.dSceneDepthMean; myKeyFrames << "\t";
   myKeyFrames << k.dSceneDepthSigma; myKeyFrames << "\n";


   myKeyFrames << k.mMeasurements.size(); //myKeyFrames << "\t";
   for(meas_it it = k.mMeasurements.begin(); it!=k.mMeasurements.end(); it++)
   {
     myKeyFrames << "\t"; myKeyFrames << it->first->indexPointMap; myKeyFrames << "\t";
     myKeyFrames << it->second.nLevel; myKeyFrames << "\t";

     if(it->second.bSubPix){
       myKeyFrames << 1; myKeyFrames << "\t";
     }else{
       myKeyFrames << 0; myKeyFrames << "\t";
     }

     myKeyFrames << it->second.v2RootPos[0]; myKeyFrames << "\t";
     myKeyFrames << it->second.v2RootPos[1]; myKeyFrames << "\t";

     myKeyFrames << it->second.Source; 
   }
   myKeyFrames << "\n";

   for(int l=0; l<LEVELS; l++)
   {
     ostringstream name_img;
     name_img << path_maps <<  "/kf/" << j <<  "_" <<  l <<  ".png";
     img_save(k.aLevels[l].im, name_img.str().c_str());

     myKeyFrames << l; myKeyFrames << "\t";

     myKeyFrames << k.aLevels[l].vCorners.size(); myKeyFrames << "\t";
     for(unsigned int n=0; n< k.aLevels[l].vCorners.size(); n++)
     {
       myKeyFrames << k.aLevels[l].vCorners[n].x ; myKeyFrames << "\t";
       myKeyFrames << k.aLevels[l].vCorners[n].y ; myKeyFrames << "\t";
     }

     myKeyFrames << k.aLevels[l].vCornerRowLUT.size(); myKeyFrames << "\t";
     for(unsigned int n=0; n< k.aLevels[l].vCornerRowLUT.size(); n++)
     {
       myKeyFrames << k.aLevels[l].vCornerRowLUT[n] ; myKeyFrames << "\t";
     }

     myKeyFrames << k.aLevels[l].vMaxCorners.size(); myKeyFrames << "\t";
     for(unsigned int n=0; n< k.aLevels[l].vMaxCorners.size(); n++)
     {
       myKeyFrames << k.aLevels[l].vMaxCorners[n].x ; myKeyFrames << "\t";
       myKeyFrames << k.aLevels[l].vMaxCorners[n].y ; myKeyFrames << "\t";
     }

     myKeyFrames << k.aLevels[l].vCandidates.size(); myKeyFrames << "\t";
     for(unsigned int n=0; n< k.aLevels[l].vCandidates.size(); n++)
     {
       myKeyFrames << k.aLevels[l].vCandidates[n].irLevelPos.x ; myKeyFrames << "\t";
       myKeyFrames << k.aLevels[l].vCandidates[n].irLevelPos.y ; myKeyFrames << "\t";

       myKeyFrames << k.aLevels[l].vCandidates[n].v2RootPos[0] ; myKeyFrames << "\t";
       myKeyFrames << k.aLevels[l].vCandidates[n].v2RootPos[1] ; myKeyFrames << "\t";

       myKeyFrames << k.aLevels[l].vCandidates[n].dSTScore ; myKeyFrames << "\t";
     }

     if(k.aLevels[l].bImplaneCornersCached){
       myKeyFrames << 1; myKeyFrames << "\t";
     }else{
       myKeyFrames << 0; myKeyFrames << "\t";
     }

     myKeyFrames << k.aLevels[l].vImplaneCorners.size(); 
     for(unsigned int n=0; n< k.aLevels[l].vImplaneCorners.size(); n++)
     {
       myKeyFrames << "\t"; myKeyFrames << k.aLevels[l].vImplaneCorners[n][0]; myKeyFrames << "\t";
       myKeyFrames << k.aLevels[l].vImplaneCorners[n][1];
     }
     myKeyFrames << "\n";

   }

   /* DENSE POINTS */
   myDensePoints << k.rgbdData.size();
   for(unsigned int denseIdx=0; denseIdx< k.rgbdData.size(); denseIdx++)
   {
     myDensePoints << "\t"; myDensePoints << k.rgbdData[denseIdx].r; 
     myDensePoints << "\t"; myDensePoints << k.rgbdData[denseIdx].g;
     myDensePoints << "\t"; myDensePoints << k.rgbdData[denseIdx].b;
     if (std::isnan(k.rgbdData[denseIdx].z)){
       myDensePoints << "\t"; myDensePoints << -1;
     }else{ 
       myDensePoints << "\t"; myDensePoints << 1;
      myDensePoints << "\t"; myDensePoints << k.rgbdData[denseIdx].z;
     }
   }
   myDensePoints << "\n";
   
  }

  path_maps_ply = path_maps + "/myPly.ply";

  myRgbd.open (path_maps_ply.c_str()); 
  myRgbd.precision (15); // long double Precisión científica (18-digitos); double Precisión científica (15-digitos)

  int cntpnts = 0;

  for(unsigned int i=0; i< mpMap[idMap]->vpKeyFrames.size(); i++)
  {
   if (mpMap[idMap]->vpKeyFrames[i]->rgbdData.size() > 0){
    int indexP = 0;
    for (int u=0; u< 480; u++){
      for (int v=0; v< 640; v++){
        if(!std::isnan(mpMap[idMap]->vpKeyFrames[i]->rgbdData[indexP].z)){
		cntpnts++;
	}
	indexP++;
      }	
    }
   }
  }

  myRgbd << "ply";
  myRgbd << "\n"; 
  myRgbd << "format ascii 1.0";
  myRgbd << "\n"; 
  myRgbd << "element vertex " ;
  myRgbd << cntpnts;
  myRgbd << "\n"; 
  myRgbd << "property float x";
  myRgbd << "\n"; 
  myRgbd << "property float y";
  myRgbd << "\n"; 
  myRgbd << "property float z";
  myRgbd << "\n"; 
  myRgbd << "property uchar red";
  myRgbd << "\n"; 
  myRgbd << "property uchar green";
  myRgbd << "\n"; 
  myRgbd << "property uchar blue";
  myRgbd << "\n"; 
  myRgbd << "property uchar alpha";
  myRgbd << "\n"; 
  myRgbd << "end_header";
  myRgbd << "\n"; 






  for(unsigned int i=0; i< mpMap[idMap]->vpKeyFrames.size(); i++)
  {
   if (mpMap[idMap]->vpKeyFrames[i]->rgbdData.size() > 0){

    int indexP = 0;
    for (int u=0; u< 480; u++){
      for (int v=0; v< 640; v++){
        if(!std::isnan(mpMap[idMap]->vpKeyFrames[i]->rgbdData[indexP].z)){
          Vector<3> xyzPose;

          xyzPose[0] = mpMap[idMap]->vpKeyFrames[i]->rgbdData[indexP].x;
          xyzPose[1] = mpMap[idMap]->vpKeyFrames[i]->rgbdData[indexP].y;
          xyzPose[2] = mpMap[idMap]->vpKeyFrames[i]->rgbdData[indexP].z;

          xyzPose = mpMap[idMap]->vpKeyFrames[i]->se3CfromW.inverse() * xyzPose;

          myRgbd << xyzPose[0];// scale;
          myRgbd << " ";
          myRgbd << xyzPose[1];// * scale;
          myRgbd << " ";
          myRgbd << xyzPose[2];// * scale;
          myRgbd << " ";
          myRgbd << mpMap[idMap]->vpKeyFrames[i]->rgbdData[indexP].r;
          myRgbd << " ";
          myRgbd << mpMap[idMap]->vpKeyFrames[i]->rgbdData[indexP].g;
          myRgbd << " ";
          myRgbd << mpMap[idMap]->vpKeyFrames[i]->rgbdData[indexP].b;
          myRgbd << " ";
          myRgbd << 0;
          myRgbd << "\n"; 
 //         numPoints++;
        }
        indexP++;
      }
    }
  }
  }


  myDensePoints.close();
  myKeyFrames.close();
  myRgbd.close();













  //OLD SAVEMAP
  command_compress = " cd " + path_save_maps + "; tar -czf " + path_save_maps + "/" + req.name + ".tar.gz " + req.name + "_ctam_map ";
  // ROBOEARTH DATABASE NAME
  //command_compress = " cd " + path_save_maps + "; tar -czf " + path_save_maps + "/" + "vslam_map.tar.gz " + "map" + "." + req.name + "_ctam_map ";
  
  FILE * find_5_file = popen(command_compress.c_str(), "r");
  char command_5_compress[1000];
  int numChar_5 = fread(command_5_compress, 1, 1000, find_5_file);
  if (numChar_5 == 0 ){
    ROS_INFO("Tar.gz and move directory");
    //OLD SAVEMAP 
    command_remove = "rm -r " + path_save_maps + "/" + req.name + "_ctam_map ";
    //ROBOEARTH_SAVE 
    //command_remove = "rm -r " + path_save_maps + "/" + "map" + "." + req.name + "_ctam_map ";
    FILE * find_6_file = popen(command_remove.c_str(), "r");
    char command_6_move[1000];
    int numChar_6 = fread(command_6_move, 1, 1000, find_6_file);
    if (numChar_6 == 0 ){
      ROS_INFO("Remove tmp directory");
    }
  }

//  std::string name_map;  
//  std::string name_id;
//  name_map = path_save_maps + "/vslam_map.tar.gz";


//  ifstream fl(name_map.c_str());  
//  fl.seekg( 0, ios::end );  
//  size_t len = fl.tellg();  
//  char *ret = new char[len];  
//  fl.seekg(0, ios::beg);   
//  fl.read(ret, len);  
//  fl.close();  

//02072015
//  re_msgs::File fileMsg;


//  name_id = req.name;
//  fileMsg.name = name_id.c_str();
//  fileMsg.data.resize(len);
//  std::copy(ret, ret+len, fileMsg.data.begin()); 

//02072015
//  res.vmap = fileMsg;

//////////////////////////////


// DELETE LOCAL MAP FILE TAR.GZ
/*
  command_delete = "rm " + path_save_maps + "/vslam_map.tar.gz";

  FILE * find_7_file = popen(command_delete.c_str(), "r");
  char command_7_find[1000];
  int numChar_7 = fread(command_7_find, 1, 1000, find_7_file);
  if (numChar_7 == 0 ){
    ROS_INFO("File removed");
  }
*/
  

  return true;

}


void System::addKeyFrameCallback(const c2tam_msgs::AddKeyFrame::ConstPtr& msg)
{
  double secs =ros::Time::now().toSec();

  bool sendRgb;

  int img_height;
  int img_width;

  double scaleMonocular = 0.0;
  std::vector<int> indexActual;
  std::vector<int> indexOld;
 
  int recoveryNumMap = -1;
  SE3 <> recoveryPose;


 // mpSearchObjects->NewFrameSearchObjects(msg->KeyFrame.img,mpMap[0]->vpKeyFrames.size());
 // mpSearchObjects->NewFrameSearchObjects(0);

  if(mpMapMaker[msg->id]->mapMode==2){
    recoveryNumMap = RecoveryMultiMap(msg->id,msg->KeyFrame.img,recoveryPose);
  }

  sKeyFrame kf;
  kf.sMeasurements.clear(); 
  kf.sCandidates.clear();
  kf.rgbdData.clear();
  IplImage *im;
  SE3 <> s;
  boost::shared_ptr<const sensor_msgs::Image> *ros_img_ptr;
  boost::shared_ptr<const sensor_msgs::Image> *ros_img_depth_ptr;
  boost::shared_ptr<const sensor_msgs::Image> *ros_img_color_ptr;

  if(!msg->type){
    ros_img_ptr = new boost::shared_ptr<const sensor_msgs::Image>(new sensor_msgs::Image(msg->KeyFrame.img));

    // ros converts the image
   im = bridge_.imgMsgToCv(*ros_img_ptr, "mono8");
   //cvSaveImage("/home/roboearth/Escritorio/blabla.png",im);

    s.get_translation()[0] = msg->KeyFrame.se3CfromW[0];
    s.get_translation()[1] = msg->KeyFrame.se3CfromW[1];
    s.get_translation()[2] = msg->KeyFrame.se3CfromW[2];

    Matrix< 3 > mAux;

    mAux(0,0) = msg->KeyFrame.se3CfromW[3];
    mAux(0,1) = msg->KeyFrame.se3CfromW[4];
    mAux(0,2) = msg->KeyFrame.se3CfromW[5];

    mAux(1,0) = msg->KeyFrame.se3CfromW[6];
    mAux(1,1) = msg->KeyFrame.se3CfromW[7];
    mAux(1,2) = msg->KeyFrame.se3CfromW[8];

    mAux(2,0) = msg->KeyFrame.se3CfromW[9];
    mAux(2,1) = msg->KeyFrame.se3CfromW[10];
    mAux(2,2) = msg->KeyFrame.se3CfromW[11];

    s.get_rotation() = mAux;

    mpMap[msg->id]->denseScale = msg->mapDenseScale;

  }else {

   // boost::shared_ptr<const sensor_msgs::Image> ros_img_ptr_2(new sensor_msgs::Image(msg->SemanticKeyFrame.img));
   ros_img_ptr = new boost::shared_ptr<const sensor_msgs::Image>(new sensor_msgs::Image(msg->SemanticKeyFrame.img));

    // ros converts the image
   im = bridge_.imgMsgToCv(*ros_img_ptr, "mono8");



    s.get_translation()[0] = msg->SemanticKeyFrame.se3CfromW[0];
    s.get_translation()[1] = msg->SemanticKeyFrame.se3CfromW[1];
    s.get_translation()[2] = msg->SemanticKeyFrame.se3CfromW[2];

    Matrix< 3 > mAux;

    mAux(0,0) = msg->SemanticKeyFrame.se3CfromW[3];
    mAux(0,1) = msg->SemanticKeyFrame.se3CfromW[4];
    mAux(0,2) = msg->SemanticKeyFrame.se3CfromW[5];

    mAux(1,0) = msg->SemanticKeyFrame.se3CfromW[6];
    mAux(1,1) = msg->SemanticKeyFrame.se3CfromW[7];
    mAux(1,2) = msg->SemanticKeyFrame.se3CfromW[8];

    mAux(2,0) = msg->SemanticKeyFrame.se3CfromW[9];
    mAux(2,1) = msg->SemanticKeyFrame.se3CfromW[10];
    mAux(2,2) = msg->SemanticKeyFrame.se3CfromW[11];

    s.get_rotation() = mAux;

  }
  ImageRef imSize(im->width,im->height);
  Image<CVD::byte> imFrame(imSize);
  memcpy(imFrame.data(),im->imageData,im->width*im->height*sizeof(uchar));

  if(!msg->type){
    kf.imFrame = imFrame;
    kf.se3CfromW = s;
    kf.bFixed = msg->KeyFrame.bFixed;
    kf.bSemantic = false;
    kf.dSceneDepthMean = msg->KeyFrame.dSceneDepthMean;  
    kf.dSceneDepthSigma = msg->KeyFrame.dSceneDepthSigma; 
    vector<c2tam_msgs::Measurement>::const_iterator it;
    for(it = msg->KeyFrame.sMeasurements.begin(); it != msg->KeyFrame.sMeasurements.end(); ++it){

      Measurement m;
      sMeasurement sm; //Roboearth
      m.v2RootPos[0] = (*it).v2RootPos[0];
      m.v2RootPos[1] = (*it).v2RootPos[1];
      m.nLevel = (*it).nLevel;
      m.bSubPix = (*it).bSubPix;
 
      sm.m = m; //Roboearth
      sm.indexPointMap =  (*it).indexPointMap; //Roboearth
      kf.sMeasurements.push_back(sm); //Roboearth
    }

    if (msg->KeyFrame.imgColor.data.size() == 0)
      sendRgb = false;
    else
      sendRgb = true;

    ros_img_depth_ptr = new boost::shared_ptr<const sensor_msgs::Image>(new sensor_msgs::Image(msg->KeyFrame.imgDepth));    
   if (sendRgb)
     ros_img_color_ptr = new boost::shared_ptr<const sensor_msgs::Image>(new sensor_msgs::Image(msg->KeyFrame.imgColor));    


    unsigned color_step, color_skip;

    int pixel_data_size = 3;
    char red_idx = 0, green_idx = 1, blue_idx = 2;
    if (sendRgb){
      if(msg->KeyFrame.imgColor.encoding.compare("mono8") == 0) pixel_data_size = 1;
      if(msg->KeyFrame.imgColor.encoding.compare("bgr8") == 0) { red_idx = 2; blue_idx = 0; }
//    ROS_ERROR_COND(pixel_data_size == 0, "Unknown image encoding: %s!", msg->KeyFrame.imgColor.encoding.c_str());
      color_step = pixel_data_size * msg->KeyFrame.imgColor.width / msg->KeyFrame.imgDepth.width;
      color_skip = pixel_data_size * (msg->KeyFrame.imgColor.height / msg->KeyFrame.imgDepth.height - 1) * msg->KeyFrame.imgColor.width;
    }
    else{
      red_idx = 0;
      green_idx = 0;
      blue_idx = 0;
    }

    const float* depth_buffer = reinterpret_cast<const float*>(&msg->KeyFrame.imgDepth.data[0]);

    const uint8_t* img_buffer;
    if (sendRgb){
      img_buffer = &msg->KeyFrame.imgColor.data[0];
      img_height = (int)msg->KeyFrame.imgColor.height;
      img_width = (int)msg->KeyFrame.imgColor.width;
    }
    else{
      img_buffer = &msg->KeyFrame.img.data[0];
      img_height = (int)msg->KeyFrame.imgDepth.height;
      img_width = (int)msg->KeyFrame.imgDepth.width;
    }

  

    // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
    int color_idx = 0, depth_idx = 0;
    double depth_scaling = 1;// = ParameterServer::instance()->get<double>("depth_scaling_factor");
    int img_idx;
    for (int v = 0; v < img_height; ++v, color_idx += color_skip)
    {
      for (int u = 0; u < img_width; ++u, color_idx += color_step, ++depth_idx)
      {
        c2tam_msgs::RgbdInfo rgbdI;
        float Z = depth_buffer[depth_idx] * depth_scaling;
        if (std::isnan (Z))
        {
         rgbdI.z = Z;
        }
        else // Fill in XYZ
        {
          rgbdI.z = Z;
        }
        // Fill in color

        if(sendRgb) 
          img_idx = color_idx;
        else
          img_idx = depth_idx;


        if(pixel_data_size == 3){

          rgbdI.r =  img_buffer[img_idx + red_idx];// rgb_buffer[color_idx + red_idx];
          rgbdI.g = img_buffer[img_idx + green_idx];//rgb_buffer[color_idx + green_idx];
          rgbdI.b  = img_buffer[img_idx + blue_idx];//rgb_buffer[color_idx + blue_idx];
        } else {

          rgbdI.r = rgbdI.g = rgbdI.b = img_buffer[img_idx];//rgb_buffer[color_idx];
        }
        rgbd denseData;
        denseData.r = rgbdI.r;
        denseData.g = rgbdI.g;
        denseData.b = rgbdI.b;
        denseData.z = rgbdI.z;
        denseData.x = (double) ((double) (((double) u - mpMapMaker[msg->id]->cam_cx) * rgbdI.z) / (double) mpMapMaker[msg->id]->cam_fx);
        denseData.y = (double) ((double) (((double) v - mpMapMaker[msg->id]->cam_cy) * rgbdI.z) / (double) mpMapMaker[msg->id]->cam_fy);

        kf.rgbdData.push_back(denseData); //Roboearth
      }
    }
  } else {
    kf.name = msg->SemanticKeyFrame.name;
    kf.depth = msg->SemanticKeyFrame.depth;
    ROS_INFO("kf.depth %f",kf.depth);

    kf.imFrame = imFrame;
    kf.se3CfromW = s;
    kf.vp2DPoints.clear();
    kf.vp3DPoints.clear();
    kf.bFixed = msg->SemanticKeyFrame.bFixed;
    kf.bSemantic = true; //msg->SemanticKeyFrame.bSemantic;

    kf.dSceneDepthMean = msg->SemanticKeyFrame.dSceneDepthMean;  
    kf.dSceneDepthSigma = msg->SemanticKeyFrame.dSceneDepthSigma; 

    sCandidate vcand;
    vector<c2tam_msgs::Surf>::const_iterator it;
    for(int indexVsurf = 0; indexVsurf < msg->SemanticKeyFrame.vSurfOD.size(); indexVsurf++){
      for(it = msg->SemanticKeyFrame.vSurfOD[indexVsurf].VectS.begin(); it != msg->SemanticKeyFrame.vSurfOD [indexVsurf].VectS.end(); ++it){
        Surf cand;
        cand.irLevelPos[0] = (*it).irLevelPos[0];
        cand.irLevelPos[1] = (*it).irLevelPos[1];
        cand.size = (*it).size;
        cand.index = (*it).index;
        vcand.vCandidatesOD.push_back(cand);
      }
      kf.sCandidates.push_back(vcand); //Roboearth
    }
    vector<c2tam_msgs::RgbdInfo>::const_iterator itDense2;
   int u2 = 0;
   int v2 = 0;
    for(itDense2 = msg->SemanticKeyFrame.rgbd.begin(); itDense2 != msg->SemanticKeyFrame.rgbd.end(); ++itDense2){
      if (v2 == 640){
        u2++;
        v2 = 0;
      }
      rgbd denseData;
      denseData.r = (*itDense2).r;
      denseData.g = (*itDense2).g;
      denseData.b = (*itDense2).b;
      denseData.z = (*itDense2).z;
      denseData.x = (double) ((double) (((double) v2 - mpMapMaker[msg->id]->cam_cx) * denseData.z) / (double) mpMapMaker[msg->id]->cam_fx);
      denseData.y = (double) ((double) (((double) u2 - mpMapMaker[msg->id]->cam_cy) * denseData.z) / (double) mpMapMaker[msg->id]->cam_fy);

      kf.rgbdData.push_back(denseData); //Roboearth
      v2++;
    }

    kf.Tco.get_translation()[0] = msg->SemanticKeyFrame.tco[0];
    kf.Tco.get_translation()[1] = msg->SemanticKeyFrame.tco[1];
    kf.Tco.get_translation()[2] = msg->SemanticKeyFrame.tco[2];

    Matrix< 3 > mAux2;

    mAux2(0,0) = msg->SemanticKeyFrame.tco[3];
    mAux2(0,1) = msg->SemanticKeyFrame.tco[4];
    mAux2(0,2) = msg->SemanticKeyFrame.tco[5];

    mAux2(1,0) = msg->SemanticKeyFrame.tco[6];
    mAux2(1,1) = msg->SemanticKeyFrame.tco[7];
    mAux2(1,2) = msg->SemanticKeyFrame.tco[8];

    mAux2(2,0) = msg->SemanticKeyFrame.tco[9];
    mAux2(2,1) = msg->SemanticKeyFrame.tco[10];
    mAux2(2,2) = msg->SemanticKeyFrame.tco[11];

    kf.Tco.get_rotation() = mAux2;

    int l = 0;
    while( l< msg->SemanticKeyFrame.p2d.size()){
      Vector<2> vAux2;
      vAux2[0] = msg->SemanticKeyFrame.p2d[l];
      l++;
      vAux2[1] = msg->SemanticKeyFrame.p2d[l];
      l++;
      kf.vp2DPoints.push_back(vAux2);
    }

    l = 0;
    while( l< msg->SemanticKeyFrame.p3d.size()){
      Vector<3> vAux3;
      vAux3[0] = msg->SemanticKeyFrame.p3d[l];
      l++;
      vAux3[1] = msg->SemanticKeyFrame.p3d[l];
      l++;
      vAux3[2] = msg->SemanticKeyFrame.p3d[l];
      l++;
      kf.vp3DPoints.push_back(vAux3);
    }
  }
  int color = 0;
  if(recoveryNumMap >= 0){
    if (mpMapMaker[msg->id]->vctRecovery[recoveryNumMap].recovery)
     color = 4;
    else
     color = 3;
  }
  mpMapMaker[msg->id]->AddServiceKeyFrame(kf,color);

pthread_mutex_lock(&mpMapMaker[msg->id]->mergeMutex);

  if(recoveryNumMap >= 0 && mpMapMaker[msg->id]->vctRecovery[recoveryNumMap].recovery){
      mpMapMaker[msg->id]->vctRecovery[recoveryNumMap].actualFirstKF = s;
      mpMapMaker[msg->id]->vctRecovery[recoveryNumMap].actualScaleFirstKF = mpMapMaker[recoveryNumMap]->mMap.denseScale;
     
     int TotalIguales = 0;
     std::vector<double> scaleVector;
     scaleVector.clear();
     double oldPoint, newPoint;

     indexActual.clear();
     indexOld.clear();
  
     for(meas_it it = mpMapMaker[recoveryNumMap]->mCurrentKF.mMeasurements.begin(); it!=mpMapMaker[recoveryNumMap]->mCurrentKF.mMeasurements.end(); it++)
     {
       for(meas_it itActual = mpMap[msg->id]->vpKeyFrames[mpMap[msg->id]->vpKeyFrames.size()-1]->mMeasurements.begin(); itActual !=mpMap[msg->id]->vpKeyFrames[mpMap[msg->id]->vpKeyFrames.size()-1]->mMeasurements.end(); itActual ++)
     {
       if( (fabs(it->second.v2RootPos[0] - itActual->second.v2RootPos[0]) <=(double) 2.0) &&
	   (fabs(it->second.v2RootPos[1] - itActual->second.v2RootPos[1]) <=(double) 2.0) && 
		(it->first->nSourceLevel == itActual->first->nSourceLevel)	){

          Vector<3> PointRelativeOld = recoveryPose * it->first->v3WorldPos;
          Vector<3> PointRelativeNew = s * itActual->first->v3WorldPos;

          oldPoint = sqrt((PointRelativeOld[0] * PointRelativeOld[0]) + 
				(PointRelativeOld[1] * PointRelativeOld[1]) + 
 				(PointRelativeOld[2] * PointRelativeOld[2]));
 
          newPoint = sqrt((PointRelativeNew[0] * PointRelativeNew[0]) + 
				(PointRelativeNew[1] * PointRelativeNew[1]) + 
 				(PointRelativeNew[2] * PointRelativeNew[2]));


          indexOld.push_back(it->first->indexPointMap);
          indexActual.push_back(itActual->first->indexPointMap);

          scaleVector.push_back(newPoint / oldPoint);

         TotalIguales++;
        } 
       }
     }

     if (scaleVector.size() > 0){
      sort(scaleVector.begin(),scaleVector.end());
      scaleMonocular = scaleVector[scaleVector.size()/2];
    }

    cerr << recoveryPose << endl;


     mpMapMaker[recoveryNumMap]->ApplyExportMap(recoveryPose, s, mpMap[msg->id]->denseScale,scaleMonocular);

     pthread_mutex_unlock(&mpMapMaker[msg->id]->mergeMutex);

     mpMapMaker[msg->id]->MergeMaps(mpMapMaker[recoveryNumMap]->mMap, msg->KeyFrame.dSceneDepthMean,msg->KeyFrame.dSceneDepthSigma,
					indexOld,indexActual);

  }
  else{
    pthread_mutex_unlock(&mpMapMaker[msg->id]->mergeMutex);

}

}



double System::KeyFrameLinearDist(SE3<> k1,SE3<> k2)
{
  Vector<3> v3KF1_CamPos = k1.inverse().get_translation();
  Vector<3> v3KF2_CamPos = k2.inverse().get_translation();
  Vector<3> v3Diff = v3KF2_CamPos - v3KF1_CamPos;
  double dDist = sqrt(v3Diff * v3Diff);
  return dDist;
}



bool System::GetId(c2tam_srvs::GetId::Request  &req, c2tam_srvs::GetId::Response &res ){

  res.id = mpMap.size();
  Map * mapPTAM = new Map;

  MapMaker * mapMakerPTAM = new MapMaker(*mapPTAM, nh_,res.id,req.num,req.cx,req.cy,req.fx,req.fy, true);
 
  mpMap.push_back(mapPTAM);
  mpMapMaker.push_back(mapMakerPTAM);

  return true;
}



bool System::FreeId(c2tam_srvs::GetId::Request  &req, c2tam_srvs::GetId::Response &res )
{
  res.id = mpMap.size();
  return true;
}

bool System::printInfoCallback(c2tam_srvs::LoadIdMap::Request  &req, c2tam_srvs::LoadIdMap::Response &res ){

  mpMapMaker[req.id]->printInfo();

}


void System::Run()
{

  ROS_INFO("<ctam_MAPPING> Initialize service...");

  request_reset_service_ = nh_.advertiseService("/reset_service", &System::RequestResetCb, this);
  reset_done_service_ = nh_.advertiseService("/reset_done", &System::ResetDoneCb, this);
  init_from_stereo_ekf_service_ = nh_.advertiseService("/init_from_stereo_ekf_service", &System::InitFromStereoEkfCb, this);
  init_from_rgbd_service_ = nh_.advertiseService("/init_from_rgbd_service", &System::InitFromRGBDCb, this);


  attempt_recovery_ = nh_.advertiseService("/attempt_recovery_service", &System::AttemptRecoveryCb, this);
  save_map_ = nh_.advertiseService("/save_map_service", &System::SaveMapCb, this);
  generate_ply_ = nh_.advertiseService("/generate_ply_service", &System::GeneratePlyCb, this);
  get_octomap_ = nh_.advertiseService("/get_octomap_service", &System::GetOctomapCb, this);
  load_map_ = nh_.advertiseService("/load_map_service", &System::LoadMapCb, this);
  load_id_map_ = nh_.advertiseService("/load_id_map", &System::LoadIdCb, this);
  info_kf_sub_ = nh_.advertiseService("/vslam/printInfo", &System::printInfoCallback, this);
  search_for_map_ = nh_.advertiseService("/search_for_map", &System::SearchForMapCb, this);
  add_key_frame_sub_ = nh_.subscribe("/addKeyFrame", 1000, &System::addKeyFrameCallback, this);
  get_id_ = nh_.advertiseService("/get_id_service", &System::GetId, this);
  free_id_ = nh_.advertiseService("/vslam/free_id_service", &System::FreeId, this);

  pcl_pub = nh_.advertise<sensor_msgs::PointCloud2>("vslam/rgb/points", 100);

  create_octo_pub = nh_.advertise<c2tam_msgs::CreateOctomap>("/ctam_octomap/createOctomap", 100);


  nh_.param("C2TAMmapping/camera_cx", cam_cx, 311.592488);  //Kinect Default calibration
  nh_.param("C2TAMmapping/camera_cy", cam_cy, 250.148823);
  nh_.param("C2TAMmapping/camera_fx", cam_fx, 525.776310);
  nh_.param("C2TAMmapping/camera_fy", cam_fy, 526.555874);



  ros::MultiThreadedSpinner spinner(4);

  ROS_INFO("<ctam_MAPPING> Ready...");

  while(ros::ok())
  {
    spinner.spin();
  }
}


