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

#include "Visualizer.h"

#include<iostream>
#include<fstream>

using namespace CVD;
using namespace std;
using namespace TooN;
using namespace GVars3;

Visualizer::Visualizer()
{

  imageSize = new CVD::ImageRef(640,480);

  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("KeyPress", GUICommandCallBack, this);

  mGLWindowVirtual = new GLWindow2(*imageSize, "CTAM: Virtual World");


  map_info_sub = nh_.subscribe("/c2tam_mapping/map_info", 10, &Visualizer::mapCb, this);
  dense_info_sub = nh_.subscribe("/c2tam_mapping/dense_info", 1, &Visualizer::denseCb, this);
  pose_sub = nh_.subscribe("/vslam/vslam_pose", 1, &Visualizer::poseCb, this);
  obj_trans_sub = nh_.subscribe("/vslam/obj_visualizer", 1, &Visualizer::objTransCb, this);

  load_drawing_model_ = nh_.advertiseService("/ctam_visualizer/load_drawing_model", &Visualizer::LoadDrawingModel, this);

  nh_.param("C2TAMvisualizer/visualizer", node_visualizer, 1);

  nh_.param("C2TAMvisualizer/camera_cx", cam_cx, 311.592488);  //Kinect Default calibration
  nh_.param("C2TAMvisualizer/camera_cy", cam_cy, 250.148823);
  nh_.param("C2TAMvisualizer/camera_fx", cam_fx, 525.776310);
  nh_.param("C2TAMvisualizer/camera_fy", cam_fy, 526.555874);

//  timerWait = new ros::Timer(nh_.createTimer(ros::Duration(1.0), &Visualizer::waitCallback, this));

  mpMap = new Map;

  mpMapViewer = new MapViewer(*mpMap); 

  mapMutex = PTHREAD_MUTEX_INITIALIZER;

  should_continue = true;
  drawVirtualDense = false;

  se3CamFromW.get_translation()[0] = 0;
  se3CamFromW.get_translation()[1] = 0;
  se3CamFromW.get_translation()[2] = 0;

  Matrix< 3 > mAux_res;

  mAux_res(0,0) = 0;
  mAux_res(0,1) = 0;
  mAux_res(0,2) = 0;

  mAux_res(1,0) = 0;
  mAux_res(1,1) = 0;
  mAux_res(1,2) = 0;

  mAux_res(2,0) = 0;
  mAux_res(2,1) = 0;
  mAux_res(2,2) = 0;

  se3CamFromW.get_rotation() = mAux_res;


  if (node_visualizer == 1)
    ROS_INFO("<CTAM_visualizer> Ready Mapping visualizer...");
  else{
    if(node_visualizer == 0)
    ROS_INFO("<CTAM_visualizer> Ready Tracking visualizer...");
  }
};


void Visualizer::waitCallback(const ros::TimerEvent&)
{
  ROS_INFO(".");
}

void Visualizer::poseCb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
  Vector <3> q;
  q[0] = pose_msg->pose.orientation.x;
  q[1] = pose_msg->pose.orientation.y;
  q[2] = pose_msg->pose.orientation.z;

  double theta = 2 * acos(pose_msg->pose.orientation.w);

  if(!(std::isnan(q[0]/theta) || std::isnan(q[1]/theta) || std::isnan(q[2]/theta)))
    normalize(q);

  SO3<> rot_q;
  se3CamFromW.get_rotation() = rot_q.exp(theta * q);

  se3CamFromW.get_translation()[0] = pose_msg->pose.position.x;// * mpMap->denseScale;
  se3CamFromW.get_translation()[1] = pose_msg->pose.position.y;// * mpMap->denseScale;
  se3CamFromW.get_translation()[2] = pose_msg->pose.position.z;// * mpMap->denseScale;

}

void Visualizer::denseCb(const c2tam_msgs::DenseInfo::ConstPtr& dense_msg)
{
  if (node_visualizer == 0){

    float cx = cam_cx;
    float cy = cam_cy;
    float fx = cam_fx;
    float fy = cam_fy;

    for (int i=0; i < dense_msg->denseKF.size(); i++){
      //fprintf(stderr,"dense_msg->denseKF\n");
      if (dense_msg->denseKF[i].denseCloud.size() > 0){
        SVertex *sv = new SVertex[640*480];
        mpMap->enableDense = true;
        int indexImage = 0;
        for (int u=0; u< 480; u++){
          for (int v=0; v< 640; v++){
            indexImage = (u*640) + v;
            sv[indexImage].z = dense_msg->denseKF[i].denseCloud[indexImage].z;
            sv[indexImage].x = (double) ((double) (((double) v - cx) * dense_msg->denseKF[i].denseCloud[indexImage].z) / (double) fx);
            sv[indexImage].y = (double) ((double) (((double) u - cy) * dense_msg->denseKF[i].denseCloud[indexImage].z) / (double) fy);
            sv[indexImage].r = (double) dense_msg->denseKF[i].denseCloud[indexImage].r /(double) 255;
            sv[indexImage].g = (double) dense_msg->denseKF[i].denseCloud[indexImage].g /(double) 255;
            sv[indexImage].b = (double) dense_msg->denseKF[i].denseCloud[indexImage].b /(double) 255;
          }
        }
        mpMap->densePoints.push_back(sv);
        mpMap->denseID.push_back(dense_msg->denseKF[i].idKf);
      }
    }
  }
}

void Visualizer::objTransCb(const c2tam_msgs::ObjectTransformArray::ConstPtr& obj_msg)
{
  SE3 <> s;
  SE3 <> s2;
  double scale;
  bool objectLoaded = false;
  int indexObj = 0;
  bool objSelected;


  for (unsigned int j=0; j < obj_msg->object.size(); j++){
     objectLoaded = false;
    objSelected = false;
    indexObj = 0;
    for (unsigned int i=0; i<mpMap->vpObjects.size(); i++){
      if(strcmp(mpMap->vpObjects[i]->Name.c_str(),obj_msg->object[j].name.c_str()) == 0){
         indexObj = i;
         objSelected = true;
         if(mpMap->vpObjects[i]->find) 
           objectLoaded = true;
      }
    }
  
    if (objSelected){

      s.get_translation()[0] = obj_msg->object[j].data[0];
      s.get_translation()[1] = obj_msg->object[j].data[1];
      s.get_translation()[2] = obj_msg->object[j].data[2];

      Matrix< 3 > mAux;

      mAux(0,0) = obj_msg->object[j].data[3];
      mAux(0,1) = obj_msg->object[j].data[4];
      mAux(0,2) = obj_msg->object[j].data[5];

      mAux(1,0) = obj_msg->object[j].data[6];
      mAux(1,1) = obj_msg->object[j].data[7];
      mAux(1,2) = obj_msg->object[j].data[8];

      mAux(2,0) = obj_msg->object[j].data[9];
      mAux(2,1) = obj_msg->object[j].data[10];
      mAux(2,2) = obj_msg->object[j].data[11];
 
      s.get_rotation() = mAux;

      scale = obj_msg->object[j].data[12]* obj_msg->object[j].data[13];

      if(objectLoaded){
        mpMap->vpObjects[indexObj]->se3CfromW = s;
        mpMap->vpObjects[indexObj]->scale = scale;
      }
      else{
        mpMap->vpObjects[indexObj]->se3CfromW = s;
        mpMap->vpObjects[indexObj]->scale = scale;
        mpMap->vpObjects[indexObj]->find = true;
      }
    }
  }
}

bool Visualizer::LoadDrawingModel(c2tam_srvs::LoadDrawingModel::Request  &req,
		           c2tam_srvs::LoadDrawingModel::Response &res )
{
  std::string path_directory;
  std::string path_node;
  std::string path_model;

  FILE * find_file = popen("rospack find ctam_visualizer", "r");
  char command_find[1000];
  int numChar = fread(command_find, 1, 1000, find_file);
  command_find[numChar-1] = 0;

  path_node = command_find;
  path_directory = path_node + "/models/";

  path_model = path_directory + req.file.name  + ".ply";;

  char *ret = new char[req.file.data.size()]; 

  ofstream ofs (path_model.c_str(),ofstream::binary);

  std::copy(req.file.data.begin(), req.file.data.end(), ret); 

  ofs.write (ret,req.file.data.size());
  ofs.close();


    //fprintf(stderr,"Load Model\n");
    MapObject *mo = new MapObject();
    mo->Name = req.file.name;
    mo->find = false;

    FILE *fileModel;
    GLfloat auxNx;
    GLfloat auxNy;
    GLfloat auxNz;

    char  s1[30];
    char  s2[30];
    char  s3[30];
    float  f1;

    int numPoints;

    path_model = path_directory + req.file.name + ".ply";

    fileModel = fopen(path_model.c_str(), "r");
    if (fileModel == NULL)
    {
        //fprintf(stderr,"\ncan't open input file\n");
    }
    else {
        //fprintf(stderr,"\nFile opened\n");
    }

    if (!feof(fileModel)){
      fscanf(fileModel,"%s\n", s1);
      fscanf(fileModel,"%s %s %f\n", s1,  s2, &f1);
      fscanf(fileModel,"%s %s %d\n", s1,  s2, &numPoints);
      fscanf(fileModel,"%s %s %s\n", s1,  s2, s3);
      fscanf(fileModel,"%s %s %s\n", s1,  s2, s3);
      fscanf(fileModel,"%s %s %s\n", s1,  s2, s3);
      fscanf(fileModel,"%s %s %s\n", s1,  s2, s3);
      fscanf(fileModel,"%s %s %s\n", s1,  s2, s3);
      fscanf(fileModel,"%s %s %s\n", s1,  s2, s3);
      fscanf(fileModel,"%s %s %s\n", s1,  s2, s3);
      fscanf(fileModel,"%s %s %s\n", s1,  s2, s3);
      fscanf(fileModel,"%s %s %s\n", s1,  s2, s3);
      fscanf(fileModel,"%s\n", s1);
    }
    mo->infoModels.model3d = new SVertex[numPoints];
    mo->infoModels.model3dNumPoints = numPoints;

    int numTotal = 0;
    if (!feof(fileModel))
      for (int loopcounter = 0 ; loopcounter < numPoints ; loopcounter++ )
      {
        fscanf(fileModel,"%f %f %f %f %f %f %f %f %f\n", &mo->infoModels.model3d[numTotal].x, &mo->infoModels.model3d[numTotal].y, &mo->infoModels.model3d[numTotal].z,
			 &auxNx, &auxNy, &auxNz,
             &mo->infoModels.model3d[numTotal].r, &mo->infoModels.model3d[numTotal].g, &mo->infoModels.model3d[numTotal].b);
        mo->infoModels.model3d[numTotal].r = mo->infoModels.model3d[numTotal].r / 255;
        mo->infoModels.model3d[numTotal].g = mo->infoModels.model3d[numTotal].g / 255;
	  mo->infoModels.model3d[numTotal].b = mo->infoModels.model3d[numTotal].b / 255;
        numTotal++;
      }

      mo->depth = req.depth;
      mo->width = req.width;
      mo->height = req.height;

      mpMap->vpObjects.push_back(mo);

  res.success = true;
  return true;
}


void Visualizer::mapCb(const c2tam_msgs::MapInfo::ConstPtr& map_msg)
{

  float cx = cam_cx; 
  float cy = cam_cy;
  float fx = cam_fx;
  float fy = cam_fy;



  pthread_mutex_lock(&mapMutex); 

  mpMap->vpPoints.clear();
  mpMap->vpKeyFrames.clear();

  for (int i=0; i < map_msg->Pointv3WorldPos.size() / 3; i++){

    MapPoint *p = new MapPoint;
    p->nSourceLevel  = map_msg->nSourceLevel[i]; 
    p->v3WorldPos[0] = map_msg->Pointv3WorldPos[3*i];
    p->v3WorldPos[1] = map_msg->Pointv3WorldPos[(3*i)+1];
    p->v3WorldPos[2] = map_msg->Pointv3WorldPos[(3*i)+2];

    mpMap->vpPoints.push_back(p);

  }

  for (int i=0; i < map_msg->KFse3CfromW.size() / 12; i++){

    KeyFrame *k = new KeyFrame();

    k->se3CfromW.get_translation()[0] = map_msg->KFse3CfromW[12*i];
    k->se3CfromW.get_translation()[1] = map_msg->KFse3CfromW[(12*i)+1];
    k->se3CfromW.get_translation()[2] = map_msg->KFse3CfromW[(12*i)+2];

    Matrix< 3 > mAux_res;

    mAux_res(0,0) = map_msg->KFse3CfromW[(12*i)+3];
    mAux_res(0,1) = map_msg->KFse3CfromW[(12*i)+4];
    mAux_res(0,2) = map_msg->KFse3CfromW[(12*i)+5];

    mAux_res(1,0) = map_msg->KFse3CfromW[(12*i)+6];
    mAux_res(1,1) = map_msg->KFse3CfromW[(12*i)+7];
    mAux_res(1,2) = map_msg->KFse3CfromW[(12*i)+8];

    mAux_res(2,0) = map_msg->KFse3CfromW[(12*i)+9];
    mAux_res(2,1) = map_msg->KFse3CfromW[(12*i)+10];
    mAux_res(2,2) = map_msg->KFse3CfromW[(12*i)+11];

    k->se3CfromW.get_rotation() = mAux_res;

    mpMap->vpKeyFrames.push_back(k);
  }

  if (node_visualizer == 1){
    for (int i=0; i < map_msg->denseKF.size(); i++){
      if (map_msg->denseKF[i].denseCloud.size() > 0){
        SVertex *sv = new SVertex[640*480];
        mpMap->enableDense = true;
        int indexImage = 0;
        for (int u=0; u< 480; u++){
          for (int v=0; v< 640; v++){
            indexImage = (u*640) + v;
            sv[indexImage].z = map_msg->denseKF[i].denseCloud[indexImage].z;
            sv[indexImage].x = (double) ((double) (((double) v - cx) * map_msg->denseKF[i].denseCloud[indexImage].z) / (double) fx);
            sv[indexImage].y = (double) ((double) (((double) u - cy) * map_msg->denseKF[i].denseCloud[indexImage].z) / (double) fy);
            sv[indexImage].r = (double) map_msg->denseKF[i].denseCloud[indexImage].r /(double) 255;
            sv[indexImage].g = (double) map_msg->denseKF[i].denseCloud[indexImage].g /(double) 255;
            sv[indexImage].b = (double) map_msg->denseKF[i].denseCloud[indexImage].b /(double) 255;
          }
        }
        mpMap->densePoints.push_back(sv);
        mpMap->denseID.push_back(map_msg->denseKF[i].idKf);
      }
    }
  }
  if (map_msg->denseKF.size() > 0)
    mpMap->denseScale = map_msg->scaleDense;
  else
    mpMap->denseScale = 1.0;

  pthread_mutex_unlock(&mapMutex);
}

void Visualizer::GUICommandCallBack(void *ptr, string sCommand, string sParams)
{
  if(sCommand=="quit" || sCommand == "exit")
    static_cast<Visualizer*>(ptr)->should_continue = false;
  if(sCommand=="KeyPress" && sParams == "d"){
    if(static_cast<Visualizer*>(ptr)->drawVirtualDense)
      static_cast<Visualizer*>(ptr)->drawVirtualDense = false;
    else
      static_cast<Visualizer*>(ptr)->drawVirtualDense = true;
  }

}

void Visualizer::drawVirtual(GLWindow2* w)
{
  w->make_current();
  w->SetupViewport();
  w->SetupVideoOrtho();
  w->SetupVideoRasterPosAndZoom();

  mpMapViewer->DrawMap(se3CamFromW, *w, drawVirtualDense);
  w->DrawCaption(mpMapViewer->GetMessageForUser());

  w->swap_buffers();
  w->HandlePendingEvents();
}




void Visualizer::Run()
{
ros::Rate r(5); // 10 hz
while (should_continue)
{
  pthread_mutex_lock(&mapMutex);
  drawVirtual(mGLWindowVirtual);
  pthread_mutex_unlock(&mapMutex);
  ros::spinOnce();
  r.sleep();
}
}


