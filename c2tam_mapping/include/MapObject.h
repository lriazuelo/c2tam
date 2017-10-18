
#ifndef __MAPOBJECT_H
#define __MAPOBJECT_H
#include <TooN/se3.h>
#include "MapPoint.h"
#include "OpenGL.h"
#include "KeyFrame.h"

#include <ros/ros.h>


struct STransform
{
    TooN::Matrix<4> m_;
    TooN::Matrix<4> c_;
	double scale_o;
	double x,y,z;
};

struct faceTexture
  {
    GLuint id;
    GLenum  format;
    GLsizei width;
    GLsizei height;
    GLint   internalFormat;
    GLubyte *texels;
  };

struct facesType
{
    int id;
    int index;
    double width;
    double height;
    double oTf[16];
    faceTexture textureReal;
    faceTexture textureVirtual;
};

struct infoPoints
{
    int label; //label from MonoSLAM
    double pointX;
    double pointY;
    double pointZ;

};

struct modelType
{
    std::string name;
    bool active;
    int num_faces;
    double width;
    double height;
    double depth;
    std::vector<facesType> faces;
    STransform transform;
    std::vector<infoPoints> points;
    SVertex *model3d;
    int model3dNumPoints;
    std::string type;
};

struct point2d
{
    Vector<2> v2RootPos;
    CVD::ImageRef irLevelPos;
    int octave;
};

struct MapObject
{
  MapObject(){
      vp2DPoints.clear();
      vp3DPoints.clear();
      mMapSurfPointPairs.clear();
      relativeScale=1.0;
  }

  //Transformation from Object frame to Camera frame up to scale
  SE3<> Tco;
  // Object pose in the world frame
  SE3<> Tow;
  //2D points returned by Object Detector
  std::vector<Surf> vp2DPoints;
  //3D points returned by Object Detector in the model reference
  std::vector< Vector<3> > vp3DPoints;
  //Name
  std::string Name;

  double Depth;
  //relative scale w.r.t. map
  double relativeScale;
  //The keyFrame which the object was fired
  KeyFrame *semanticKF;

  //3D points candidates to be added to the map in the world reference
  std::map<MapPoint*, Vector<3> > mMapSurfPointPairs;

  //Drawing information
  modelType infoModels;



  //Time first detection
  ros::Time timeDetection;

  //float depth;

};

#endif
