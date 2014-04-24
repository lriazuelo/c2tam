
#ifndef __MAPOBJECT_H
#define __MAPOBJECT_H
#include <TooN/se3.h>
#include "MapPoint.h"
#include "OpenGL.h"
#include "KeyFrame.h"

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
      vpOD2Points.clear();
      vpOD3Points.clear();
  }

  //Dorian's Transformation
  SE3<> ODTransform;
  //2D points returned by Object Detector
  std::vector<point2d> vpOD2Points;
  //3D points returned by Object Detector in the camera reference
  std::vector< Vector<3> > vpOD3Points;
  //3D points returned by Object Detector in the model reference
  std::vector< Vector<3> > vpOD3PointsModel;
  //Name
  std::string Name;
  // Camera where was detected
  SE3<> se3CfromW;


  //3D points candidates to be added to the map in the world reference
  std::vector<MapPoint*> vpPoints;
  //3D matches in the Object frame
  std::vector< Vector<3> > vpMatches;

  //Drawing information
  modelType infoModels;

  bool find;

  float depth;
  float width;
  float height;

  double scale;
};

#endif
