// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
//
// MapViewer.h
//
// Defines the MapViewer class
//
// This defines a simple map viewer widget, which can draw the
// current map and the camera/keyframe poses within it.
//
#ifndef __MAP_VIEWER_H
#define __MAP_VIEWER_H

#include "Map.h"
#include <TooN/TooN.h>
using namespace TooN;
#include <TooN/se3.h>
#include <sstream>
#include "GLWindow2.h"

class Map;

class MapViewer
{
public:
  MapViewer(Map &map);
  void DrawMap(SE3<> se3CamFromWorld, GLWindow2 &w, bool dense);
  void DrawMapUpdater(SE3<> se3CamFromWorld, GLWindow2 &w, bool dense);
  std::string GetMessageForUser();

protected:
  Map &mMap;

  void DrawGrid();
  void DrawMapDots();
  void DrawMapDotsUpdater();
  void DrawDense(); //DCTAM
  void DrawCamera(SE3<> se3, bool bSmall=false);
  void DrawCameraColor(SE3<> se3CfromW, bool bSmall,int color);
  void SetupFrustum();
  void SetupModelView(SE3<> se3WorldFromCurrent = SE3<>());

  Vector<3> mv3MassCenter;
  SE3<> mse3ViewerFromWorld;

  std::ostringstream mMessageForUser;
};

#endif
