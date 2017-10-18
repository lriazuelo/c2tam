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
