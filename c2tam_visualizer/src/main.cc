#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "Visualizer.h"


#include <GL/glut.h>

using namespace std;
using namespace GVars3;

int main(int argc, char*argv[])
{

  glutInit(&argc, argv);

  ros::init(argc, argv, "C2TAMvisualizer");

  cout << "  C2TAM visualizer " << endl
       << "  --------------- " << endl;

  Visualizer v;
  v.Run();
}
