// Copyright 2008 Isis Innovation Limited
// This is the main extry point for PTAM

#include <ros/ros.h>
#include <stdlib.h>
#include <iostream>
#include <gvars3/instances.h>
#include "System.h"

#include <GL/glut.h>

using namespace std;
using namespace GVars3;

int main(int argc, char*argv[])
{
  glutInit(&argc, argv);
  ros::init(argc, argv, "C2TAMtracking");

  cout << "  C2TAM tracking" << endl
       << "  --------------- " << endl
       << endl
       << "  Parsing settings.cfg ...." << endl;

  std::string path_node;
  std::string path_set;
  std::string path_set_custom;
  std::ofstream set_custom_file;

  FILE * find_file = popen("rospack find c2tam_tracking", "r");
  char command_find[1000];
  int numChar = fread(command_find, 1, 1000, find_file);
  command_find[numChar-1] = 0;
  path_node = command_find;
//  path_set = path_node + "/settings.cfg";

  path_set_custom = path_node + "/settings_custom.cfg";
  set_custom_file.open (path_set_custom.c_str()); 
  set_custom_file.precision (15);
  set_custom_file << "exec ";
  set_custom_file << path_node.c_str();
  set_custom_file << "/camera.cfg";
  set_custom_file << "\n"; 
  set_custom_file.close();

  GUI.LoadFile(path_set_custom.c_str());
//  GUI.LoadFile(path_set.c_str());

  GUI.StartParserThread(); // Start parsing of the console input
  atexit(GUI.StopParserThread);

  try
  {
     #ifdef _SEQUENCE_
      System s(std::string(argv[1]), 25);
     #else
      System s;
     #endif
      s.Run();
  }
  catch(CVD::Exceptions::All e)
  {
      cout << endl
           << "!! Failed to run system; got exception. " << endl
           << "   Exception was: " << endl
           << e.what << endl;
  }
}


