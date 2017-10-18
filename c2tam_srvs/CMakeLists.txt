cmake_minimum_required(VERSION 2.8.3)
project(c2tam_srvs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs 
  std_msgs
  c2tam_msgs
  message_generation
)

add_service_files(
  FILES
  AttemptRecovery.srv    
  LoadDrawingModel.srv  
  RunVslam.srv
  GetId.srv              
  LoadIdMap.srv         
  SaveEnvironment.srv
  GetMapObjects.srv      
  LoadMap.srv           
  SaveMap.srv
  GetOctomap.srv         
  LoadVslamMap.srv      
  SearchForMap.srv
  InitFromRGBD.srv       
  QueueSize.srv         
  SetMetaObjects.srv
  InitFromStereoEKF.srv  
  RequestReset.srv      
  SetObjects.srv
)

generate_messages(
  DEPENDENCIES 
  roscpp
  sensor_msgs 
  std_msgs
  c2tam_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs c2tam_msgs message_runtime 
)

