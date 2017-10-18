cmake_minimum_required(VERSION 2.8.3)
project(c2tam_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs 
  std_msgs
  message_generation
)

add_message_files(
  FILES
  AddKeyFrame.msg       
  LoadPoint.msg            
  Point3d.msg
  Candidate.msg        
  MapInfo.msg              
  Point.msg
  CreateOctomap.msg     
  MapObject.msg             
  PoseKeyFrame.msg
  DataPoints.msg       
  MapPoints.msg           
  RgbdInfo.msg
  DenseInfo.msg         
  MeasPoint.msg           
  SemanticInfo.msg
  DenseKf.msg          
  Measurement.msg          
  SemanticKeyFrame.msg
  File.msg              
  ModPoint.msg            
  Surf.msg
  Index.msg             
  NewInfo.msg              
  SurfPoint.msg
  KeyFrame.msg          
  ObjectFinded.msg         
  VCandidates.msg
  KeyFramePoseList.msg  
  ObjectTransformArray.msg  
  VSurf.msg
  ListModPoints.msg     
  ObjectTransform.msg
  LoadInfo.msg          
  Point2d.msg
)

generate_messages(
  DEPENDENCIES 
  roscpp
  sensor_msgs 
  std_msgs
)

catkin_package(
   CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime 
)

