cmake_minimum_required(VERSION 3.0.2)
project(uma_fp_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  tf
  std_msgs
  omni_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system filesystem date_time thread)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(FILES
  ftResponse.msg
  zeroSensorpetition.msg
  stitchArea.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
  include
  LIBRARIES
  error_lib
  pid_lib
  fulcrum_pose_lib
  kinematic_lib
  force_control_lib
  initialize_lib
  tool_lib
  transformadas_lib
  ur_lib
  read_tf_lib
  fulcrum_lib
  alpha_beta_lib
  fuzzy_lib
  jacob_lib
  CATKIN_DEPENDS
  #roscpp
  #std_msgs
  #tf2_ros
  #tf2_eigen
  #tf2_geometry_msgs
  #geometry_msgs
  #sensor_msgs
  #tf
  interactive_markers
  #moveit_core
  #moveit_ros_planning
  #moveit_ros_planning_interface
  #moveit_visual_tools
  DEPENDS
  system_lib
  EIGEN3
  Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${TF_INCLUDE_DIRS}
  dependecies/include
  services/include
)

## Declare a C++ library
add_library(error_lib dependecies/src/computeError.cpp)
add_library(pid_lib dependecies/src/computePID.cpp)
add_library(fulcrum_pose_lib dependecies/src/computePoseDes_JM.cpp)
add_library(kinematic_lib dependecies/src/computeT.cpp)
add_library(force_control_lib dependecies/src/forceControlStrategy.cpp)
add_library(fulcrum_lib dependecies/src/fulcrumEstimation.cpp)
add_library(initialize_lib dependecies/src/initialize.cpp)
add_library(tool_lib dependecies/src/selectTool.cpp)
add_library(transformadas_lib dependecies/src/uma_transf_class.cpp)
add_library(ur_lib dependecies/src/ur_script.cpp)
add_library(read_tf_lib dependecies/src/readRobotTransform.cpp)
add_library(alpha_beta_lib dependecies/src/alphaBegaGenerator.cpp)
add_library(fuzzy_lib dependecies/src/fuzzySystem.cpp)
add_library(jacob_lib dependecies/src/jacobianss.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/uma_fp_control_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

################
## Executable ##
################
#add_executable(turtle_tf_listener dependecies/src/readRobotTransform.cpp)
#target_link_libraries(turtle_tf_listener ${catkin_LIBRARIES})
# CONTROL
#add_executable(${PROJECT_NAME}_control services/src/robotControl.cpp) # dependecies/src/readRobotTransform.cpp
#set_target_properties(${PROJECT_NAME}_control PROPERTIES OUTPUT_NAME control PREFIX "")
#target_link_libraries(${PROJECT_NAME}_control ${catkin_LIBRARIES} tool_lib transformadas_lib error_lib pid_lib
#                                                                  fulcrum_pose_lib force_control_lib fulcrum_lib
#                                                                  ur_lib initialize_lib kinematic_lib)#read_tf_lib
                                                                
#add_executable(${PROJECT_NAME}_controlVacuum services/src/robotControllVacuum.cpp) # dependecies/src/readRobotTransform.cpp
#set_target_properties(${PROJECT_NAME}_controlVacuum PROPERTIES OUTPUT_NAME controlVacuum PREFIX "")
#target_link_libraries(${PROJECT_NAME}_controlVacuum ${catkin_LIBRARIES} tool_lib transformadas_lib error_lib pid_lib
#                                                                  fulcrum_pose_lib force_control_lib fulcrum_lib
#                                                                  ur_lib initialize_lib kinematic_lib)#read_tf_lib

add_executable(${PROJECT_NAME}_controlTeleop services/src/robotControlTeleop.cpp) # dependecies/src/readRobotTransform.cpp
set_target_properties(${PROJECT_NAME}_controlTeleop PROPERTIES OUTPUT_NAME controlTeleop PREFIX "")
target_link_libraries(${PROJECT_NAME}_controlTeleop ${catkin_LIBRARIES} tool_lib transformadas_lib error_lib pid_lib
                                                                  fulcrum_pose_lib force_control_lib fulcrum_lib
                                                                  ur_lib initialize_lib kinematic_lib)#read_tf_lib

add_executable(${PROJECT_NAME}_feedbackHp services/src/forceFeedbackHp.cpp)
set_target_properties(${PROJECT_NAME}_feedbackHp PROPERTIES OUTPUT_NAME feedbackHp PREFIX "")
target_link_libraries(${PROJECT_NAME}_feedbackHp ${catkin_LIBRARIES})

#add_executable(${PROJECT_NAME}_forceControl services/src/robotForceControl.cpp) # dependecies/src/readRobotTransform.cpp
#set_target_properties(${PROJECT_NAME}_forceControl PROPERTIES OUTPUT_NAME forceControl PREFIX "")
#target_link_libraries(${PROJECT_NAME}_forceControl ${catkin_LIBRARIES} tool_lib transformadas_lib error_lib pid_lib
#                                                                  fulcrum_pose_lib force_control_lib fulcrum_lib
#                                                                  ur_lib initialize_lib kinematic_lib fuzzy_lib)

#add_executable(${PROJECT_NAME}_jacobianControl services/src/jacobianControl.cpp) # dependecies/src/readRobotTransform.cpp
#set_target_properties(${PROJECT_NAME}_jacobianControl PROPERTIES OUTPUT_NAME jacobianControl PREFIX "")
#target_link_libraries(${PROJECT_NAME}_jacobianControl ${catkin_LIBRARIES} tool_lib transformadas_lib error_lib pid_lib
#                                                                          fulcrum_pose_lib force_control_lib fulcrum_lib
#                                                                          ur_lib initialize_lib kinematic_lib fuzzy_lib)

add_executable(${PROJECT_NAME}_tfg services/src/movimientoTFG.cpp) # dependecies/src/readRobotTransform.cpp
set_target_properties(${PROJECT_NAME}_tfg PROPERTIES OUTPUT_NAME tfg PREFIX "")
target_link_libraries(${PROJECT_NAME}_tfg ${catkin_LIBRARIES} tool_lib transformadas_lib error_lib pid_lib
                                                                          fulcrum_pose_lib force_control_lib fulcrum_lib
                                                                          ur_lib initialize_lib kinematic_lib fuzzy_lib)

add_executable(${PROJECT_NAME}_hybridControl services/src/hybridControl.cpp) # dependecies/src/readRobotTransform.cpp
set_target_properties(${PROJECT_NAME}_hybridControl PROPERTIES OUTPUT_NAME hybridControl PREFIX "")
target_link_libraries(${PROJECT_NAME}_hybridControl ${catkin_LIBRARIES} tool_lib transformadas_lib error_lib pid_lib
                                                                          fulcrum_pose_lib force_control_lib fulcrum_lib
                                                                          ur_lib initialize_lib kinematic_lib fuzzy_lib)

add_executable(${PROJECT_NAME}_abdomenForces services/src/abdomenForces.cpp) # dependecies/src/readRobotTransform.cpp
set_target_properties(${PROJECT_NAME}_abdomenForces PROPERTIES OUTPUT_NAME abdomenForces PREFIX "")
target_link_libraries(${PROJECT_NAME}_abdomenForces ${catkin_LIBRARIES} tool_lib transformadas_lib error_lib pid_lib
                                                                          fulcrum_pose_lib force_control_lib fulcrum_lib
                                                                          ur_lib initialize_lib kinematic_lib fuzzy_lib)
#read_tf_lib
# COORDINADOR
add_executable(${PROJECT_NAME}_coordinator src/coordinatorTwoRobot_InputByPoint.cpp)
set_target_properties(${PROJECT_NAME}_coordinator PROPERTIES OUTPUT_NAME coordinator PREFIX "")
target_link_libraries(${PROJECT_NAME}_coordinator ${catkin_LIBRARIES})
# ------
#add_executable(${PROJECT_NAME}_coordinatorONE src/coordinatorOneRobot.cpp)
#set_target_properties(${PROJECT_NAME}_coordinatorONE PROPERTIES OUTPUT_NAME coordinatorONE PREFIX "")
#target_link_libraries(${PROJECT_NAME}_coordinatorONE ${catkin_LIBRARIES})
#coordinatorTwoRobot_InputByTeleop
add_executable(${PROJECT_NAME}_coordinatorTeleop src/coordinatorTwoRobot_InputByTeleop.cpp)
set_target_properties(${PROJECT_NAME}_coordinatorTeleop PROPERTIES OUTPUT_NAME coordinatorTeleop PREFIX "")
target_link_libraries(${PROJECT_NAME}_coordinatorTeleop ${catkin_LIBRARIES})
# -------
add_executable(${PROJECT_NAME}_coordinator_oneTeleop src/coordinator_oneTeleop.cpp)
set_target_properties(${PROJECT_NAME}_coordinator_oneTeleop PROPERTIES OUTPUT_NAME coordinator_oneTeleop PREFIX "")
target_link_libraries(${PROJECT_NAME}_coordinator_oneTeleop ${catkin_LIBRARIES})
#identificador sistema robot
#add_executable(${PROJECT_NAME}_identificador src/identificarSistema.cpp)
#set_target_properties(${PROJECT_NAME}_identificador PROPERTIES OUTPUT_NAME identificador PREFIX "")
#target_link_libraries(${PROJECT_NAME}_identificador ${catkin_LIBRARIES})
#
add_executable(${PROJECT_NAME}_coordinatorForce src/coordinatorForceTest_twoRobot.cpp)
set_target_properties(${PROJECT_NAME}_coordinatorForce PROPERTIES OUTPUT_NAME coordinatorForce PREFIX "")
target_link_libraries(${PROJECT_NAME}_coordinatorForce ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_coordinatorVacuum src/coordinatorVacuum_twoRobot.cpp)
set_target_properties(${PROJECT_NAME}_coordinatorVacuum PROPERTIES OUTPUT_NAME coordinatorVacuum PREFIX "")
target_link_libraries(${PROJECT_NAME}_coordinatorVacuum ${catkin_LIBRARIES})
# HAPTIC
add_executable(${PROJECT_NAME}_hapticMock mocks/teleoperacionKeyboard.cpp)
set_target_properties(${PROJECT_NAME}_hapticMock PROPERTIES OUTPUT_NAME hapticMock PREFIX "")
target_link_libraries(${PROJECT_NAME}_hapticMock ${catkin_LIBRARIES})
# teleop
add_executable(${PROJECT_NAME}_teleopTFG mocks/teleoperaciontfgKeyboard.cpp)
set_target_properties(${PROJECT_NAME}_teleopTFG PROPERTIES OUTPUT_NAME teleopTFG PREFIX "")
target_link_libraries(${PROJECT_NAME}_teleopTFG ${catkin_LIBRARIES})
# DIGITAL TWINS SOFT TISSUE
#add_executable(${PROJECT_NAME}_tissueDT services/src/softTissueDT.cpp)
#set_target_properties(${PROJECT_NAME}_tissueDT PROPERTIES OUTPUT_NAME tissueDT PREFIX "")
#target_link_libraries(${PROJECT_NAME}_tissueDT ${catkin_LIBRARIES})
# GRIPPER
#add_executable(${PROJECT_NAME}_gripperMock mocks/gripperMock.cpp)
#set_target_properties(${PROJECT_NAME}_gripperMock PROPERTIES OUTPUT_NAME gripperMock PREFIX "")
#target_link_libraries(${PROJECT_NAME}_gripperMock ${catkin_LIBRARIES})
# VISION
#add_executable(${PROJECT_NAME}_visionMock mocks/visionFreeNeedle.cpp)
#set_target_properties(${PROJECT_NAME}_visionMock PROPERTIES OUTPUT_NAME visionMock PREFIX "")
#target_link_libraries(${PROJECT_NAME}_visionMock ${catkin_LIBRARIES})
# FORCE SENSOR 
#add_executable(${PROJECT_NAME}_fulcrumSensor mocks/abdomenForceSensorMock.cpp)
#set_target_properties(${PROJECT_NAME}_fulcrumSensor PROPERTIES OUTPUT_NAME fulcrumSensor PREFIX "")
#target_link_libraries(${PROJECT_NAME}_fulcrumSensor ${catkin_LIBRARIES})
#---
#add_executable(${PROJECT_NAME}_tissueSensor mocks/tissueForceSensorMock.cpp)
#set_target_properties(${PROJECT_NAME}_tissueSensor PROPERTIES OUTPUT_NAME tissueSensor PREFIX "")
#target_link_libraries(${PROJECT_NAME}_tissueSensor ${catkin_LIBRARIES})
# EVALUATE
#add_executable(${PROJECT_NAME}_evalPredicates services/src/evaluatePredicates.cpp)
#set_target_properties(${PROJECT_NAME}_evalPredicates PROPERTIES OUTPUT_NAME evalPredicates PREFIX "")
#target_link_libraries(${PROJECT_NAME}_evalPredicates ${catkin_LIBRARIES})
# ROSBAG 2 CSV
#add_executable(${PROJECT_NAME}_ros2csv services/src/getCSV.cpp)
#set_target_properties(${PROJECT_NAME}_ros2csv PROPERTIES OUTPUT_NAME ros2csv PREFIX "")
#target_link_libraries(${PROJECT_NAME}_ros2csv ${catkin_LIBRARIES})
#------------
#add_executable(${PROJECT_NAME}_ros2TemporalCSV2 services/src/getTemporalCSV2.cpp)
#set_target_properties(${PROJECT_NAME}_ros2TemporalCSV2 PROPERTIES OUTPUT_NAME ros2TemporalCSV2 PREFIX "")
#target_link_libraries(${PROJECT_NAME}_ros2TemporalCSV2 ${catkin_LIBRARIES})
# ROSBAG 2 CSV 4 OWL
#add_executable(${PROJECT_NAME}_ros2csv4owl services/src/getCSVforOWL.cpp)
#set_target_properties(${PROJECT_NAME}_ros2csv4owl PROPERTIES OUTPUT_NAME ros2csv4owl PREFIX "")
#target_link_libraries(${PROJECT_NAME}_ros2csv4owl ${catkin_LIBRARIES})
# ONTOLOGY MOCK
#add_executable(${PROJECT_NAME}_ontologyMock mocks/ontologyMock.cpp)
#set_target_properties(${PROJECT_NAME}_ontologyMock PROPERTIES OUTPUT_NAME ontologyMock PREFIX "")
#target_link_libraries(${PROJECT_NAME}_ontologyMock ${catkin_LIBRARIES})
# CALIBRATION
add_executable(${PROJECT_NAME}_calibration services/src/getCalibration2.cpp)
set_target_properties(${PROJECT_NAME}_calibration PROPERTIES OUTPUT_NAME calibration PREFIX "")
target_link_libraries(${PROJECT_NAME}_calibration ${catkin_LIBRARIES} tool_lib transformadas_lib error_lib pid_lib
                                                                      fulcrum_pose_lib force_control_lib fulcrum_lib
                                                                      ur_lib initialize_lib kinematic_lib fuzzy_lib)