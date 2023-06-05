cmake_minimum_required(VERSION 2.8.3)
project(mobile_platform_msgs)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  nav_msgs
  map_msgs
  actionlib
  actionlib_msgs
)

find_package(Boost REQUIRED)

add_message_files(DIRECTORY msg
  FILES
  PurePursuitStatus.msg
  PurePursuitResult.msg
  Process.msg
  PathPlanningRoute.msg
  CloudInfo.msg
  Cleaning.msg
  BcmCommand.msg
  DriveCommand.msg
  BMS.msg
  Chassis.msg
  Edge.msg
  EPS.msg
  ErrorCode.msg
  Joy.msg
  Light.msg
  Motor.msg
  Motor.msg
  Surround.msg
  WheelEncoder.msg
  SweeperAuxilaries.msg
  VCU.msg
  EpsCalibration.msg
  Radio.msg
  Sonar.msg
  SonarProbe.msg
  AreaWarning.msg
  LocalizationLost.msg
  ScanMatchingStatus.msg
)

add_service_files(DIRECTORY srv
  FILES
  Agent.srv
  JTCoveragePlanning.srv
  HomeToDock.srv
  MainVoice.srv
  TmpVoice.srv
  VolumeChange.srv
  TargetPoint.srv
  PassingPoint.srv
  CreatePath.srv
  CreateZone.srv
  CollectPose.srv
  GetDeviceMeta.srv
  GetYawMap.srv
  InitPoseSet.srv
  Pursuit.srv
  UpdateMap.srv
  BaseStatus.srv
  GeneratePath.srv
  InitPoseLoad.srv
  InitPoseSave.srv
  GoToChargingPoint.srv
  GoToCoordinate.srv
  GoToWaypoint.srv
  WaypointLoad.srv
  WaypointSave.srv
  PatrolStart.srv
  PatrolStop.srv
  VirtualLineLoad.srv
  VirtualLineSave.srv
  NavigationStart.srv
  NavigationStop.srv
  MappingDelete.srv
  MappingEdit.srv
  MappingList.srv
  MappingRevert.srv
  MappingSave.srv
  MappingSelect.srv
  MappingSelected.srv
  MappingStart.srv
  MappingStop.srv
  SetChargingRelay.srv
  ChargingLoad.srv
  ChargingSave.srv
  ChargingService.srv
  GetChargingRelay.srv
  DockingStart.srv
  DockingStop.srv
  AutodockingRestart.srv
  DockingStatus.srv
  ProcessStatus.srv
  MotorControllerFeedback.srv
  HeadingControl.srv
  SpeedControl.srv
  SystemdService.srv
  PathPlanningSave.srv
  PathPlanningList.srv
  PathPlanningLoad.srv
  PathPlanningExecute.srv
  PathPlanningDelete.srv
  PathTracking.srv
  UpdateProhibition.srv
  PathRecord.srv
  CollectPose.srv
  BagRecord.srv
  ConvexPolygon.srv
  OssUpload.srv
  CoveragePointMap.srv
  ZoneApiSrv.srv
)

add_action_files(
  FILES
  SavingAllMap.action
)

add_action_files(
  FILES
  SavingAllMap.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  map_msgs
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime actionlib actionlib_msgs)

install(
  DIRECTORY msg srv action
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

