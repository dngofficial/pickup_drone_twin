cmake_minimum_required(VERSION 3.8)
project(urc_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BeaconRange.msg"
  "msg/TagObservation.msg"
  "msg/TagObservationArray.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjectArray.msg"
  "msg/LocalizationStatus.msg"
  "msg/GripperCommand.msg"
  "msg/GripperStatus.msg"
  "msg/DeliveryEvent.msg"
  "msg/MissionStatus.msg"
  "srv/SetBinPolicy.srv"
  "srv/SelectTarget.srv"
  "srv/SetDetector.srv"
  "srv/SetGripper.srv"
  "srv/SceneControl.srv"
  "action/StartMission.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
