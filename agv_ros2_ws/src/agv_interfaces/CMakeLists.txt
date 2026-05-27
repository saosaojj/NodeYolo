cmake_minimum_required(VERSION 3.8)
project(agv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlcData.msg"
  "msg/IOState.msg"
  "msg/AGVStatus.msg"
  "msg/YoloDetection.msg"
  "msg/YoloResult.msg"
  "msg/WiFiStatus.msg"
  "msg/BluetoothDevice.msg"
  "msg/BatteryState.msg"
  "msg/Scan3DData.msg"
  "msg/PointCloudChunk.msg"
  "msg/CameraConfig.msg"
  "msg/PlcConfig.msg"
  "msg/SystemConfig.msg"
  "msg/MotorState.msg"
  "msg/DriveCommand.msg"
  "srv/ReadPlc.srv"
  "srv/WritePlc.srv"
  "srv/SetIO.srv"
  "srv/ControlAGV.srv"
  "srv/TrainModel.srv"
  "srv/ConnectWiFi.srv"
  "srv/ConnectBluetooth.srv"
  "srv/SetModel.srv"
  "srv/SetConfidence.srv"
  "srv/SetCharging.srv"
  "srv/GenerateScanMap.srv"
  "srv/StartScan.srv"
  "srv/GetConfig.srv"
  "srv/SetConfig.srv"
  "srv/SaveConfig.srv"
  "srv/LoadConfig.srv"
  "srv/SetSpeed.srv"
  "srv/SetSteering.srv"
  "action/NavigateTo.action"
  "action/Patrol.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
