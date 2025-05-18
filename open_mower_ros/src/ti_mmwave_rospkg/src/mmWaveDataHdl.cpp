// Copyright 2021 TI/Zhang/ETHZ-ASL (?)


#include "ti_mmwave_rospkg/DataHandlerClass.h"
#include "ti_mmwave_rospkg/mmWaveDataHdl.h"
#include <string>

namespace ti_mmwave_rospkg
{
PLUGINLIB_EXPORT_CLASS(ti_mmwave_rospkg::mmWaveDataHdl, nodelet::Nodelet);

mmWaveDataHdl::mmWaveDataHdl()
{
}

void mmWaveDataHdl::onInit()
{
  ros::NodeHandle private_nh("~");

  std::string mySerialPort;
  std::string myFrameID;
  int myBaudRate;
  int myMaxAllowedElevationAngleDeg;
  int myMaxAllowedAzimuthAngleDeg;

  private_nh.getParam("data_port", mySerialPort);

  private_nh.getParam("data_rate", myBaudRate);

  private_nh.getParam("frame_id", myFrameID);

  if (!(private_nh.getParam("max_allowed_elevation_angle_deg", myMaxAllowedElevationAngleDeg)))
  {
    myMaxAllowedElevationAngleDeg = 90;  // Use max angle if none specified
  }

  if (!(private_nh.getParam("max_allowed_azimuth_angle_deg", myMaxAllowedAzimuthAngleDeg)))
  {
    myMaxAllowedAzimuthAngleDeg = 90;  // Use max angle if none specified
  }

  ROS_INFO_STREAM("mmWaveDataHdl: data_port = " << mySerialPort.c_str());
  ROS_INFO_STREAM("mmWaveDataHdl: data_rate = " << myBaudRate);
  ROS_INFO_STREAM("mmWaveDataHdl: max_allowed_elevation_angle_deg = " <<  myMaxAllowedElevationAngleDeg);
  ROS_INFO_STREAM("mmWaveDataHdl: max_allowed_azimuth_angle_deg = " <<  myMaxAllowedAzimuthAngleDeg);
  ROS_INFO_STREAM("mmWaveDataHdl: myFrameID = " <<  myFrameID);

  DataUARTHandler DataHandler(&private_nh);
  DataHandler.setFrameID((char*)myFrameID.c_str());  // NOLINT => linter would prefer casting
  DataHandler.setUARTPort((char*)mySerialPort.c_str());  // NOLINT => linter would prefer casting
  DataHandler.setBaudRate(myBaudRate);
  DataHandler.setMaxAllowedElevationAngleDeg(myMaxAllowedElevationAngleDeg);
  DataHandler.setMaxAllowedAzimuthAngleDeg(myMaxAllowedAzimuthAngleDeg);
  DataHandler.start();

  NODELET_DEBUG("mmWaveDataHdl: Finished onInit function");
}

}  // namespace ti_mmwave_rospkg
