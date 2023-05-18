#include <camera_info_manager/camera_info_manager.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <cstdio>
#include <functional>
#include <iostream>
#include <tuple>
#include <cassert>  // assert
#include <bits/stdc++.h>

// Inludes common necessary includes for development using depthai library
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/ImuConverter.hpp>

#include "depthai/depthai.hpp"

std::vector<std::string> usbStrings = { "UNKNOWN", "LOW", "FULL", "HIGH", "SUPER", "SUPER_PLUS" };

auto getMonoResolution(std::string resolution)
{
  typedef dai::node::MonoCamera::Properties::SensorResolution monoResolution;
  if (resolution == "720p")
  {
    return std::make_tuple(1280, 720, monoResolution::THE_720_P);
  }
  else if (resolution == "400p")
  {
    return std::make_tuple(640, 400, monoResolution::THE_400_P);
  }
  else if (resolution == "800p")
  {
    return std::make_tuple(1280, 800, monoResolution::THE_800_P);
  }
  else if (resolution == "480p")
  {
    return std::make_tuple(640, 480, monoResolution::THE_480_P);
  }
  else
  {
    ROS_ERROR("Invalid parameter. -> monoResolution: %s", resolution.c_str());
    throw std::runtime_error("Invalid mono camera resolution.");
  }
}
auto getRGBResolution(std::string resolution)
{
  typedef dai::node::ColorCamera::Properties::SensorResolution rgbResolution;
  if (resolution == "1080p")
  {
    return std::make_tuple(1920, 1080, rgbResolution::THE_1080_P);
  }
  else if (resolution == "4K")
  {
    return std::make_tuple(3840, 2160, rgbResolution::THE_4_K);
  }
  else if (resolution == "12MP")
  {
    return std::make_tuple(4056, 3040, rgbResolution::THE_12_MP);
  }
  else if (resolution == "13MP")
  {
    return std::make_tuple(4208, 3120, rgbResolution::THE_13_MP);
  }
  else
  {
    ROS_ERROR("Invalid parameter. -> rgbResolution: %s", resolution.c_str());
    throw std::runtime_error("Invalid color camera resolution.");
  }
}
auto addMonoPipeline(dai::Pipeline& pipeline, int fps, dai::node::MonoCamera::Properties::SensorResolution resolution,
                     std::string name, dai::CameraBoardSocket socket)
{
  auto monoCamera = pipeline.create<dai::node::MonoCamera>();
  // auto xOut = pipeline.create<dai::node::XLinkOut>();
  // xOut->setStreamName(name);

  monoCamera->setResolution(resolution);
  monoCamera->setBoardSocket(socket);
  monoCamera->setFps(fps);

  // monoCamera->out.link(xOut->input);
  return monoCamera;
}
void addRGBPipeline(dai::Pipeline& pipeline, int fps, dai::node::ColorCamera::Properties::SensorResolution resolution,
                    std::string name, dai::CameraBoardSocket socket)
{
  auto rgbCamera = pipeline.create<dai::node::ColorCamera>();
  auto xOut = pipeline.create<dai::node::XLinkOut>();
  xOut->setStreamName(name);

  rgbCamera->setResolution(resolution);
  rgbCamera->setBoardSocket(socket);
  rgbCamera->setFps(fps);

  rgbCamera->isp.link(xOut->input);
}
auto addDepthPipeline(dai::Pipeline& pipeline, int fps, dai::node::ColorCamera::Properties::SensorResolution resolution,
                    std::string name, dai::CameraBoardSocket socket)
{
  auto stereo = pipeline.create<dai::node::StereoDepth>();
  // auto xOut = pipeline.create<dai::node::XLinkOut>();
  // xOut->setStreamName(name);


  stereo->initialConfig.setConfidenceThreshold(200);        // Known to be best
  stereo->setRectifyEdgeFillColor(0);                              // black, to better see the cutout
  stereo->initialConfig.setLeftRightCheckThreshold(5);  // Known to be best
  stereo->setLeftRightCheck(true);
  stereo->setExtendedDisparity(false);
  stereo->setSubpixel(true);
  stereo->setRectifyEdgeFillColor(0);
  // stereo->depth.link(xOut->input);
  return stereo;
}
void addIMUPipeline(dai::Pipeline& pipeline, int fps, std::string name)
{
  auto imu = pipeline.create<dai::node::IMU>();
  auto xOut = pipeline.create<dai::node::XLinkOut>();
  xOut->setStreamName(name);

  imu->enableIMUSensor({ dai::IMUSensor::ACCELEROMETER, dai::IMUSensor::GYROSCOPE_CALIBRATED }, fps);
  imu->setBatchReportThreshold(5);
  imu->setMaxBatchReports(20);

  imu->out.link(xOut->input);
}
std::tuple<dai::Pipeline, int, int> createPipeline(int fps, int imu_frequency, std::string stereoResolution,
                                                   std::string colorResolution)
{
  dai::Pipeline pipeline;
  pipeline.setXLinkChunkSize(0);

  dai::node::MonoCamera::Properties::SensorResolution monoResolution;
  int stereoWidth, stereoHeight;
  std::tie(stereoWidth, stereoHeight, monoResolution) = getMonoResolution(stereoResolution);
  auto rgbResolution = std::get<2>(getRGBResolution(colorResolution));

  auto left_pipe = addMonoPipeline(pipeline, fps, monoResolution, "left", dai::CameraBoardSocket::LEFT);
  auto right_pipe = addMonoPipeline(pipeline, fps, monoResolution, "right", dai::CameraBoardSocket::RIGHT);
  addRGBPipeline(pipeline, fps, rgbResolution, "rgb", dai::CameraBoardSocket::RGB);
  auto depth_pipe = addDepthPipeline(pipeline, fps, rgbResolution, "depth", dai::CameraBoardSocket::RGB);
  addIMUPipeline(pipeline, imu_frequency, "imu");

  auto xOutLeft = pipeline.create<dai::node::XLinkOut>();
  auto xOutRight = pipeline.create<dai::node::XLinkOut>();
  xOutLeft->setStreamName("left");
  xOutRight->setStreamName("right");


  left_pipe->out.link(depth_pipe->left);
  right_pipe->out.link(depth_pipe->right);

  auto xOutDepth = pipeline.create<dai::node::XLinkOut>();
  xOutDepth->setStreamName("depth");
  depth_pipe->depth.link(xOutDepth->input);
  depth_pipe->syncedLeft.link(xOutLeft->input);
  depth_pipe->syncedRight.link(xOutRight->input);

  return std::make_tuple(pipeline, stereoWidth, stereoHeight);
}
std::shared_ptr<dai::Device> connect(ros::NodeHandle& nh, dai::Pipeline& pipeline, std::string mxId)
{
  bool isDeviceFound = false;
  std::shared_ptr<dai::Device> device;
  std::vector<dai::DeviceInfo> availableDevices = dai::Device::getAllAvailableDevices();

  std::cout << "Listing available devices..." << std::endl;
  for (auto deviceInfo : availableDevices)
  {
    std::cout << "Device Mx ID: " << deviceInfo.getMxId() << std::endl;
    if (deviceInfo.getMxId() == mxId)
    {
      if (deviceInfo.state == X_LINK_UNBOOTED || deviceInfo.state == X_LINK_BOOTLOADER)
      {
        isDeviceFound = true;
        device = std::make_shared<dai::Device>(pipeline, deviceInfo);
        break;
      }
      else if (deviceInfo.state == X_LINK_BOOTED)
      {
        throw std::runtime_error("ros::NodeHandle() from Node \"" + nh.getNamespace() +
                                 "\" DepthAI Device with MxId  \"" + mxId +
                                 "\" is already booted on different process.  \"");
      }
    }
    else if (mxId.empty())
    {
      isDeviceFound = true;
      device = std::make_shared<dai::Device>(pipeline, deviceInfo);
      break;
    }
  }

  if (!isDeviceFound)
  {
    throw std::runtime_error("ros::NodeHandle() from Node \"" + nh.getNamespace() + "\" DepthAI Device with MxId  \"" +
                             mxId + "\" not found.  \"");
  }

  std::cout << "Device USB status: " << usbStrings[static_cast<int32_t>(device->getUsbSpeed())] << std::endl;
  return device;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "oak-d_w");
  ros::NodeHandle nh("~");

  std::string tf, mxId, monoResolution = "720p", rgbResolution = "1080p";
  int badParams = 0, fps = 30, imu_frequency = 400;
  double angularVelCovariance, linearAccelCovariance;

  badParams += !nh.getParam("mxId", mxId);
  badParams += !nh.getParam("tf", tf);
  badParams += !nh.getParam("imu_frequency", imu_frequency);
  badParams += !nh.getParam("fps", fps);
  badParams += !nh.getParam("monoResolution", monoResolution);
  badParams += !nh.getParam("rgbResolution", rgbResolution);
  badParams += !nh.getParam("angularVelCovariance", angularVelCovariance);
  badParams += !nh.getParam("linearAccelCovariance", linearAccelCovariance);

  if (badParams > 0)
  {
    std::cout << " Bad parameters -> " << badParams << std::endl;
    throw std::runtime_error("Couldn't find %d of the parameters");
  }

  dai::Pipeline pipeline;
  int width, height;
  std::tie(pipeline, width, height) = createPipeline(fps, imu_frequency, monoResolution, rgbResolution);

  auto device = connect(nh, pipeline, mxId);
  auto calibrationHandler = device->readCalibration();

  dai::rosBridge::ImageConverter converter(tf + "_left_camera_optical_frame", true);
  dai::rosBridge::ImageConverter rightconverter(tf + "_right_camera_optical_frame", true);
  dai::rosBridge::ImageConverter rgbConverter(tf + "_rgb_camera_optical_frame", false);
  dai::rosBridge::ImuConverter imuConverter(tf + "_imu_frame", dai::ros::ImuSyncMethod::COPY,
                                            linearAccelCovariance, angularVelCovariance);

  auto leftCameraInfo =
      converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::LEFT, width, height);
  auto rightCameraInfo =
      converter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RIGHT, width, height);
  auto rgbCameraInfo =
      rgbConverter.calibrationToCameraInfo(calibrationHandler, dai::CameraBoardSocket::RGB, width, height);

  auto leftQueue = device->getOutputQueue("left", 30, false);
  auto rightQueue = device->getOutputQueue("right", 30, false);
  auto imgQueue = device->getOutputQueue("rgb", 30, false);
  auto imuQueue = device->getOutputQueue("imu", 30, false);
  auto depthQueue = device->getOutputQueue("depth", 30, false);

  dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> leftPublish(
      leftQueue, nh, "left_raw/image_raw_gray", [&](auto in, auto& out) { converter.toRosMsg(in, out); }, 30, leftCameraInfo,
      "left_raw");
  dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rightPublish(
      rightQueue, nh, "right_raw/image_raw_gray", [&](auto in, auto& out) { rightconverter.toRosMsg(in, out); }, 30,
      rightCameraInfo, "right_raw");
  dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> rgbPublish(
      imgQueue, nh, std::string("rgb/image_raw_rgb"), [&](auto in, auto& out) { rgbConverter.toRosMsg(in, out); }, 30,
      rgbCameraInfo, "rgb");
  dai::rosBridge::BridgePublisher<sensor_msgs::Imu, dai::IMUData> imuPublish(
      imuQueue, nh, std::string("imu/data"), [&](auto in, auto& out) { imuConverter.toRosMsg(in, out); }, 100, "", "imu");
  dai::rosBridge::BridgePublisher<sensor_msgs::Image, dai::ImgFrame> depthPublish(
            depthQueue,
            nh,
            std::string("depth/depth_registered_raw"),
            std::bind(&dai::rosBridge::ImageConverter::toRosMsg,
                      &rgbConverter,  // since the converter has the same frame name
                                        // and image type is also same we can reuse it
                      std::placeholders::_1,
                      std::placeholders::_2),
            30,
            rgbCameraInfo,
            "depth");

  rightPublish.addPublisherCallback();
  leftPublish.addPublisherCallback();
  rgbPublish.addPublisherCallback();
  imuPublish.addPublisherCallback();
  depthPublish.addPublisherCallback();

  ros::spin();
  return 0;
}