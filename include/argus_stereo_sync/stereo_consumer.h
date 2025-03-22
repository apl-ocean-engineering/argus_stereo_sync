//
//
// Copyright 2025 University of Washington

#pragma once

// clang-format off
// rclcpp must be included before anything that might include X11.h
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
// clang-format on

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <cudaEGL.h>

#include "Thread.h"

namespace argus_stereo_sync {

class StereoConsumer : public ArgusSamples::Thread {
 public:
  explicit StereoConsumer(
      Argus::IEGLOutputStream *leftStream, Argus::IEGLOutputStream *rightStream,
      const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &left_pub,
      const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &right_pub);

  ~StereoConsumer();

 private:
  virtual bool threadInitialize();
  virtual bool threadExecute();
  virtual bool threadShutdown();

  Argus::IEGLOutputStream *m_leftStream;
  Argus::IEGLOutputStream *m_rightStream;
  CUeglStreamConnection m_cuStreamLeft;
  CUeglStreamConnection m_cuStreamRight;
  CUcontext m_cudaContext;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
};

}  // namespace argus_stereo_sync
