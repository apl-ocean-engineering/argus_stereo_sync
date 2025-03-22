//
//
// Copyright 2025 University of Washington

#pragma once

// clang-format off
// rclcpp.hpp must be included before anything that might call X11.h
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cudaEGL.h>
// clang-format on

namespace argus_stereo_sync {

class CudaFrameAcquire {
 public:
  CudaFrameAcquire(
      CUeglStreamConnection& connection,
      const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& publisher);
  ~CudaFrameAcquire();

  bool publish();

 private:
  CUeglStreamConnection& m_connection;
  CUgraphicsResource m_resource;
  CUeglFrame m_frame;
  CUstream m_stream;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}  // namespace argus_stereo_sync
