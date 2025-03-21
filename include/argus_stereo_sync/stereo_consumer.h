#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>

// #include <cuda.h>
#include <cudaEGL.h>

// #include "ArgusHelpers.h"
// #include "CUDAHelper.h"
// #include "EGLGlobal.h"
// #include "Error.h"
#include "Thread.h"

// #include "convert.h"

namespace argus_stereo_sync {

class StereoConsumer : public ArgusSamples::Thread {
  public:
    explicit StereoConsumer(Argus::IEGLOutputStream *leftStream, Argus::IEGLOutputStream *rightStream,   
       const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &left_pub,
       const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &right_pub );

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


}