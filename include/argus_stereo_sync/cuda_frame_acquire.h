#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

// #include <EGLStream/EGLStream.h>
// #include <cuda.h>
#include <cudaEGL.h>

// #include "ArgusHelpers.h"
// #include "CUDAHelper.h"
// #include "EGLGlobal.h"
// #include "Error.h"
// #include "Thread.h"

// #include "convert.h"

namespace argus_stereo_sync {

class CudaFrameAcquire {
   public:
     CudaFrameAcquire(CUeglStreamConnection& connection, 
       const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &publisher);
     ~CudaFrameAcquire();

    bool publish();
  
  private:
    CUeglStreamConnection& m_connection;
    CUgraphicsResource m_resource;
    CUeglFrame m_frame;
    CUstream m_stream;

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

};

}
