#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/fill_image.hpp"

// #include <Argus/Argus.h>
// #include <EGLStream/EGLStream.h>
// #include <cuda.h>
// #include <cudaEGL.h>

// #include "ArgusHelpers.h"
// #include "CUDAHelper.h"
// #include "EGLGlobal.h"
// #include "Error.h"
// #include "Thread.h"


#include "argus_stereo_sync/cuda_frame_acquire.h"

#include "argus_stereo_sync/convert.h"

namespace argus_stereo_sync {

//   using namespace Argus;
// using namespace ArgusSamples;

///////////////////////////////////////////////////////////////////////////////////////////////////

CudaFrameAcquire::CudaFrameAcquire(CUeglStreamConnection& connection, const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &pub)
                                   : m_connection(connection)
                                   , m_stream(NULL), m_resource(0),
                                   pub_(pub) {
  CUresult result = cuEGLStreamConsumerAcquireFrame(&m_connection, &m_resource, &m_stream, -1);
  if (result == CUDA_SUCCESS) {
    cuGraphicsResourceGetMappedEglFrame(&m_frame, m_resource, 0, 0);
  }
}

CudaFrameAcquire::~CudaFrameAcquire() {
  if (m_resource) {
    cuEGLStreamConsumerReleaseFrame(&m_connection, m_resource, &m_stream);
  }
}

bool CudaFrameAcquire::publish() {
  CUDA_RESOURCE_DESC cudaResourceDesc;
  memset(&cudaResourceDesc, 0, sizeof(cudaResourceDesc));
  cudaResourceDesc.resType = CU_RESOURCE_TYPE_ARRAY;

  cudaResourceDesc.res.array.hArray = m_frame.frame.pArray[0];
  CUsurfObject cudaSurfObj1 = 0;
  CUresult cuResult = cuSurfObjectCreate(&cudaSurfObj1, &cudaResourceDesc);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to create surface object 1 (%s)", getCudaErrorString(cuResult));
  }
  
  cudaResourceDesc.res.array.hArray = m_frame.frame.pArray[1];
  CUsurfObject cudaSurfObj2 = 0;
  cuResult = cuSurfObjectCreate(&cudaSurfObj2, &cudaResourceDesc);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to create surface object 2 (%s)", getCudaErrorString(cuResult));
  }

  float delta = convertSurfObject(cudaSurfObj1, cudaSurfObj2, m_frame.width, m_frame.height, oBuffer);
  cuSurfObjectDestroy(cudaSurfObj1);
  cuSurfObjectDestroy(cudaSurfObj2);

  sensor_msgs::msg::Image output;
  //output.header.stamp = ros::Time::now();
  sensor_msgs::fillImage(output, sensor_msgs::image_encodings::BGR8, m_frame.height, m_frame.width, 3 * m_frame.width, (void*) oBuffer);

    pub_->publish(output);
  return true;
}

}