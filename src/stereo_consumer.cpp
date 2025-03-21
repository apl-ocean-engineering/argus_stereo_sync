// #include "argus_stereo_sync/convert.h"
#include "argus_stereo_sync/stereo_consumer.h"
#include "argus_stereo_sync/cuda_frame_acquire.h"

// #include <Argus/Argus.h>
// #include <EGLStream/EGLStream.h>
// #include <cuda.h>
// #include <cudaEGL.h>

// #include "ArgusHelpers.h"
#include "CUDAHelper.h"
#include "EGLGlobal.h"
#include "Error.h"
// #include "Thread.h"


#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)


namespace argus_stereo_sync {

using namespace Argus;
using namespace ArgusSamples;


StereoConsumer::StereoConsumer(Argus::IEGLOutputStream *leftStream, 
                                Argus::IEGLOutputStream *rightStream,   
                                const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &left_pub,
                                const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &right_pub )
           : m_leftStream(leftStream), 
             m_rightStream(rightStream),
             m_cuStreamLeft(nullptr), 
             m_cuStreamRight(nullptr),
             m_cudaContext(0), 
             left_pub_(left_pub), 
             right_pub_(right_pub)
                  {}

StereoConsumer::~StereoConsumer() {}

///////////////////////////////////////////////////////////////////////////////////////////////////

bool StereoConsumer::threadInitialize() {
  PROPAGATE_ERROR(initCUDA(&m_cudaContext));

  CONSUMER_PRINT("Connecting CUDA consumer to left stream\n");
  CUresult cuResult = cuEGLStreamConsumerConnect(&m_cuStreamLeft, m_leftStream->getEGLStream());
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to connect CUDA to EGLStream (%s)", getCudaErrorString(cuResult));
  }

  CONSUMER_PRINT("Connecting CUDA consumer to right stream\n");
  cuResult = cuEGLStreamConsumerConnect(&m_cuStreamRight, m_rightStream->getEGLStream());
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to connect CUDA to EGLStream (%s)", getCudaErrorString(cuResult));
  }
  return true;
}

bool StereoConsumer::threadExecute() {
  CONSUMER_PRINT("Waiting for Argus producer to connect to left stream.\n");
  m_leftStream->waitUntilConnected();

  CONSUMER_PRINT("Waiting for Argus producer to connect to right stream.\n");
  m_rightStream->waitUntilConnected();

  CONSUMER_PRINT("Streams connected, processing frames.\n");
  while (true) {
    CONSUMER_PRINT("Thread running...\n");
    EGLint streamState = EGL_STREAM_STATE_CONNECTING_KHR;

    if (!eglQueryStreamKHR(m_leftStream->getEGLDisplay(), m_leftStream->getEGLStream(),
        EGL_STREAM_STATE_KHR, &streamState) || (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
      CONSUMER_PRINT("left : EGL_STREAM_STATE_DISCONNECTED_KHR received\n");
      break;
    }

    if (!eglQueryStreamKHR(m_rightStream->getEGLDisplay(), m_rightStream->getEGLStream(),
        EGL_STREAM_STATE_KHR, &streamState) || (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)) {
      CONSUMER_PRINT("right : EGL_STREAM_STATE_DISCONNECTED_KHR received\n");
      break;
    }

    CudaFrameAcquire left_acq(m_cuStreamLeft, left_pub_);
    CudaFrameAcquire right_acq(m_cuStreamRight, right_pub_);

    PROPAGATE_ERROR(left_acq.publish() && right_acq.publish());
  }
    
  CONSUMER_PRINT("No more frames. Cleaning up.\n");
  PROPAGATE_ERROR(requestShutdown());
  return true;
}

bool StereoConsumer::threadShutdown() {
  CUresult cuResult = cuEGLStreamConsumerDisconnect(&m_cuStreamLeft);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to disconnect CUDA stream (%s)", getCudaErrorString(cuResult));
  }

  cuResult = cuEGLStreamConsumerDisconnect(&m_cuStreamRight);
  if (cuResult != CUDA_SUCCESS) {
    ORIGINATE_ERROR("Unable to disconnect CUDA stream (%s)", getCudaErrorString(cuResult));
  }

  PROPAGATE_ERROR(cleanupCUDA(&m_cudaContext));
  CONSUMER_PRINT("Done.\n");
  return true;
}


}