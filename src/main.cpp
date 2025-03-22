// #include <csignal>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
// #include "sensor_msgs/image_encodings.hpp"

#include <Argus/Argus.h>

// #include <EGLStream/EGLStream.h>
// #include <cuda.h>
// #include <cudaEGL.h>

// #include "ArgusHelpers.h"
// #include "CUDAHelper.h"
#include "EGLGlobal.h"
#include "Error.h"
// #include "Thread.h"

#include "argus_stereo_sync/constants.h"
// #include "argus_stereo_sync/convert.h"
#include "argus_stereo_sync/stereo_consumer.h"
// #include "argus_stereo_sync/cuda_frame_acquire.h"


uint8_t* oBuffer = new uint8_t[3 * STREAM_SIZE.width() * STREAM_SIZE.height()];


namespace argus_stereo_sync {

  using namespace Argus;
  // using namespace ArgusSamples;

class ArgusStereoSyncNode : public rclcpp::Node {
  public:
  ArgusStereoSyncNode(const std::string &node_name,
                       const rclcpp::NodeOptions &options)
                       : Node(node_name, options),
                       camera_provider_(CameraProvider::create())
                       {

                        left_image_pub = create_publisher<sensor_msgs::msg::Image>("left/image_raw", 1);
                        left_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", 1);
                        right_image_pub = create_publisher<sensor_msgs::msg::Image>("right/image_raw", 1);
                        right_camera_info_pub = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info", 1);

                       }

   virtual ~ArgusStereoSyncNode()
   {
    iCaptureSession->stopRepeat();
    iCaptureSession->waitForIdle();
  
    RCLCPP_INFO(get_logger(), "Captures complete, disconnecting producer.");
    iStreamLeft->disconnect();
    iStreamRight->disconnect();
  
    if(stereo_consumer_) PROPAGATE_ERROR_CONTINUE(stereo_consumer_->shutdown());
    if(camera_provider_) camera_provider_.reset();
    PROPAGATE_ERROR_CONTINUE(g_display.cleanup());
  
    RCLCPP_INFO(get_logger(),"Done -- exiting.");

   }

bool execute() {
  PROPAGATE_ERROR(g_display.initialize());

  ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(camera_provider_);
  if (!iCameraProvider) {
    ORIGINATE_ERROR("Failed to get ICameraProvider interface");
  }
  RCLCPP_INFO(get_logger(),"Argus Version: %s", iCameraProvider->getVersion().c_str());

  iCameraProvider->getCameraDevices(&cameraDevices);
  RCLCPP_INFO(get_logger(),"CAMERA DEVICES COUNT: %lu", cameraDevices.size());
  if (cameraDevices.size() < 2) {
    ORIGINATE_ERROR("Must have at least 2 sensors available");
  }

  std::vector <CameraDevice*> lrCameras;
  lrCameras.push_back(cameraDevices[0]);
  lrCameras.push_back(cameraDevices[1]);

  captureSession.reset(iCameraProvider->createCaptureSession(lrCameras));
  iCaptureSession = interface_cast<ICaptureSession>(captureSession);
  if (!iCaptureSession) {
    ORIGINATE_ERROR("Failed to get capture session interface");
  }

  UniqueObj<OutputStreamSettings> streamSettings(
      iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
  IOutputStreamSettings *iStreamSettings =
      interface_cast<IOutputStreamSettings>(streamSettings);
  IEGLOutputStreamSettings *iEGLStreamSettings =
      interface_cast<IEGLOutputStreamSettings>(streamSettings);
  if (!iStreamSettings || !iEGLStreamSettings) {
    ORIGINATE_ERROR("Failed to create OutputStreamSettings");
  }
  iEGLStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
  iEGLStreamSettings->setResolution(STREAM_SIZE);
  iEGLStreamSettings->setEGLDisplay(g_display.get());
  iEGLStreamSettings->setMode(EGL_STREAM_MODE_MAILBOX);
  iEGLStreamSettings->setMetadataEnable(true);

  RCLCPP_INFO(get_logger(),"Creating left stream.");
  iStreamSettings->setCameraDevice(lrCameras[0]);
  streamLeft.reset(iCaptureSession->createOutputStream(streamSettings.get()));
  iStreamLeft=interface_cast<IEGLOutputStream>(streamLeft);
  if (!iStreamLeft) {
    ORIGINATE_ERROR("Failed to create left stream");
  }

  RCLCPP_INFO(get_logger(),"Creating right stream.");
  iStreamSettings->setCameraDevice(lrCameras[1]);
  streamRight.reset(iCaptureSession->createOutputStream(streamSettings.get()));
  iStreamRight = interface_cast<IEGLOutputStream>(streamRight);
  if (!iStreamRight) {
    ORIGINATE_ERROR("Failed to create right stream");
  }

  request.reset(iCaptureSession->createRequest());
  IRequest *iRequest = interface_cast<IRequest>(request);
  if (!iRequest) {
    ORIGINATE_ERROR("Failed to create Request");
  }

  iRequest->enableOutputStream(streamLeft.get());
  iRequest->enableOutputStream(streamRight.get());

  ISourceSettings *iSourceSettings = interface_cast<ISourceSettings>(request);
  if (!iSourceSettings) {
    ORIGINATE_ERROR("Failed to get source settings request interface");
  }
  iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9 / FRAMERATE));
  iSourceSettings->setExposureTimeRange(EXPOSURE_TIME_RANGE);
  iSourceSettings->setGainRange(GAIN_RANGE);
  
  IAutoControlSettings *iAutoControlSettings = 
	  interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
  iAutoControlSettings->setIspDigitalGainRange(ISP_DIGITAL_GAIN_RANGE);

  RCLCPP_INFO(get_logger(),"Stereo consumer");
  stereo_consumer_ = std::make_shared<StereoConsumer>(iStreamLeft, iStreamRight, left_image_pub, right_image_pub);

  PROPAGATE_ERROR(stereo_consumer_->initialize());
  PROPAGATE_ERROR(stereo_consumer_->waitRunning());

  RCLCPP_INFO(get_logger(),"Starting repeat capture requests.");
  if (iCaptureSession->repeat(request.get()) != STATUS_OK) {
    ORIGINATE_ERROR("Failed to start repeat capture request for preview");
  }

  return true;
}


protected:

ArgusSamples::EGLDisplayHolder g_display;
UniqueObj<CaptureSession> captureSession;
std::vector<CameraDevice*> cameraDevices;

UniqueObj<CameraProvider> camera_provider_;
UniqueObj<Request> request;
std::shared_ptr<StereoConsumer> stereo_consumer_;

UniqueObj<OutputStream> streamLeft, streamRight;
IEGLOutputStream *iStreamRight, *iStreamLeft;
ICaptureSession *iCaptureSession;

rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_camera_info_pub;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_camera_info_pub;

};



} // namespace argus_stereo_sync

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<argus_stereo_sync::ArgusStereoSyncNode>("argus_stereo_sync", rclcpp::NodeOptions());
  if(!node->execute()) {
      return -1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;

 
  // if (!ArgusSamples::execute()) {
  //   delete[] oBuffer;
  //   return EXIT_FAILURE;
  // }
  // delete[] oBuffer;
  // return EXIT_SUCCESS;
}
