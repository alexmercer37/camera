#include "../inc/camera.h"

using namespace std;
using namespace k4a;
using namespace cv;

lcloud *cloud = new lcloud;

void Camera::init_kinect(uint32_t &device_count, k4a::device &device, k4a::capture &capture, k4a_device_configuration_t &init)
{
  device_count = device::get_installed_count();
  if (device_count == 0)
  {
    cout << "Error:no K4A devices found." << endl;
    return;
  }
  else
  {
    cout << "Found" << device_count << "connected devices." << endl;
  }
  device = device::open(K4A_DEVICE_DEFAULT);
  cout << "Done:open device." << endl;
  init = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
  init.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; // A是alpha，具有alpha纹理格式的颜色
  init.color_resolution = K4A_COLOR_RESOLUTION_720P; // 1920*1080
  init.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;    // 640*576
  init.camera_fps = K4A_FRAMES_PER_SECOND_30;        // 30帧
  init.synchronized_images_only = true;              // 只支持同步图像

  device.start_cameras(&init);
  cout << "Done:start camera." << endl;
  device.start_imu();
  cout << "Done:start imu." << endl;

  int iAuto = 0;
  while (1)
  {
    if (device.get_capture(&capture))
      cout << iAuto << ". Capture several frames to give auto-exposure" << endl;
    if (iAuto < 30)
    {
      iAuto++;
      continue;
    }
    else
    {
      cout << "Done: auto-exposure" << endl;
      break;
    }
  }
}
void Camera::getpicture(k4a::capture &capture, cv::Mat &cv_depth, cv::Mat &cv_color1, cv::Mat &cv_infrared, cv::Mat &cv_color, k4a::transformation &k4aTransformation)
{
  k4a::device device;
  if (device.get_capture(&capture, std::chrono::milliseconds(0)))
  {
    uint32_t device_count;
    k4a_device_configuration_t init;

    image k4a_color;
    image k4a_depth;
    image k4a_infrared;
    image k4a_tf_depth;

    k4a_color = capture.get_color_image();
    k4a_depth = capture.get_depth_image();
    k4a_infrared = capture.get_ir_image();
    k4a_tf_depth = k4aTransformation.depth_image_to_color_camera(k4a_depth);

    cv_color1 = Mat(k4a_color.get_height_pixels(), k4a_color.get_width_pixels(), CV_8UC4, k4a_color.get_buffer());
    GaussianBlur(cv_color1, cv_color, cv::Size(5, 5), 3, 3);
    // bilateralFilter(cv_color1, cv_color, 9, 50, 5);
    cv_depth = Mat(k4a_tf_depth.get_height_pixels(), k4a_tf_depth.get_width_pixels(), CV_16U, k4a_tf_depth.get_buffer());
    cv_infrared = Mat(k4a_infrared.get_height_pixels(), k4a_infrared.get_width_pixels(), CV_16U, k4a_infrared.get_buffer());
    cvtColor(cv_color, cv_color, cv::COLOR_BGRA2BGR);
    // cv_depth.convertTo(cv_depth, CV_8U, 1);
    // cv_infrared.convertTo(cv_infrared, CV_8U, 1);
  }
}
void Camera::stopCamera()
{
  k4a::device device;
  device.close();
}
