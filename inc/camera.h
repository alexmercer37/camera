#ifndef CAMERA_H
#define CAMERA_H
#include <iostream>
#include <k4a/k4a.hpp>
#include "opencv.h"
#include </usr/local/include/opencv4/opencv2/imgproc.hpp>
#include "../inc/pointcloud.h"
class Camera
{
public:
    Camera();
    void init_kinect(uint32_t &device_count, k4a::device &device, k4a::capture &capture, k4a_device_configuration_t &init);
    void getpicture(k4a::capture &capture, cv::Mat &cv_depth, cv::Mat &cv_color1, cv::Mat &cv_infrared, cv::Mat &cv_color, k4a::transformation &k4aTransformation);
    void stopCamera();
};

#endif