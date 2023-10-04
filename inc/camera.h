#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <k4a/k4a.hpp>
#include <k4arecord/record.h>
#include <k4arecord/playback.h>
#include "../inc/OpencvHead.h"

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include <cstdlib>

#include "../inc/opencv.h"
class camera
{
public:
    void init_kinect(uint32_t &device_count, k4a::device &device, k4a::capture &capture, k4a_device_configuration_t &init);
    void getpicture(k4a::capture &capture, cv::Mat &cv_depth, cv::Mat &cv_color1, cv::Mat &cv_infrared, cv::Mat &cv_color, k4a::transformation &k4aTransformation);
    void stopCamera();
};
#endif