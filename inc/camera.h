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

class camera
{
public:
    void init_kinect(k4a::capture &capture, k4a::transformation &k4aTransformation, k4a::calibration &k4aCalibration);
    void getpicture(k4a::capture &capture, cv::Mat &cv_color1, cv::Mat &cv_color, cv::Mat &cv_depth, k4a::transformation &k4aTransformation);
    void getpicture(k4a::capture &capture, cv::Mat &cv_color, cv::Mat &cv_depth, k4a::transformation &k4aTransformation);
    void pictureTransformation(k4a::capture &capture, cv::Mat &rgbFrame, cv::Mat &depthFrame, k4a::transformation &k4aTransformation);
    void stopCamera();

private:
    u_int32_t device_count;
    k4a::device device;
    k4a_device_configuration_t init;

    k4a::image k4a_color;
    k4a::image k4a_depth;
    k4a::image k4a_infrared;
    k4a::image k4a_tf_depth;
};
#endif