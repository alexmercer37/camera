#pragma once

#include "camera.h"
#include "../inc/opencv.h"
#include <sys/types.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "../inc/pointcloud.h"
#include "common/ilogger.hpp"
#include "builder/trt_builder.hpp"
#include "app_yolo/yolo.hpp"
#include "app_yolo/multi_gpu.hpp"
#include "app_yolo/yolo.hpp"

extern cv::Mat rgbFrame, depthFrame, color, mask, contour;
extern k4a::capture capture;
extern std::vector<cv::Vec4f> plines;