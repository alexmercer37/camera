#ifndef _OPENCVHEAD_H
#define _OPENCVHEAD_H
#include "OpencvHead.h"
#include <iostream>
#include <cstdio>
#include <string.h>
class cameraCV
{
public:
    void getContour(cv::Mat &input, cv::Mat &output);
    void detectStraightLine(cv::Mat &contour, std::vector<cv::Vec4f> &plines, cv::Mat &output);
    void getColor(const cv::Mat &input, cv::Mat &mask, cv::Mat &output);

private:
    cv::Mat hsv, gray, median, laplacian, lap8BitFrame;
};

#endif