#ifndef _OPENCVHEAD_H
#define _OPENCVHEAD_H
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
class opencv
{
public:
    opencv();
    void getContour(cv::Mat &input, cv::Mat &output);
    void detectStraightLine(cv::Mat &contour, std::vector<cv::Vec4f> &plines, cv::Mat &output);
    void getColor(const cv::Mat &input, cv::Mat &mask, cv::Mat &output);

private:
    cv::Mat hsv, gray, median, laplacian, lap8BitFrame;
};

#endif