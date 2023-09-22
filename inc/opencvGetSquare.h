#ifndef _OPENCVGETSQUARE_H
#define _OPENCVGETSQUARE_H

#include <iostream>
#include <math.h>
#include <string.h>

#include "OpencvHead.h"

typedef struct colorHist
{
    float feat;
    int x;
    int y;
} colorHist;

// 求两向量的余弦值
static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0);
// 提取指定颜色区域
void detectColor(const cv::Mat &srcImg, cv::Mat &dstImg, int Hmin, int Hmax, int Smin, int Smax, int Vmin = 0, int Vmax = 255);
// 寻找长方形(直接处理原图)
void findSquares(const cv::Mat &image, std::vector<std::vector<cv::Point>> &squares);
// 寻找长方形（提取颜色后再处理）
void findSquaresAfterDetectColor(const cv::Mat &srcImg, std::vector<std::vector<cv::Point>> &squares);
// 把长方形画在图上
void drawSquares(cv::Mat &image, const std::vector<std::vector<cv::Point>> &squares);
// 把直线画在图上
void drawLines(cv::Mat &image, const std::vector<cv::Vec4f> &plines);

#endif