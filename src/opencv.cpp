
#include "../inc/opencv.h"

void opencv::getContour(cv::Mat &input, cv::Mat &output)
{
	cv::medianBlur(input, median, 3);
	cv::cvtColor(median, gray, cv::COLOR_BGRA2GRAY);
	cv::Laplacian(gray, laplacian, 3, 3);				// laplacian算子提取轮廓
	cv::convertScaleAbs(laplacian, lap8BitFrame);		// 将图片压缩为8位
	cv::Canny(lap8BitFrame, output, 50, 150, 3, false); // canny提取轮廓
}

void opencv::getColor(const cv::Mat &input, cv::Mat &mask, cv::Mat &output)
{
	cv::Mat dilate;
	mask = cv::Mat::zeros(input.size(), CV_8UC1);
	cv::cvtColor(input, input, cv::COLOR_BGR2HSV); // error
	cv::inRange(input, cv::Scalar(0, 0, 211), cv::Scalar(180, 30, 255), mask);	 // 白色
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); // 设置结构元
	cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);						 // 开操作
	cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);						 // 闭操作
	input.copyTo(output, mask);
	cv::imshow("mask", mask);
}

void opencv::detectStraightLine(cv::Mat &contour, std::vector<cv::Vec4f> &plines, cv::Mat &output)
{
	cv::HoughLinesP(
		contour, plines, 1, CV_PI / 180, 300, 200,
		40); // 霍夫直线检测，参数：8位单通道图像，cv::Vec4f,rho,theta,最小的直线长度，两段直线认为是一根直线的最小的距离
	for (size_t i = 0; i < plines.size(); i++)
	{
		cv::Vec4f hlines = plines[i];
		cv::line(output, cv::Point(hlines[0], hlines[1]), cv::Point(hlines[2], hlines[3]), cv::Scalar(255, 0, 0), 3,
				 cv::LINE_AA);
	}
}
