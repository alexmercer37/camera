/*
 * @Author: Tommy0929
 * @Date: 2023-05-02 15:02:48
 * @LastEditors: Tommy0929 tommy07210728@163.com
 * @FilePath: /kinectCpp2/src/main.cpp
 * WHUROBOCON_SAVED!!!
 */
#include "inc/main.h"
#include "tensorRT/builder/trt_builder.hpp"
#include "application/app_yolo/yolo.hpp"
#include "application/app_yolo/multi_gpu.hpp"

cv::Mat rgbFrame, depthFrame, irFrame, depthFrameSizeAsRGB;
cv::Mat color, mask, contour;
k4a::capture capture;
k4a::transformation k4aTransformation;
k4a::calibration k4aCalibration;
std::vector<cv::Vec4f> plines;
std::shared_future<Yolo::BoxArray> prefuture;

int main(int argc, char const *argv[])
{
  Camera camera;
  camera.cameraInit(capture, k4aTransformation, k4aCalibration);
  // 编译模型时取消注释
  //  TRT::compile(
  //      TRT::Mode::FP16,
  //      1,
  //      "/home/tommy0929/Desktop/kinectCpp2/workspace/yolov5s.onnx",
  //      "yolov5s.trtmodel");
  //  INFO("Done");
  auto yoloEngine = Yolo::create_infer("yolov5s.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);
  while (true)
  {
    auto start = std::chrono::system_clock::now();
    camera.pictureTransformation(capture, rgbFrame, depthFrame, k4aTransformation);
    // cameraCV.getColor(rgbFrame, mask, color);
    // depthFrame.copyTo(depthCut, mask);
    // cameraPCL.getXYZPointCloud(k4aTransformation, k4aCalibration, depthCut);
    // cameraPCL.pclManager();
    if (prefuture.valid())
    {
      auto yoloStart = std::chrono::system_clock::now();
      prefuture = yoloEngine->commit(rgbFrame);
      auto bboxes = prefuture.get();
      auto yoloEnd = std::chrono::system_clock::now();
      auto yoloDuration = std::chrono::duration_cast<std::chrono::microseconds>(yoloEnd - yoloStart);
      int imgHeight = rgbFrame.rows;
      int imgWidth = rgbFrame.cols;
      int left, top, right, bottom;

      int i = 0;
      int NUM = bboxes.size();
      std::vector<cameraCV> cameraCVs(NUM);
      std::vector<cameraPCL> cameraPCLs(NUM);
      for (auto &box : bboxes)
      {
        left = (int)box.left;
        if (left < 0)
          left = 0;
        top = (int)box.top;
        if (top < 0)
          top = 0;
        right = (int)box.right;
        if (right > imgWidth)
          right = imgWidth;
        bottom = (int)box.bottom;
        if (bottom > imgHeight)
          bottom = imgHeight;

        cv::Rect selection = cv::Rect(left, top, right - left, bottom - top); // yolo障碍物检测检测到的区域
        cameraCVs[i].getColor(rgbFrame(selection), mask, color);
        cv::Mat depthCut = cv::Mat::zeros(cv::Size(imgWidth, imgHeight), CV_16U);
        depthFrame(selection).copyTo(depthCut(selection), mask);
        cameraPCLs[i].getXYZPointCloud(k4aTransformation, k4aCalibration, depthCut);
        cameraPCLs[i].pclManager();

        uint8_t r, g, b;
        std::tie(r, g, b) = iLogger::random_color(box.class_label);
        cv::rectangle(rgbFrame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(b, g, r), 3);

        cameraPCLs[i].clearCloud();
        i++;
      }
      cv::imshow("rgb", rgbFrame);
      cv::imshow("depth", depthFrame);
      // cv::waitKey(1);
    }
    else
    {
      prefuture = yoloEngine->commit(rgbFrame);
    }
    // cv::imshow("mask", mask);

    // cv::imshow("depth", depthCut);
    // cameraCV.getColor(rgbFrame, mask, color);
    // cameraCV.getContour(color, contour);
    // cameraCV.detectStraightLine(contour, plines, rgbFrame);
    // cv::imshow("color", color);
    // cv::imshow("contour", contour);
    // cv::imshow("lines", rgbFrame);

    // color.release();
    // contour.release();
    rgbFrame.release();
    depthFrame.release();
    capture.reset();
    // cameraPCL.clearCloud();

    if (cv::waitKey(1) == 27)
      break;
  }
  cv::destroyAllWindows();
  camera.stopCamera();
  return 0;
}
