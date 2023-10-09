#include "../inc/main.h"
#include "tensorRT/builder/trt_builder.hpp"
#include "application/app_yolo/yolo.hpp"
#include "application/app_yolo/multi_gpu.hpp"
using namespace std;
cv::Mat cv_color, cv_color1, cv_depth, cv_infrared, depthFrameSizeAsRGB, rgbFrame, depthFrame;
cv::Mat color, mask, contour;
k4a::capture capture;
k4a::device device;
k4a::transformation k4aTransformation;
k4a::calibration k4aCalibration;
std::vector<cv::Vec4f> plines;
std::shared_future<Yolo::BoxArray> prefuture;

int main(int argc, char const *argv[])
{
  camera camera;

  camera.init_kinect(capture, k4aTransformation, k4aCalibration);
  // 编译模型时取消注释
  // TRT::compile(
  //     TRT::Mode::FP16,
  //     1,
  //     "/home/ddxy/Downloads/kinect4/workspace/best.onnx",
  //     "ball.trtmodel");
  // INFO("Done");
  auto yoloEngine = Yolo::create_infer("ball.trtmodel", Yolo::Type::V5, 0, 0.8f, 0.5f);
  while (true)
  {
    auto start = std::chrono::system_clock::now();
    camera.getpicture(capture, cv_color1, cv_color, cv_depth, k4aTransformation);
    // cameraCVs.getColor(cv_color, mask, color);
    // cv_depth.copyTo(depthCut, mask);
    // lclouds.getMaskAccordingToColor(cv_color, mask);
    // lclouds.getXYZPointCloud(k4aTransformation, k4aCalibration, depthCut);
    // lclouds.getPLY();
    if (prefuture.valid())
    {
      auto yoloStart = std::chrono::system_clock::now();
      prefuture = yoloEngine->commit(cv_color);
      auto bboxes = prefuture.get();
      auto yoloEnd = std::chrono::system_clock::now();
      auto yoloDuration = std::chrono::duration_cast<std::chrono::microseconds>(yoloEnd - yoloStart);
      int imgHeight = cv_color.rows;
      int imgWidth = cv_color.cols;
      int left, top, right, bottom;

      int i = 0;
      int NUM = bboxes.size();
      std::vector<lcloud> l(NUM);
      std::vector<cameraCV> cameraCVs(NUM);
      std::vector<lcloud> lclouds(NUM);
      std::vector<thread> threads(NUM);
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

        cv::Rect select = cv::Rect(left, top, right - left, bottom - top); // yolo障碍物检测检测到的区域
        cameraCVs[i].getColor(cv_color(select), mask, color);
        cv::Mat depthCut = cv::Mat::zeros(cv::Size(imgWidth, imgHeight), CV_16U);
        cv_depth(select).copyTo(depthCut(select), mask);
        // lclouds[i].getMaskAccordingToColor(cv_color, mask);
        lclouds[i].getXYZPointCloud(k4aTransformation, k4aCalibration, depthCut);
        int l = 10;
        for (i = 0; i < l; i++)
          thread t[l];
        {
          threads[i] = thread(&lcloud::getPLY, i);
          // threads[i] = thread(&lcloud::getPLY, &lclouds[i], ref(l[i]), cv_color(select));
          // threads[i] = thread(&lcloud::getPLY, ref(l), cv_color(select));
          // thread t(&lcloud::getPLY, &node, 28);
          // lclouds[i].getPLY();
        }
        for (int i = 0; i < l; i++)
        {
          threads[i].join();
        }

        uint8_t r, g, b;
        std::tie(r, g, b) = iLogger::random_color(box.class_label);
        cv::rectangle(cv_color, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(b, g, r), 3);

        lclouds[i].clearCloud();
        i++;
      }
      for (auto &entry : threads)
        entry.join();
      threads.clear();

      cv::imshow("rgb", cv_color);
      cv::imshow("depth", cv_depth);
      // cv::waitKey(1);
    }
    else
    {
      prefuture = yoloEngine->commit(cv_color);
    }
    // cv::imshow("mask", mask);

    // cv::imshow("depth", depthCut);
    // cameraCVs.getColor(cv_color, mask, color);
    // cameraCVs.getContour(color, contour);
    // cameraCVs.detectStraightLine(contour, plines, cv_color);
    // cv::imshow("color", color);
    // cv::imshow("contour", contour);
    // cv::imshow("lines", cv_color);

    // color.release();
    // contour.release();
    cv_color.release();
    cv_depth.release();
    capture.reset();
    // lclouds.clearCloud();

    if (cv::waitKey(1) == 27)
      break;
  }
  cv::destroyAllWindows();
  camera.stopCamera();
  return 0;
}
