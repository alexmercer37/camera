#ifndef FILTER_H
#define FILTER_H

#include <iostream>
#define maxsize 30

// 混合滤波
template <typename T>
class comb_fil
{
private:
    T sum;
    T max;
    T min;

public:
    T average;
    T data[maxsize];
    int flag;
    comb_fil() : sum(0.0), max(0.0), min(0.0), average(0.0), flag(0){};
    T combination_filter();
};

// 卡尔曼（单状态）
template <typename L>
class kal_fil
{
private:
    int A;        // 状态转移矩阵
    int H;        // 观测矩阵
    float C_last; // 上次的过程协方差矩阵
    float kg;     // 卡尔曼增益
public:
    L x_last; // 系统上次的预测值
    L input;  // 本次测量值
    L result; // 滤波结果
    float R;  // 观测噪声
    float Q;  // 过程噪声
    kal_fil(float _R, float _Q);
    L kalman_filter();
};

#endif