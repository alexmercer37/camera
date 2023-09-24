#include "../inc/cameraGyro.h"

using namespace std;

kinectGyro::kinectGyro()
    : zero_arry_number(0), zero_arry_add(0.0), zero_point(0), first_zero_flag(0), angle(0.0), zero_arry_now_position(0), gyro_data_arry_position(0), now_gyro_v_data(0.0)
{
  comb_filter = new comb_fil<float>();
  kal_filter = new kal_fil<float>(0.005, 0.0001);
}

void kinectGyro::updateZeroPoint()
{
  if (zero_arry_number == zero_arry_maxsize)
  {
    zero_arry_add -= zero_arry[zero_arry_now_position];
    zero_arry_add += now_gyro_v_data;
    zero_point = zero_arry_add / zero_arry_maxsize;
    zero_arry[zero_arry_now_position] = now_gyro_v_data;
    (++zero_arry_now_position) %= zero_arry_maxsize;
  }
}

void kinectGyro::recieveGyroData(const k4a_imu_sample_t *imu_data)
{
  // 零点数据
  if (zero_arry_number < zero_arry_maxsize)
  {
    zero_arry[zero_arry_now_position] = imu_data->gyro_sample.xyz.z;
    zero_arry_now_position++;
    zero_arry_number++;
    zero_arry_add += imu_data->gyro_sample.xyz.z;
    return;
  }

  // 计算零点
  if (first_zero_flag == 0)
  {
    zero_point = (float)zero_arry_add / zero_arry_maxsize;
    first_zero_flag = 1;
    kal_filter->x_last = zero_point;
    cout << "首次零点更新完成" << endl;
  }

  // 非零点数据
  if (gyro_data_arry_position < data_arry_maxsize)
  {
    gyro_data_arry[gyro_data_arry_position] = imu_data->gyro_sample.xyz.z;
    comb_filter->data[gyro_data_arry_position] = imu_data->gyro_sample.xyz.z;
    gyro_data_arry_position++;
  }
}

void kinectGyro::dealGyroData()
{
  if (gyro_data_arry_position != data_arry_maxsize)
    return;
  gyro_data_arry_position = 0;
  // 滤波
  kal_filter->input = comb_filter->combination_filter();
  now_gyro_v_data = kal_filter->kalman_filter();
  if (now_gyro_v_data > gyro_zero_thread || now_gyro_v_data < 0) // 数据有效
  {
    if (now_gyro_v_data > 0)
      angle += (double)(0.01 * now_gyro_v_data * 180.0f / PI * 2.175336); // 1.07619
    else
      angle += (double)(0.01 * now_gyro_v_data * 180.0f / PI * 2.206139); // 1.0956
    if (angle > 360)
      angle -= 360;
    else if (angle < -360)
      angle += 360;
    cout << angle << endl;
  }
  else // 用于更新零点
  {
    updateZeroPoint();
  }
}
