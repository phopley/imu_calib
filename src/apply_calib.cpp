/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Daniel Koch
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/**
 * \file apply_calib.cpp
 * \author Daniel Koch <daniel.p.koch@gmail.com>
 *
 * Class for applying a previously computed calibration to IMU data
 */

#include "imu_calib/apply_calib.h"

namespace imu_calib
{

ApplyCalib::ApplyCalib() :
  gyro_sample_count_(0),
  gyro_bias_x_(0.0),
  gyro_bias_y_(0.0),
  gyro_bias_z_(0.0),
  accel_sample_count_(0),
  accel_bias_x_(0.0),
  accel_bias_y_(0.0),
  accel_bias_z_(0.0)
{
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string calib_file;
  nh_private.param<std::string>("calib_file", calib_file, "imu_calib.yaml");

  if (!calib_.loadCalib(calib_file) || !calib_.calibReady())
  {
    ROS_FATAL("Calibration could not be loaded");
    ros::shutdown();
  }

  nh_private.param<bool>("calibrate_gyros", calibrate_gyros_, true);
  nh_private.param<int>("gyro_calib_samples", gyro_calib_samples_, 100);
  
  nh_private.param<bool>("null_accelerometer", null_accelerometer_, true);
  nh_private.param<int>("null_accelerometer_samples", null_accelerometer_samples_, 100);  

  raw_sub_ = nh.subscribe("/imu/data_raw", 5, &ApplyCalib::rawImuCallback, this);
  corrected_pub_ = nh.advertise<sensor_msgs::Imu>("/imu/data", 5);
}

void ApplyCalib::rawImuCallback(sensor_msgs::Imu::ConstPtr raw)
{
  if (calibrate_gyros_)
  {
    ROS_INFO_ONCE("Calibrating gyros; do not move the IMU");

    // recursively compute mean gyro measurements
    gyro_sample_count_++;
    gyro_bias_x_ = ((gyro_sample_count_ - 1) * gyro_bias_x_ + raw->angular_velocity.x) / gyro_sample_count_;
    gyro_bias_y_ = ((gyro_sample_count_ - 1) * gyro_bias_y_ + raw->angular_velocity.y) / gyro_sample_count_;
    gyro_bias_z_ = ((gyro_sample_count_ - 1) * gyro_bias_z_ + raw->angular_velocity.z) / gyro_sample_count_;

    if (gyro_sample_count_ >= gyro_calib_samples_)
    {
      ROS_INFO("Gyro calibration complete! (bias = [%.3f, %.3f, %.3f])", gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
      calibrate_gyros_ = false;
    }
  }
  
  if(null_accelerometer_ == true)
  {
    ROS_INFO_ONCE("Nulling accelerometer; do not move the IMU");
    
    // Recursively compute mean accelerometer measurements from corrected acceleration readings
    sensor_msgs::Imu corrected = *raw;
    accel_sample_count_++;

    calib_.applyCalib(raw->linear_acceleration.x, raw->linear_acceleration.y, raw->linear_acceleration.z,
                      &corrected.linear_acceleration.x, &corrected.linear_acceleration.y, &corrected.linear_acceleration.z);
                      
    accel_bias_x_ = ((accel_sample_count_ - 1) * accel_bias_x_ + corrected.linear_acceleration.x) / accel_sample_count_;    
    accel_bias_y_ = ((accel_sample_count_ - 1) * accel_bias_y_ + corrected.linear_acceleration.y) / accel_sample_count_;
    accel_bias_z_ = ((accel_sample_count_ - 1) * accel_bias_z_ + (corrected.linear_acceleration.z-9.80665)) / accel_sample_count_;

    if (accel_sample_count_ >= null_accelerometer_samples_)
    {
      ROS_INFO("Nulling accelerometers complete! (bias = [%.3f, %.3f, %.3f])", accel_bias_x_, accel_bias_y_, accel_bias_z_);
      null_accelerometer_ = false;
    }    
  }
  
  if((null_accelerometer_ == false) && (calibrate_gyros_ == false))
  {
    sensor_msgs::Imu corrected = *raw;

    calib_.applyCalib(raw->linear_acceleration.x, raw->linear_acceleration.y, raw->linear_acceleration.z,
                      &corrected.linear_acceleration.x, &corrected.linear_acceleration.y, &corrected.linear_acceleration.z);
                      
    corrected.linear_acceleration.x -= accel_bias_x_;
    corrected.linear_acceleration.y -= accel_bias_y_;
    corrected.linear_acceleration.z -= accel_bias_z_;

    corrected.angular_velocity.x -= gyro_bias_x_;
    corrected.angular_velocity.y -= gyro_bias_y_;
    corrected.angular_velocity.z -= gyro_bias_z_;

    corrected_pub_.publish(corrected);
  }
}

} // namespace accel_calib
