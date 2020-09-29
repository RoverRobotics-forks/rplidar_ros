/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *
 */


#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
using std::placeholders::_1;

#define M_PI 3.1415926535897932384626433832795
#define RAD2DEG(x) ((x)*180./M_PI)

class RplidarNodeClient : public rclcpp::Node
{
  public:
    RplidarNodeClient()
    : Node("rplidar_node_client")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan/", 10, std::bind(&RplidarNodeClient::scan_callback, this, _1));
    }

  private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) const
    {
      int count = scan->scan_time / scan->time_increment;
      RCLCPP_INFO(this->get_logger(), "I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
      RCLCPP_INFO(this->get_logger(), "angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

      for(int i = 0; i < count; i++) {
          float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
          RCLCPP_INFO(this->get_logger(), ": [%f, %f]", degree, scan->ranges[i]);
      }
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RplidarNodeClient>());
  rclcpp::shutdown();
  return 0;
}
