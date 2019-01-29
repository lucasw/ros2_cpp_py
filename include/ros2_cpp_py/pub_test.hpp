/**
 Copyright 2015 Lucas Walter

     This file is part of Vimjay.

    Vimjay is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Vimjay is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Vimjay.  If not, see <http://www.gnu.org/licenses/>.

*/
// borrowed this code from https://github.com/lucasw/image_manip

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
using std::placeholders::_1;

class Color : public rclcpp::Node
{
protected:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  void pubImage();

  int width_ = 1024;
  int height_ = 1024;
  int red_ = 255;
  int green_ = 255;
  int blue_= 255;

  double frame_rate_ = 40.0;
  void updateTimer();

  bool dirty_ = true;
  cv::Mat image_;

public:
  Color();
};
