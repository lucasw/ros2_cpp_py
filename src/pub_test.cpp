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

  double frame_rate_ = 20.0;
  void updateTimer();

  bool dirty_ = true;
  cv::Mat image_;

public:
  Color();
};

Color::Color() : Node("color")
{
  set_parameter_if_not_set("red", red_);
  get_parameter_or("red", red_, red_);
  set_parameter_if_not_set("green", green_);
  get_parameter_or("green", green_, green_);
  set_parameter_if_not_set("blue", blue_);
  get_parameter_or("blue", blue_, blue_);
  // this works okay but would rather get width and height from
  // an input image
  set_parameter_if_not_set("width", width_);
  get_parameter_or("width", width_, width_);
  set_parameter_if_not_set("height", height_);
  get_parameter_or("height", height_, height_);
  set_parameter_if_not_set("frame_rate", frame_rate_);
  get_parameter_or("frame_rate", frame_rate_, frame_rate_);

  pub_ = create_publisher<sensor_msgs::msg::Image>("image");

  RCLCPP_INFO(get_logger(), "%d x %d",  width_, height_);
  updateTimer();
}

void Color::updateTimer()
{
  if (frame_rate_ > 0.0) {
    int period_ms = 1000.0 / frame_rate_;
    RCLCPP_INFO(get_logger(), "frame rate: %f, period ms %d", frame_rate_, period_ms);
    timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
        std::bind(&Color::pubImage, this));
  } else {
    RCLCPP_WARN(get_logger(), "setting frame rate to 0.0");
    timer_ = nullptr;
  }
}

void Color::pubImage()
{
  if (dirty_ || image_.empty()) {
    get_parameter_or("red", red_, red_);
    get_parameter_or("green", green_, green_);
    get_parameter_or("blue", blue_, blue_);
    get_parameter_or("width", width_, width_);
    get_parameter_or("height", height_, height_);

    image_ = cv::Mat(cv::Size(width_, height_), CV_8UC3);
    image_ = cv::Scalar(red_, green_, blue_);
    dirty_ = false;
  }

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = now();  // or reception time of original message?
  cv_image.image = image_;
  cv_image.encoding = "rgb8";
  // TODO(lucasw) cache the converted image message and only call toImageMsg() above
  // in if dirty.
  pub_->publish(cv_image.toImageMsg());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto color = std::make_shared<Color>();
  rclcpp::spin(color);
  rclcpp::shutdown();
  return 0;
}
