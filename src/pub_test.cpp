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
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// using std::placeholders::_1;

class Color
{
protected:
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  ros::Timer timer_;
  void pubImage(const ros::TimerEvent& e);

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

Color::Color()
{
#if 0
  set_parameter_if_not_set("red", red_);
  get_parameter_or("red", red_, red_);
  set_parameter_if_not_set("green", green_);
  get_parameter_or("green", green_, green_);
  set_parameter_if_not_set("blue", blue_);
  get_parameter_or("blue", blue_, blue_);
  // this works okay but would rather get width and height from
  // an input image
#endif
  ros::param::get("~width", width_);
  ros::param::get("~height", height_);
  ros::param::get("~frame_rate", frame_rate_);

  pub_ = nh_.advertise<sensor_msgs::Image>("image", 10);

  ROS_INFO("%d x %d",  width_, height_);
  updateTimer();
}

void Color::updateTimer()
{
  if (frame_rate_ > 0.0) {
    const double period_s = 1.0 / frame_rate_;
    ROS_INFO("frame rate: %f, period s %f", frame_rate_, period_s);
    timer_ = nh_.createTimer(ros::Duration(period_s),
        &Color::pubImage, this);
  } else {
    ROS_WARN("setting frame rate to 0.0");
    // TODO(lucasw)
    // timer_ = nullptr;
  }
}

void Color::pubImage(const ros::TimerEvent& e)
{
  (void)e;
  if (dirty_ || image_.empty()) {
#if 0
    get_parameter_or("red", red_, red_);
    get_parameter_or("green", green_, green_);
    get_parameter_or("blue", blue_, blue_);
    get_parameter_or("width", width_, width_);
    get_parameter_or("height", height_, height_);
#endif
    image_ = cv::Mat(cv::Size(width_, height_), CV_8UC3);
    image_ = cv::Scalar(red_, green_, blue_);
    dirty_ = false;
  }

  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = ros::Time::now();  // or reception time of original message?
  cv_image.image = image_;
  cv_image.encoding = "rgb8";
  // TODO(lucasw) cache the converted image message and only call toImageMsg() above
  // in if dirty.
  pub_.publish(cv_image.toImageMsg());
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros1_pub_test");

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  // setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto color = std::make_shared<Color>();
  ros::spin();
  return 0;
}
