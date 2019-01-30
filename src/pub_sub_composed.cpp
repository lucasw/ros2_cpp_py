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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <ros2_cpp_py/pub_test.hpp>
#include <ros2_cpp_py/sub_test.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto pub = std::make_shared<Color>();
  auto sub = std::make_shared<SubTest>();
  // rclcpp::spin(pub);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pub);
  executor.add_node(sub);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

