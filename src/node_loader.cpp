/*
 * Copyright (c) 2019 Lucas Walter
 * February 2019
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <class_loader/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>


// based on demos/composition/api_composition.cpp 
struct NodeLoader : public rclcpp::Node
{
  NodeLoader() : Node("node_loader")
  {

  }

  bool addNode(const std::string& library_path, const std::string& class_name)
  {
    std::shared_ptr<class_loader::ClassLoader> loader;
    try {
      loader = std::make_shared<class_loader::ClassLoader>(library_path);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load library: %s", ex.what());
      return false;
    }
    auto node = loader->createInstance<rclcpp::Node>(class_name);

  }

  std::vector<std::shared_ptr<class_loader::ClassLoader> > loaders_;
  std::vector<std::shared_ptr<rclcpp::Node> > nodes_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto node_loader = std::make_shared<NodeLoader>();
  rclcpp::spin(node_loader);
  rclcpp::shutdown();
}
