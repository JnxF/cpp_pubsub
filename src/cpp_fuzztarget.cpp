// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

std::string get_all_input()
{
  std::string res = "";
  std::string line;
  bool first = true;
  while (std::getline(std::cin, line)) {
    if (first) first = false;
    else res.append("\n");
    res.append(line);
  }
  return res;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("talker");
  auto chatter_pub = node->create_publisher<std_msgs::msg::String>("topic", 10);
  rclcpp::Rate loop_rate(62500);

  while (rclcpp::ok()) {
    int connections = chatter_pub->get_subscription_count();
    if (connections > 0) {
      auto msg = std_msgs::msg::String();
      msg.data = get_all_input();
      chatter_pub->publish(msg);
      // std::cout << "Publishing" << std::endl;
      break;
    } 
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  // std::cout << "Dying" << std::endl;
  return 0;
}
