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

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(std::string input)
  : Node("cpp_fuzztarget")
  {
    round_ = 0;
    input_ = input;
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(200ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (round_ == 0) {

    } else if (round_ == 1) {
      // Send message
      auto message = std_msgs::msg::String();
      message.data = input_;
      RCLCPP_INFO(this->get_logger(), "Fuzz target publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    } else {
      exit(0);
    }

    round_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std::string input_;
  int round_;
};

std::string get_all_input() {
  std::string res = "";
  std::string line;
  bool first = true;
  while (std::getline(std::cin, line))
  {
    if (first) first = false;
    else res.append("\n");

    res.append(line);
  }
  return res;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>(get_all_input()));
  rclcpp::shutdown();
  return 0;
}
