// Copyright 2020 National Institute of Advanced Industrial Science and Technology, Japan
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
//
// Author: Floris Erich <floris.erich@aist.go.jp>

#ifndef CONFIGURATION_FROM_MOCAP__FRAME_DIFFERENCE_HPP_
#define CONFIGURATION_FROM_MOCAP__FRAME_DIFFERENCE_HPP_

#include <mutex>
#include <thread>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ConfigurationFromMocap
{

class FrameDifference : public rclcpp::Node
{
public:
  FrameDifference(rclcpp::Clock::SharedPtr);
private:
  void timerCallback();
  void updateTransform(const std::string& frame);
  std::string source_frame_;
  std::string global_frame_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
  std::mutex mutex_;
  std::vector<std::thread> threads_;
  std::multimap<std::string, geometry_msgs::msg::TransformStamped> latched_transforms_;
  bool changed_;
};

}

#endif
