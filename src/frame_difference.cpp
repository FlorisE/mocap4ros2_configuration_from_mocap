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

#include <functional>
#include <utility>
#include <chrono>

#include "configuration_from_mocap/frame_difference.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

ConfigurationFromMocap::FrameDifference::FrameDifference(rclcpp::Clock::SharedPtr clock) :
  rclcpp::Node("frame_difference"),
  clock_(clock),
  buffer_(std::make_shared<tf2_ros::Buffer>(clock_, tf2::Duration(100ms))),
  listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_))
{
  declare_parameter<std::string>("source_frame", std::string("base_link"));
  get_parameter<std::string>("source_frame", source_frame_);
  declare_parameter<std::string>("global_frame", std::string("mocap"));
  get_parameter<std::string>("global_frame", global_frame_);

  std::vector<std::string> frames{"link1", "link2", "link3", "link4", "link5", "link6", "ee"};

  pub_ = create_publisher<tf2_msgs::msg::TFMessage>("frame_difference", 1);

  for (const std::string& frame : frames)
  {
    threads_.push_back( std::thread( std::bind( &FrameDifference::updateTransform, this, _1 ), frame ) );
  }

  timer_ = this->create_wall_timer(5ms, std::bind(&FrameDifference::timerCallback, this));
}

void ConfigurationFromMocap::FrameDifference::updateTransform(const std::string& frame)
{
  auto duration = 5ms;
  while (rclcpp::ok())
  {
    auto start = std::chrono::high_resolution_clock::now();
    try
    {
      auto transform = buffer_->lookupTransform(frame + "_estimated", frame, rclcpp::Clock().now(), tf2::durationFromSec(0.02));
      mutex_.lock();
      changed_ = true;
      latched_transforms_.erase(frame);
      latched_transforms_.insert(std::make_pair(frame, transform));
      mutex_.unlock();
    }
    catch ( tf2::TransformException& ex )
    {
      RCLCPP_WARN( get_logger(), "%s", ex.what() );
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto ellapsed = end - start;
    if (ellapsed < duration)
    {
      std::this_thread::sleep_for(duration - ellapsed);
    }
  }
}

void ConfigurationFromMocap::FrameDifference::timerCallback()
{
  tf2_msgs::msg::TFMessage tf_diff;
  mutex_.lock();
  if (changed_)
  {
    for (auto& transform : latched_transforms_)
    {
      tf_diff.transforms.push_back(transform.second);
    }
    changed_ = false;
  }
  mutex_.unlock();
  pub_->publish(tf_diff);
  return;
}
