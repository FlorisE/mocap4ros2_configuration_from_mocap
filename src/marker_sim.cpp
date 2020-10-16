#include <chrono>
#include <memory>
#include <iostream>

#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "configuration_from_mocap/marker_sim.hpp"

#include <Eigen/Geometry>

using namespace std::chrono_literals;
using std::placeholders::_1;

MarkerSim::MarkerSim(rclcpp::Clock::SharedPtr clock) : 
  Node("marker_publisher"),
  clock_(clock),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(clock_, tf2::Duration(100ms))),
  tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
  std::string urdf_path;
  std::string marker_path;

  declare_parameter<std::string>("urdf_path", std::string("urdf.xml"));
  declare_parameter<std::string>("marker_path", std::string("markers.xml"));
  declare_parameter<std::string>("source_frame", std::string("base_link"));
  declare_parameter<std::string>("global_frame", std::string("mocap"));

  get_parameter<std::string>("urdf_path", urdf_path);
  get_parameter<std::string>("marker_path", marker_path);
  get_parameter<std::string>("source_frame", source_frame_);
  get_parameter<std::string>("global_frame", global_frame_);

  robot_model_.initFile( urdf_path );

  if (!parse_markers(marker_path, stored_markers_))
  {
    RCLCPP_ERROR(get_logger(), "Unable to parse marker definition");
    return;
  }

  RCLCPP_INFO(get_logger(), "Number of markers parsed: " + std::to_string((int) stored_markers_.size()));

  for (const std::pair<std::string, std::shared_ptr<marker_lib::Marker>>& marker : stored_markers_)
  {
    threads_.push_back(std::thread(std::bind(&MarkerSim::updateTransform, this, _1), marker.first));
  }

  RCLCPP_INFO(get_logger(), "Number of frames tracked: " + std::to_string((int) tracked_frames_.size()));

  publisher_ = this->create_publisher<MocapMarkers>("markers_with_id", 1);
  timer_ = this->create_wall_timer(10ms, std::bind(&MarkerSim::timer_callback, this));
}

void MarkerSim::updateTransform(const std::string& frame)
{
  auto markers_for_frame = stored_markers_.equal_range( frame );
  auto duration = 5ms;
  while (rclcpp::ok())
  {
    auto start = std::chrono::high_resolution_clock::now();
    geometry_msgs::msg::TransformStamped transformStamped;
    rclcpp::Time time = rclcpp::Clock().now();
    try
    {
      transformStamped = tf_buffer_->lookupTransform(global_frame_, frame, time, tf2::durationFromSec(0.005));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(get_logger(),ex.what());
      std::cout << "Transform failed" << std::endl;
      continue;
    }

    mutex_.lock();
    latched_markers_.erase(frame);
    for ( auto marker_pair = markers_for_frame.first; 
              marker_pair != markers_for_frame.second;
              marker_pair++ )
    {
      const std::shared_ptr<marker_lib::Marker> mocap_marker = marker_pair->second;
      geometry_msgs::msg::PointStamped marker_point;
      marker_point.point.x = mocap_marker->x;
      marker_point.point.y = mocap_marker->y;
      marker_point.point.z = mocap_marker->z;
      tf2::doTransform(marker_point, marker_point, transformStamped);
      MocapMarkerWithId marker;
      marker.index = mocap_marker->id;
      marker.translation.x = marker_point.point.x;
      marker.translation.y = marker_point.point.y;
      marker.translation.z = marker_point.point.z;
      latched_markers_.insert(std::make_pair(frame, marker));
    }
    mutex_.unlock();

    auto end = std::chrono::high_resolution_clock::now();
    auto ellapsed = end - start;
    if (ellapsed < duration)
    {
      std::this_thread::sleep_for(duration - ellapsed);
    }
  }
}

void MarkerSim::timer_callback()
{
  mocap4ros_msgs::msg::MarkersWithId markers;
  // lock because we might be in the progress of updating the markers
  mutex_.lock();
  for (auto pair : latched_markers_)
  {
    markers.markers.push_back(pair.second);
  }
  publisher_->publish(markers);
  mutex_.unlock();

  return;
}
