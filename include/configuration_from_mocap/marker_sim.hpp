#ifndef CONFIGURATION_FROM_MOCAP_MARKER_SIM
#define CONFIGURATION_FROM_MOCAP_MARKER_SIM

#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "mocap4ros_msgs/msg/marker.hpp"
#include "mocap4ros_msgs/msg/markers.hpp"
#include "mocap4ros_msgs/msg/marker_with_id.hpp"
#include "mocap4ros_msgs/msg/markers_with_id.hpp"

#include "configuration_from_mocap/marker_lib.hpp"

#include <urdf/model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

typedef mocap4ros_msgs::msg::MarkerWithId MocapMarkerWithId;
typedef mocap4ros_msgs::msg::MarkersWithId MocapMarkers;

class MarkerSim : public rclcpp::Node
{
public:
  MarkerSim(rclcpp::Clock::SharedPtr);
  void updateTransform(const std::string& frame);
  void timer_callback();
private:
  rclcpp::Publisher<MocapMarkers>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  urdf::Model robot_model_;
  std::string source_frame_;
  std::string global_frame_;
  std::multimap<std::string, std::shared_ptr<marker_lib::Marker>> stored_markers_;
  std::set<std::string> tracked_frames_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<std::thread> threads_;
  std::mutex mutex_;
  std::multimap<std::string, MocapMarkerWithId> latched_markers_;
};

#endif
