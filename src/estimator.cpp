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
#include <memory>
#include <map>
#include <string>
#include <utility>
#include <chrono>

#include "configuration_from_mocap/geometry_lib.h"
#include "configuration_from_mocap/estimator.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

typedef std::shared_ptr<marker_lib::Marker> MarkerPtr;
typedef std::multimap<std::string, MarkerPtr> JointMarkerMultimap;
typedef std::multimap<std::string, MarkerPtr>::const_iterator JointMarkerMultimapConstIterator;
typedef std::pair<JointMarkerMultimapConstIterator,
                  const JointMarkerMultimapConstIterator> joint_marker_multimap_iterator_pair;

bool try_insert_joint_position(
  rclcpp::Logger logger,
  const std::string& frame_name,
  const tf2::Vector3& position,
  std::map<std::string, tf2::Vector3>& known_joint_positions,
  std::map<std::string, tf2::Vector3>::iterator& position_it
)
{
  auto inserted = known_joint_positions.insert( std::make_pair( frame_name, position ) );
  if ( !inserted.second )
  {
    RCLCPP_ERROR(logger, "Failed to insert known joint position for joint %s", frame_name);
    return false;
  }
  position_it = inserted.first;
  return true;
}

void print_joint( rclcpp::Logger logger, const urdf::JointSharedPtr joint )
{
  RCLCPP_DEBUG(logger, "Joint");
  RCLCPP_DEBUG(logger, "Name: %s", joint->name);
  RCLCPP_DEBUG(logger,  "Axis x: %d, y: %d, z: %d", joint->axis.x, joint->axis.y, joint->axis.z);
  RCLCPP_DEBUG(logger, "Joint type: ");
  switch ( joint->type ) {
    case urdf::Joint::REVOLUTE:
      RCLCPP_DEBUG(logger, "Revolute");
      break;
    case urdf::Joint::FIXED:
      RCLCPP_DEBUG(logger, "Fixed");
      break;
    default:
      RCLCPP_DEBUG(logger, "Unknown");
  }
  RCLCPP_DEBUG(logger, "Parent link name: %s", joint->parent_link_name);
  RCLCPP_DEBUG(logger, "Child link name: %s", joint->child_link_name);
  double roll, pitch, yaw;
  urdf::Pose& pose = joint->parent_to_joint_origin_transform;
  pose.rotation.getRPY( roll, pitch, yaw );
  RCLCPP_DEBUG(
    logger, 
    "Parent to joint origin x: %d, y: %d, z: %d, roll: %d, pitch: %d, yaw: %d",
    pose.position.x, pose.position.y, pose.position.z, roll, pitch, yaw
  );
}

void print_link( rclcpp::Logger logger, const urdf::LinkSharedPtr link )
{
    RCLCPP_DEBUG(logger, "Link: %s", link->name);
    urdf::JointSharedPtr parent = link->parent_joint;
    if ( parent != nullptr )
    {
      print_joint( logger, parent );
    }
    RCLCPP_DEBUG(logger, "Child joints:");
    for ( auto& joint : link->child_joints )
      RCLCPP_DEBUG(logger, joint->name);
    RCLCPP_DEBUG(logger, "Child links:");
    for ( auto& child_link : link->child_links )
      RCLCPP_DEBUG(logger, child_link->name);
    RCLCPP_DEBUG(logger, "================");
}

void print_vector( rclcpp::Logger logger, const tf2::Vector3& vec )
{
  RCLCPP_DEBUG(logger, "x: %d, y: %d, z: %d", vec.m_floats[0], vec.m_floats[1], vec.m_floats[2]);
}

void print_rotation( rclcpp::Logger logger, const tf2::Quaternion& rot )
{
  tf2::Vector3 axis = rot.getAxis();
  RCLCPP_DEBUG(logger, "axis x, y, z: %d, %d, %d, angle: %d", axis.x(), axis.y(), axis.z(), rot.getAngle());
}

/** Convenience function for extracting a vector of markers from a multimap 
 * of pairs of joint/link frame and marker
 * @param source The source multimap to extract the markers from
 * @param target The target vector to which the markers will be pushed back
 */
void extract_marker_list( 
  const std::multimap<std::string, std::shared_ptr<marker_lib::Marker>>& source,
  std::vector<std::shared_ptr<marker_lib::Marker>>& target
)
{
  for ( auto& pair : source )
  {
    target.push_back( pair.second );
  }
}

/** Find the marker with the specified id
 * @param stored_markers Multimap of frames and their rigidly attached markers
 * @param frame Frame for which the marker should be retrieved
 * @param id id number of the marker that should be retrieved
 */
std::shared_ptr<marker_lib::Marker> find_stored_marker(
  const std::multimap<std::string, std::shared_ptr<marker_lib::Marker>>& stored_markers,
  const std::string& frame,
  int id
)
{
  auto markers = stored_markers.equal_range( frame );
  for ( auto marker_pair = markers.first; marker_pair != markers.second; marker_pair++ )
  {
    if ( (marker_pair->second)->id == id )
    {
      return marker_pair->second;
    }
  }
  return nullptr;
}

/** Forward kinematics to determine position of a joint
 * @param parent_joint_position The position of the joint's parent joint
 * @param parent_joint_rotation The rotation of the joint's parent joint
 * @param joint Joint of which the position should be determined
 * @param known_joint_positions Map of joint name and joint position vector 
 * (relative to a global frame)
 * @param position_it Iterator that specifies where to store the result
 */
bool get_joint_position(
  rclcpp::Logger logger,
  const tf2::Vector3& parent_joint_position,
  const tf2::Quaternion& parent_joint_rotation,
  const urdf::JointSharedPtr joint,
  std::map<std::string, tf2::Vector3>& known_joint_positions,
  std::map<std::string, tf2::Vector3>::iterator& position_it
)
{
  tf2::Vector3 parent_to_joint_origin_transform = tf2::Vector3(
    joint->parent_to_joint_origin_transform.position.x,
    joint->parent_to_joint_origin_transform.position.y,
    joint->parent_to_joint_origin_transform.position.z
  );
  tf2::Vector3 new_position = parent_joint_position + tf2::quatRotate(parent_joint_rotation, parent_to_joint_origin_transform);

  return try_insert_joint_position(
    logger, joint->child_link_name, new_position, known_joint_positions, position_it 
  );
}

/** Gets the position of the joint based on a vector of intersecting points
 * Assumes that the intersection points are the result of intersecting two spheres with a plane
 * @param intersection_points Possible points at which the joint could be located
 * @param parent_joint_position Position of the parent joint
 * @param joint Joint for which to determine the position
 * @param known_joint_positions Map of joint name and joint position vector 
 * (relative to a global frame)
 * @param joint_position_it Iterator that specifies where to store the result
 * @return Flag that specifies whether the joint position could be determined
 */
bool get_joint_position(
  rclcpp::Logger logger,
  const std::vector<tf2::Vector3>& found_intersection_points,
  const tf2::Vector3& parent_joint_position,
  const urdf::JointSharedPtr joint,
  std::map<std::string, tf2::Vector3>& known_joint_positions,
  std::map<std::string, tf2::Vector3>::iterator& joint_position_it
)
{
  if ( found_intersection_points.size() == 0 )
  {
    RCLCPP_ERROR(logger, "No intersection points found");
    return false;
  }
  else if ( found_intersection_points.size() == 1 )
  {
    tf2::Vector3 joint_new_position = parent_joint_position + 
                                       found_intersection_points[0];
    return try_insert_joint_position(
      logger, joint->child_link_name, joint_new_position, known_joint_positions, joint_position_it
    );
  }
  else
  {
    std::map<tf2::Vector3, int> number_of_intersections;
    for ( unsigned int i = 0; i < found_intersection_points.size(); ++i )
    {
      for ( unsigned int j = 1; j < found_intersection_points.size(); ++j )
      {
        if ( i == j )
          continue;
        if ( found_intersection_points[i] == found_intersection_points[j] )
        {
          auto counter = number_of_intersections.find( found_intersection_points[i] );
          if ( counter == number_of_intersections.end() )
          {
            number_of_intersections.insert( std::make_pair( found_intersection_points[0], 1 ) );
          }
          else
          {
            counter->second++;
          }
        }
      }
    }
    if ( number_of_intersections.size() == 0 )
    {
      return false;
    }
    std::shared_ptr<tf2::Vector3> point_with_most_intersections;
    int max_intersections = -1;
    for ( auto vector_counter_pair : number_of_intersections )
    {
      if ( vector_counter_pair.second > max_intersections )
      {
        max_intersections = vector_counter_pair.second;
        point_with_most_intersections = std::make_shared<tf2::Vector3>(
          vector_counter_pair.first
        );
      }
    }
    tf2::Vector3 joint_new_position = parent_joint_position + 
                                       *point_with_most_intersections;
    return try_insert_joint_position(
      logger, joint->child_link_name, joint_new_position, known_joint_positions, joint_position_it
    );
  }
}

/** Get the rotation between two vectors
 * @param a Source vector of the rotation
 * @param b Target vector of the rotation
 * @return Rotation determined
 */
tf2::Quaternion get_rotation( const tf2::Vector3& a, const tf2::Vector3& b )
{
  tf2::Vector3 a_normalized = a.normalized();
  tf2::Vector3 b_normalized = b.normalized();
  double angle = a_normalized.angle( b_normalized );
  tf2::Vector3 axis = a_normalized.cross( b_normalized );
  tf2::Quaternion rotation( axis, angle );
  return rotation;
}

tf2::Quaternion get_rotation( const tf2::Vector3& a, const tf2::Vector3& b, const tf2::Vector3& p_normal )
{
  tf2::Vector3 a_normalized = a.normalized();
  tf2::Vector3 b_normalized = b.normalized();
  tf2::Vector3 p_normalized = p_normal.normalized();
  auto a_perp = (a_normalized - a_normalized.dot( p_normalized ) * p_normalized).normalized();
  auto b_perp = (b_normalized - b_normalized.dot( p_normalized ) * p_normalized).normalized();
  //auto theta = std::acos( dot( a_perp, b_perp ) / (norm(a_perp) * norm(b_perp)) );
  return tf2::shortestArcQuat( a_perp, b_perp ).normalized();
}

void print_debug_marker(
  int markerId,
  float sx,
  float sy,
  float sz,
  float ex,
  float ey,
  float ez,
  float r,
  float g,
  float b,
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub_
)
{
    visualization_msgs::msg::Marker rviz_marker;
    rviz_marker.header.frame_id = "mocap";
    rviz_marker.header.stamp = rclcpp::Clock().now();
    rviz_marker.ns = "basic_shapes";
    rviz_marker.id = markerId;
    rviz_marker.pose.position.x = 0;
    rviz_marker.pose.position.y = 0;
    rviz_marker.pose.position.z = 0;
    rviz_marker.pose.orientation.x = 0;
    rviz_marker.pose.orientation.y = 0;
    rviz_marker.pose.orientation.z = 0;
    rviz_marker.pose.orientation.w = 1;
    rviz_marker.scale.x = 0.01;
    rviz_marker.scale.y = 0.02;
    rviz_marker.color.r = r;
    rviz_marker.color.g = g;
    rviz_marker.color.b = b;
    rviz_marker.color.a = 1.0;
    rviz_marker.type = visualization_msgs::msg::Marker::ARROW;
    geometry_msgs::msg::Point start;
    start.x = sx;
    start.y = sy;
    start.z = sz;
    geometry_msgs::msg::Point end;
    end.x = ex,
    end.y = ey;
    end.z = ez;
    rviz_marker.points.push_back(start);
    rviz_marker.points.push_back(end);
    rviz_marker_pub_->publish(rviz_marker);

}

/** Gets the rotation of a joint based on the difference between the marker position
 * and the expected marker position if the joint was not rotated
 * @param mocap_markers Multimap of frame and marker as detected by the mocap system
 *        (translated to source frame)
 * @param stored_markers Multimap of frame and marker as stored in the markerfile
 * (relative to joint origin)
 * @param parent_joint_rotation Rotation of the parent joint
 * @param position Position of the joint origin relative to the global frame
 * @param joint Joint for which to determine rotation
 * @param res Result map to insert the calculated rotation into
 */
bool get_joint_rotation(
  const std::multimap<std::string, std::shared_ptr<marker_lib::Marker>>& mocap_markers,
  const std::multimap<std::string, std::shared_ptr<marker_lib::Marker>>& stored_markers,
  const tf2::Quaternion& parent_joint_rotation,
  const tf2::Vector3& position,
  const urdf::JointSharedPtr joint,
  std::map<std::string, tf2::Quaternion>& res,
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub_
)
{
  auto markers = mocap_markers.equal_range( joint->child_link_name );
  if ( markers.first == markers.second )
    return false;
 
  tf2::Quaternion parent_to_joint_origin_transform(
    joint->parent_to_joint_origin_transform.rotation.x,
    joint->parent_to_joint_origin_transform.rotation.y,
    joint->parent_to_joint_origin_transform.rotation.z,
    joint->parent_to_joint_origin_transform.rotation.w
  );
  tf2::Quaternion global_rotation = (parent_to_joint_origin_transform * parent_joint_rotation).normalized();
    
  tf2::Vector3 tf2_axis(
    joint->axis.x,
    joint->axis.y,
    joint->axis.z
  );

  std::vector<tf2::Quaternion> rotations;
  for ( auto marker_pair = markers.first; marker_pair != markers.second; marker_pair++ )
  {
    std::shared_ptr<marker_lib::Marker> mocap_marker = marker_pair->second;
    std::shared_ptr<marker_lib::Marker> stored_marker = find_stored_marker( stored_markers,
									    joint->child_link_name,
                                                                            mocap_marker->id );
    if ( stored_marker == nullptr )
      continue;

    tf2::Vector3 stored_marker_vec( stored_marker->x, stored_marker->y, stored_marker->z );
    tf2::Vector3 stored_marker_rotated = tf2::quatRotate( global_rotation, stored_marker_vec );
    tf2::Vector3 vector_to_marker(
      mocap_marker->x - position.x(),
      mocap_marker->y - position.y(),
      mocap_marker->z - position.z()
    );
    tf2::Vector3 plane_normal = tf2::quatRotate( global_rotation, tf2_axis );

    /*print_debug_marker(
      mocap_marker->id,
      position.x(),
      position.y(),
      position.z(),
      position.x() + stored_marker_rotated.x(),
      position.y() + stored_marker_rotated.y(),
      position.z() + stored_marker_rotated.z(),
      0,
      1.0,
      0,
      rviz_marker_pub_
    );
    print_debug_marker(
      mocap_marker->id*10,
      position.x(),
      position.y(),
      position.z(),
      position.x() + vector_to_marker.x(),
      position.y() + vector_to_marker.y(),
      position.z() + vector_to_marker.z(),
      0,
      0,
      1.0,
      rviz_marker_pub_
    );
    print_debug_marker(
      mocap_marker->id*100,
      position.x(),
      position.y(),
      position.z(),
      position.x() + plane_normal.x()/3,
      position.y() + plane_normal.y()/3,
      position.z() + plane_normal.z()/3,
      1.0,
      0,
      0,
      rviz_marker_pub_
    );*/

    tf2::Quaternion rotation = get_rotation( stored_marker_rotated, vector_to_marker, plane_normal );
    rotations.push_back( rotation );
  }

  if (rotations.size() == 0)
  {
    return false;
  }

  tf2::Quaternion return_quat = rotations[0];
  if (rotations.size() > 1)
  {
    for ( std::vector<std::shared_ptr<tf2::Quaternion>>::size_type i = 1; i < rotations.size(); ++i )
    {
      return_quat = return_quat.slerp( rotations[i], 0.5 );
    }
  }

  res.insert( std::make_pair( joint->child_link_name, (return_quat * global_rotation).normalize() ) );
  return true;
}

/** Function to find intersection points of marker spheres and joint's parent joint plane
 * @param mocap_markers Multimap of frame and marker as detected by the mocap system
 *        (translated to source frame)
 * @param stored_markers Multimap of frame and marker as stored in the markerfile
 * (relative to joint origin)
 * @param plane Plane in which to detect the intersection points
 * @param circle Circle embedded in 3D that shows the reach of the parent link
 * @param local_origin Origin position of the parent joint (relative to source frame)
 * @param mocap_markers_for_joint Pair of begin and end iterators for the markers for which to 
 *        calculate intersection
 * @param joint Joint which has the marker(s) attached
 * @param found_intersection_points Vector to append found intersection points to
 */
bool find_intersection_points(
  rclcpp::Logger logger,
  const std::multimap<std::string, std::shared_ptr<marker_lib::Marker>>& stored_markers,
  const geometry_lib::Plane& plane,
  const geometry_lib::Circle3D& circle,
  const tf2::Vector3& local_origin,
  const joint_marker_multimap_iterator_pair& mocap_markers_for_joint,
  urdf::JointSharedPtr joint,
  std::vector<geometry_lib::Point3D>& found_intersection_points
)
{
  for ( auto marker_pair = mocap_markers_for_joint.first; 
        marker_pair != mocap_markers_for_joint.second;
        marker_pair++ )
  {
    const std::shared_ptr<marker_lib::Marker> marker = marker_pair->second;
    auto stored_marker = find_stored_marker( stored_markers, joint->child_link_name, marker->id );
    geometry_lib::Sphere s(
      marker->x - local_origin.x(),
      marker->y - local_origin.y(),
      marker->z - local_origin.z(),
      stored_marker->norm()
    );
    auto sphere_plane_intersection = intersect( s, plane );
    if ( sphere_plane_intersection.has_value() )
    {
      auto intersection_circle = sphere_plane_intersection.value();
      auto circle_circle_intersection = intersect( circle, intersection_circle );
      try
      {
        auto intersection_points = std::any_cast<
          std::pair<geometry_lib::Point3D, geometry_lib::Point3D>
        >( circle_circle_intersection );

        found_intersection_points.push_back( intersection_points.first );
        if ( intersection_points.first != intersection_points.second )
        {
          found_intersection_points.push_back( intersection_points.second );
        }
      }
      catch ( const std::bad_any_cast& ex )
      {
        RCLCPP_ERROR(logger, "No intersection detected between circle and circle");
        return false;
      }
    }
    else
    {
      return false;
    }
  }
  return true;
}

/** Radians to degrees */
double deg( double rad )
{
  return rad * 180 / 3.1415926;
}

/** Load the urdf file from the specified path */
urdf::Model load_urdf( const std::string& urdf_path )
{
  urdf::Model robot_model;
  robot_model.initFile( urdf_path );
  return robot_model;
}

/** Update a "skipped link" followed by a link with two markers
 * @param mocap_markers Multimap of frame and marker as detected by the mocap system
 *        (translated to source frame)
 * @param stored_markers Multimap of frame and marker as stored in the markerfile
 * (relative to joint origin)
 * @param j0 Root joint for computation
 * @param j1 Joint's parent joint
 * @param j2 Joint considered
 * @param known_joint_positions Map of joint name and joint position vector 
 * (relative to a global frame)
 * @param known_joint_rotations Map of joint name and joint rotation 
 * (relative to the rotation in the global frame)
 * @return Whether any changes were made
 */
bool update_skipped_link(
  rclcpp::Logger logger,
  const std::multimap<std::string, std::shared_ptr<marker_lib::Marker>>& mocap_markers,
  const std::multimap<std::string, std::shared_ptr<marker_lib::Marker>>& stored_markers,
  const urdf::JointSharedPtr j0,
  urdf::JointSharedPtr j1,
  urdf::JointSharedPtr j2,
  std::map<std::string, tf2::Vector3>& known_joint_positions,
  std::map<std::string, tf2::Quaternion>& known_joint_rotations,
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub_
)
{
  auto j0_position_it = known_joint_positions.find( j0->child_link_name );
  auto j0_rotation_it = known_joint_rotations.find( j0->child_link_name );
  if ( j0_position_it == known_joint_positions.end() || 
       j0_rotation_it == known_joint_rotations.end() )
  {
    // we need to know the position and rotation of j-2 in order for this procedure to work
    return false;
  }

  std::map<std::string, tf2::Vector3>::iterator j1_position_it;
  bool position_determined = get_joint_position(
    logger, j0_position_it->second, j0_rotation_it->second,
    j1, known_joint_positions, j1_position_it
  );
  if ( !position_determined )
  { 
    // if for some reason we couldn't determine the position of the parent joint,
    // then this procedure won't work
    return false;
  }

  geometry_lib::Plane plane( j2->axis.x, j2->axis.y, j2->axis.z, 0 );
  tf2::Vector3 j2_parent_to_joint_origin_transform( 
    j2->parent_to_joint_origin_transform.position.x,
    j2->parent_to_joint_origin_transform.position.y,
    j2->parent_to_joint_origin_transform.position.z
  );
  geometry_lib::Circle3D circle( 
    0, 0, 0, j2_parent_to_joint_origin_transform.length(), plane 
  );
  auto mocap_markers_for_joint = mocap_markers.equal_range( j2->child_link_name );
  //auto mocap_markers_for_joint = mocap_markers.equal_range( j2->name );
  if ( mocap_markers_for_joint.first == mocap_markers_for_joint.second )
  { // the joint has no markers, hence we can't get the joint's position and rotation
    return true;
  }
  std::vector<geometry_lib::Point3D> found_intersection_points;
  bool intersection_points_found = find_intersection_points(
    logger, stored_markers, plane, circle, j1_position_it->second,
    mocap_markers_for_joint, j2, found_intersection_points
  );
  if ( !intersection_points_found )
  {
    return true;
  }
  
  std::map<std::string, tf2::Vector3>::iterator j2_position_it;
  std::vector<tf2::Vector3> tf2_found_intersection_points;
  for (auto point : found_intersection_points)
  {
    tf2_found_intersection_points.push_back(
      tf2::Vector3( point.x, point.y, point.z )
    );
  }
  bool joint_position_found = get_joint_position(
    logger, tf2_found_intersection_points, j1_position_it->second,
    j2, known_joint_positions, j2_position_it
  );
  if ( !joint_position_found )
  {
    return true;
  }
  // get the rotation for j1
  tf2::Quaternion j1_rotation = get_rotation(
    tf2::quatRotate( j0_rotation_it->second, j2_parent_to_joint_origin_transform ),
    j2_position_it->second - j1_position_it->second
  );
  tf2::Quaternion j1_global_rotation = j0_rotation_it->second * j1_rotation;
  auto inserted = known_joint_rotations.insert(
    std::make_pair( j1->child_link_name, j1_global_rotation )
  );
  if ( !inserted.second )
  {
    RCLCPP_ERROR(logger, "Failed to insert known joint rotation for joint %s", j1->name);
    return true;
  }

  // get the rotation for j2
  bool found_j2_rotation = get_joint_rotation(
    mocap_markers, stored_markers, j1_rotation, j2_position_it->second, j2, known_joint_rotations, rviz_marker_pub_
  );
  if ( !found_j2_rotation )
  {
    RCLCPP_ERROR(logger, "Could not find rotation of j2");
  }

  // even if we couldn't find the rotation of this joint, 
  // we should have still found the parent joint rotation
  return true;
}

marker_lib::Marker mocap_marker_to_marker_lib_marker( const mocap4ros_msgs::msg::MarkerWithId& in )
{
  return marker_lib::Marker( in.index, in.translation.x, in.translation.y, in.translation.z );
}

tf2::Vector3 average_vector( const std::vector<tf2::Vector3>& vectors )
{
  tf2::Vector3 result;
  for (auto v : vectors)
  {
    result += v / vectors.size();
  }
  return result;
}

/** Constructor for a robot frame estimator
 * @param robot_model URDF description of the robot which frames should be tracked
 * @param frames_and_markers Multimap specifying which markers are rigidly attached to which frames
 */
ConfigurationFromMocap::Estimator::Estimator(rclcpp::Clock::SharedPtr clock) : 
  rclcpp::Node("configuration_estimator"),
  clock_(clock),
  buffer_(std::make_shared<tf2_ros::Buffer>(clock_, tf2::Duration(100ms))),
  listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_))
{
  std::string urdf_path;
  declare_parameter<std::string>("urdf_path", std::string("urdf.xml"));
  get_parameter<std::string>("urdf_path", urdf_path);

  robot_model_ = load_urdf(urdf_path);

  for (auto& joint : robot_model_.joints_)
  {
    print_joint(get_logger(), joint.second);
  }

  std::string marker_path;
  declare_parameter<std::string>("marker_path", std::string("markers.xml"));
  get_parameter<std::string>("marker_path", marker_path);
  if (!parse_markers(marker_path, stored_markers_))
  {
    RCLCPP_ERROR(get_logger(), "Unable to parse marker definition");
    return;
  }

  declare_parameter<std::string>("source_frame", std::string("base_link"));
  get_parameter<std::string>("source_frame", source_frame_);
  declare_parameter<std::string>("global_frame", std::string("mocap"));
  get_parameter<std::string>("global_frame", global_frame_);

  // keep a copy of the marker list
  extract_marker_list( stored_markers_, marker_list_ );
  br_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  markers_sub_ = create_subscription<mocap4ros_msgs::msg::MarkersWithId>(
    "markers_with_id", 10, std::bind(&Estimator::markers_callback, this, _1)
  );

  rviz_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("debug_markers", 1);
}

/** Callback function for when markers are received */
void ConfigurationFromMocap::Estimator::markers_callback(
  const mocap4ros_msgs::msg::MarkersWithId::SharedPtr markers
)
{
  std::multimap<std::string, std::shared_ptr<marker_lib::Marker>> mocap_markers;
  std::map<std::string, int> num_markers_per_joint;
  for ( mocap4ros_msgs::msg::MarkerWithId marker : markers->markers )
  {
    std::shared_ptr<marker_lib::Marker> found_marker = find_marker( marker.index, marker_list_ );
    if ( found_marker != nullptr )
    {
      auto num_markers_it = num_markers_per_joint.find( found_marker->frame );
      if ( num_markers_it == num_markers_per_joint.end() )
      {
        num_markers_per_joint.insert( std::make_pair( found_marker->frame, 1 ) );
      }
      else
      {
        num_markers_it->second++;
      }
      mocap_markers.insert(
        std::make_pair(
          found_marker->frame,
          std::make_shared<marker_lib::Marker>( mocap_marker_to_marker_lib_marker( marker ) ) 
        )
      );
    }
  }

  bool changed = true;
  std::map<std::string, tf2::Vector3> known_joint_positions;
  std::map<std::string, tf2::Quaternion> known_joint_rotations;

  std::map<std::string, urdf::JointSharedPtr> joint_for_link;
  geometry_msgs::msg::TransformStamped global_frame_to_source_frame_;
  bool transformFound = false;
  while ( !transformFound )
  {
    try
    {
      global_frame_to_source_frame_ = buffer_->lookupTransform(
        global_frame_, source_frame_, rclcpp::Clock().now(), tf2::durationFromSec(0.005)
      );
      transformFound = true;
    }
    catch ( tf2::TransformException& ex )
    {
      RCLCPP_WARN( get_logger(), "%s", ex.what() );
      rclcpp::Rate sleep_rate(1);
      sleep_rate.sleep();
    }
  }

  // joints_ is a map of the joint name and a JointSharedPtr
  for ( const auto& joint : robot_model_.joints_ )
  {
    joint_for_link.insert( std::make_pair( ((joint.second)->child_link_name ), joint.second ) );
    if ((joint.second)->parent_link_name == source_frame_)
    {
      tf2::Vector3 transform_vec(
        global_frame_to_source_frame_.transform.translation.x,
        global_frame_to_source_frame_.transform.translation.y,
        global_frame_to_source_frame_.transform.translation.z
      );
      tf2::Quaternion rotation_quat = tf2::Quaternion(
          global_frame_to_source_frame_.transform.rotation.x,
          global_frame_to_source_frame_.transform.rotation.y,
          global_frame_to_source_frame_.transform.rotation.z,
          global_frame_to_source_frame_.transform.rotation.w
      );
      known_joint_positions.insert( std::make_pair( source_frame_, transform_vec ) );
      known_joint_rotations.insert( std::make_pair( source_frame_, rotation_quat ) );
    }
  }

  while ( changed )
  {
    changed = false;

    for ( const auto& joint_pair : robot_model_.joints_ )
    {
      const auto& joint = joint_pair.second;
      auto position_it = known_joint_positions.find( joint->child_link_name );
      auto rotation_it = known_joint_rotations.find( joint->child_link_name );
      bool position_known = position_it != known_joint_positions.end();
      bool rotation_known = rotation_it != known_joint_rotations.end(); 

      // if position and rotation of this joint are known, then we have nothing left to do
      if ( position_known && rotation_known )
        continue;

      urdf::JointSharedPtr parent_parent_joint;
      urdf::JointSharedPtr parent_joint;
      int num_markers = 0;
      auto num_markers_it = num_markers_per_joint.find( joint->child_link_name );
      if ( num_markers_it != num_markers_per_joint.end() )
      {
        num_markers = num_markers_it->second;
      }
      auto parent_joint_position_it = known_joint_positions.find( joint->parent_link_name );
      auto parent_joint_rotation_it = known_joint_rotations.find( joint->parent_link_name );
      bool parent_position_known = parent_joint_position_it != known_joint_positions.end();
      bool parent_rotation_known = parent_joint_rotation_it != known_joint_rotations.end();

      switch ( joint->type ) {
        case urdf::Joint::REVOLUTE:
          {
            bool parents_parent_position_known = false;
            bool parents_parent_rotation_known = false;
            auto parent_joint_it = joint_for_link.find( joint->parent_link_name );
            if ( parent_joint_it != joint_for_link.end() )
            {
              parent_joint = parent_joint_it->second;
              auto parent_parent_joint_it = joint_for_link.find( parent_joint->parent_link_name );
              if ( parent_parent_joint_it != joint_for_link.end() )
              {
                parent_parent_joint = parent_parent_joint_it->second;
                auto parent_parent_position_it = known_joint_positions.find(
                  parent_parent_joint->child_link_name
                );
                auto parent_parent_rotation_it = known_joint_rotations.find(
                  parent_parent_joint->child_link_name
                );
                parents_parent_position_known = 
                  parent_parent_position_it != known_joint_positions.end();
                parents_parent_rotation_known =
                  parent_parent_rotation_it != known_joint_rotations.end();
              }
            }

            // check if we can update a joint pose using this marker
            // this is the case if one of the following holds:
            // 1. we know the child joint pose and the parent joint pose
            // 2. we know the child joint pose or the parent joint pose and at least 1 marker attached
            // 3. we know the parent joint's parent joint pose and at least 2 markers attached
            if ( !position_known )
            {
              if ( !parent_position_known && !parent_rotation_known )
              {
                // if we don't know the parents joint position and orientation then we cannot calculate 
                // the current joint position using a single marker
                // otherwise we can use try the "skip joint" algorithm
                if ( parents_parent_position_known && 
                     parents_parent_rotation_known && 
                     num_markers == 2 )
                {
                  changed = update_skipped_link( 
                    get_logger(), mocap_markers, stored_markers_, parent_parent_joint, parent_joint, 
                    joint, known_joint_positions, known_joint_rotations, rviz_marker_pub_
                  );
                }
                continue;
              }

              position_known = get_joint_position(
                get_logger(), parent_joint_position_it->second, parent_joint_rotation_it->second,
                joint, known_joint_positions, position_it
              );

              if ( !position_known )
              {
                continue;
              }

              changed = true;
            }

            if ( !rotation_known )
            {
              rotation_known = get_joint_rotation(
                mocap_markers, stored_markers_, parent_joint_rotation_it->second, 
                position_it->second, joint, known_joint_rotations, rviz_marker_pub_
              );

              if ( !rotation_known )
              {
                continue;
              }

              changed = true;
            }
          }
          break;
        case urdf::Joint::FIXED:
          if ( parent_position_known && parent_rotation_known )
          {
            tf2::Vector3 joint_position(
              joint->parent_to_joint_origin_transform.position.x,
              joint->parent_to_joint_origin_transform.position.y,
              joint->parent_to_joint_origin_transform.position.z
            );
            tf2::Quaternion joint_rotation(
              joint->parent_to_joint_origin_transform.rotation.x,
              joint->parent_to_joint_origin_transform.rotation.y,
              joint->parent_to_joint_origin_transform.rotation.z,
              joint->parent_to_joint_origin_transform.rotation.w
            );
            auto& parent_position = parent_joint_position_it->second;
            auto& parent_rotation = parent_joint_rotation_it->second;
            auto new_position = parent_position + tf2::quatRotate( parent_rotation, joint_position );
            auto new_rotation = parent_rotation * joint_rotation;
            new_rotation.normalize();
            known_joint_positions.insert( std::make_pair( joint->child_link_name, new_position ) );
            known_joint_rotations.insert( std::make_pair( joint->child_link_name, new_rotation ) );
            changed = true;
            continue;
          }
          break;
        case urdf::Joint::PRISMATIC:
          if (parent_position_known && parent_rotation_known )
          {
            tf2::Vector3 joint_position(
              joint->parent_to_joint_origin_transform.position.x,
              joint->parent_to_joint_origin_transform.position.y,
              joint->parent_to_joint_origin_transform.position.z
            );
            tf2::Quaternion joint_rotation(
              joint->parent_to_joint_origin_transform.rotation.x,
              joint->parent_to_joint_origin_transform.rotation.y,
              joint->parent_to_joint_origin_transform.rotation.z,
              joint->parent_to_joint_origin_transform.rotation.w
            );
            auto& parent_position = parent_joint_position_it->second;
            auto& parent_rotation = parent_joint_rotation_it->second;
            auto mocap_markers_for_joint = mocap_markers.equal_range( joint->child_link_name );

            std::vector<tf2::Vector3> results;
            for ( auto marker_pair = mocap_markers_for_joint.first; 
                  marker_pair != mocap_markers_for_joint.second;
                  marker_pair++ )
            {
              const std::shared_ptr<marker_lib::Marker> mocap_marker = marker_pair->second;
              auto stored_marker = find_stored_marker(
                stored_markers_, joint->child_link_name, mocap_marker->id
              );
              if ( stored_marker == nullptr )
                continue;
              tf2::Vector3 base_position = parent_position + tf2::quatRotate( parent_rotation, joint_position );
              tf2::Vector3 stored_marker_vec(
                stored_marker->x, stored_marker->y, stored_marker->z
              );
              tf2::Vector3 rotated_marker = tf2::quatRotate( parent_rotation, stored_marker_vec );
              tf2::Vector3 vector_to_marker(
                mocap_marker->x - base_position.x(),
                mocap_marker->y - base_position.y(),
                mocap_marker->z - base_position.z()
              );
              tf2::Vector3 displacement_vector(
                vector_to_marker.x() - rotated_marker.x(),
                vector_to_marker.y() - rotated_marker.y(),
                vector_to_marker.z() - rotated_marker.z()
              );
              tf2::Quaternion actual_rotation = parent_rotation * joint_rotation;
              tf2::Vector3 actual_position = tf2::quatRotate( actual_rotation, joint_position );
              tf2::Vector3 new_position = parent_position + actual_position + displacement_vector;
              results.push_back( new_position );
            }
            if (results.size() == 0)
              continue;
            auto average = average_vector( results );
            known_joint_positions.insert( std::make_pair( joint->child_link_name, average ) );
            known_joint_rotations.insert( std::make_pair( joint->child_link_name, parent_rotation ) );
            changed = true;
            continue;
          }
          break;
        default: // switch joint->type
          RCLCPP_DEBUG(get_logger(), "Unsupported joint type");
      }
    }
  }

  RCLCPP_DEBUG(get_logger(), "Sending transforms");

  for (auto& pos_pair : known_joint_positions)
  {
    auto rot_pair = known_joint_rotations.find( pos_pair.first );
    if (rot_pair != known_joint_rotations.end())
    {
      const tf2::Vector3& position = pos_pair.second;
      const tf2::Quaternion& rotation = rot_pair->second;
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = rclcpp::Clock().now();
      transform.header.frame_id = global_frame_;
      transform.child_frame_id = pos_pair.first + "_estimated";
      transform.transform.translation.x = position.x();
      transform.transform.translation.y = position.y();
      transform.transform.translation.z = position.z();
      transform.transform.rotation.x = rotation.x();
      transform.transform.rotation.y = rotation.y();
      transform.transform.rotation.z = rotation.z();
      transform.transform.rotation.w = rotation.w();
      br_->sendTransform(transform);
    }
  }
}
