#include <algorithm>
#include <string>
#include <memory>

#include "nav2_util/node_utils.hpp"
#include "nav2_line_following_controller/line_following_controller.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_line_following_controller/geometry.h"

#include "nav2_costmap_2d/footprint_collision_checker.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace nav2_line_following_controller
{


Angle yaw_from_pose(const geometry_msgs::msg::PoseStamped & pose ) {
  tf2::Quaternion q;
  tf2::fromMsg(pose.pose.orientation, q);
  tf2::Matrix3x3 m(q);
  tf2Scalar r,p,y;
  m.getRPY(r,p,y);
  return Angle::radians(y);
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(Angle yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw.radians());
    geometry_msgs::msg::Quaternion rv;
    rv.w = q.getW();
    rv.x = q.getX();
    rv.y = q.getY();
    rv.z = q.getZ();
    return rv;
}


rcl_interfaces::msg::SetParametersResult LineFollowingController::on_parameters_set_callback(
  const std::vector<rclcpp::Parameter> & parameters) {
  
  RCLCPP_INFO(
    logger_,
    "Got a parameters reconfigure callback: %s",
    plugin_name_.c_str());
  
  for(const auto & parameter : parameters) {
    if(parameter.get_name() == plugin_name_+".max_cte") {
      max_cte_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".max_velocity") {
      max_velocity_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".max_reverse_velocity") {
      max_reverse_velocity_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".max_acceleration") {
      max_acceleration_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".max_deceleration") {
      max_deceleration_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".max_lateral_acceleration") {
      max_lateral_acceleration_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".min_turn_radius") {
      min_turn_radius_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".steering_k_p") {
      steering_k_p_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".steering_k_d") {
      steering_k_d_ = parameter.as_double();
    }
    if(parameter.get_name() == plugin_name_+".transform_tolerance") {
      transform_tolerance_ = rclcpp::Duration::from_seconds(parameter.as_double());
    }
    if(parameter.get_name() == plugin_name_+".lookahead_distance") {
      lookahead_distance_ = fabs(parameter.as_double());
    }
    if(parameter.get_name() == plugin_name_+".route_smoothing_strength") {
      route_smoothing_strength_ = parameter.as_double();
    }

    RCLCPP_INFO(
      logger_,
      "%s: Parameter name: %s value: %s",
      plugin_name_.c_str(),
      parameter.get_name().c_str(),
      parameter.value_to_string().c_str());

  }

  rcl_interfaces::msg::SetParametersResult result; 
  result.successful = true;
  result.reason = "";
  return result;
}



void LineFollowingController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, 
    std::string name, 
    std::shared_ptr<tf2_ros::Buffer> tf, 
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
  )
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

declare_parameter_if_not_declared(
  node,
  plugin_name_ + ".lookahead_distance",
  rclcpp::ParameterValue(0.4));

declare_parameter_if_not_declared(
  node,
  plugin_name_ + ".max_acceleration",
  rclcpp::ParameterValue(0.2));

declare_parameter_if_not_declared(
  node,
  plugin_name_ + ".max_cte", 
  rclcpp::ParameterValue(0.2));

declare_parameter_if_not_declared(
  node, 
  plugin_name_ + ".max_deceleration",
  rclcpp::ParameterValue(0.2));

declare_parameter_if_not_declared(
  node, 
  plugin_name_ + ".max_lateral_acceleration",
  rclcpp::ParameterValue(0.1));

declare_parameter_if_not_declared(
  node,
  plugin_name_ + ".max_reverse_velocity",
  rclcpp::ParameterValue(0.5));

declare_parameter_if_not_declared(
  node,
  plugin_name_ + ".max_velocity", 
  rclcpp::ParameterValue(1.0));

declare_parameter_if_not_declared(
  node, 
  plugin_name_ + ".min_turn_radius", 
  rclcpp::ParameterValue(0.5));

{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.name = plugin_name_ + ".route_smoothing_strength";
  descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  descriptor.description = "Amounte of smoothing to applyt to route, must be between 0 and 1. 0 means no smoothing, 1 means maximum smoothing.";
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = 0.0;
  range.to_value = 1.0;
  descriptor.floating_point_range.push_back(range);

  node->declare_parameter(descriptor.name, 0.5, descriptor);
}

declare_parameter_if_not_declared(
  node,
  plugin_name_ + ".steering_k_d",
  rclcpp::ParameterValue(3.0));

declare_parameter_if_not_declared(
  node,
  plugin_name_ + ".steering_k_p",
  rclcpp::ParameterValue(3.0));

declare_parameter_if_not_declared(
  node, 
  plugin_name_ + ".transform_tolerance", 
  rclcpp::ParameterValue(0.1));


  // handle keeps the callback alive
  parameters_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&LineFollowingController::on_parameters_set_callback, this, std::placeholders::_1));
  

  node->get_parameter(plugin_name_ + ".max_cte", max_cte_);
  node->get_parameter(plugin_name_ + ".max_velocity", max_velocity_);
  node->get_parameter(plugin_name_ + ".max_reverse_velocity", max_reverse_velocity_);
  node->get_parameter(plugin_name_ + ".max_deceleration", max_deceleration_);
  node->get_parameter(plugin_name_ + ".max_acceleration", max_acceleration_);
  node->get_parameter(plugin_name_ + ".max_lateral_acceleration", max_lateral_acceleration_);
  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".min_turn_radius", min_turn_radius_);
  node->get_parameter(plugin_name_ + ".steering_k_p", steering_k_p_);
  node->get_parameter(plugin_name_ + ".steering_k_d", steering_k_d_);
  node->get_parameter(plugin_name_ + ".route_smoothing_strength", route_smoothing_strength_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", 1);
  lookahead_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("lookahead", 1);
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 1);
}

void LineFollowingController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type nav2_line_following_controller::LineFollowingController",
    plugin_name_.c_str());
  local_plan_pub_.reset();
  lookahead_pub_.reset();
  marker_pub_.reset();
}

void LineFollowingController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type nav2_line_following_controller::LineFollowingController",
    plugin_name_.c_str());
  local_plan_pub_->on_activate();
  lookahead_pub_->on_activate();
  marker_pub_->on_activate();
}

void LineFollowingController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type nav2_line_following_controller::LineFollowingController",
    plugin_name_.c_str());
  local_plan_pub_->on_deactivate();
  lookahead_pub_->on_deactivate();
  marker_pub_->on_deactivate();
}



geometry_msgs::msg::TwistStamped LineFollowingController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*twist_pv*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  bool stop = false;
  Angle yaw = yaw_from_pose(pose);

  route_position_->set_position({pose.pose.position.x, pose.pose.position.y});

  // move to next segment if done
  if(route_position_->done) {
    // another subroute available?
    if(current_subroute_index_ < routes_.size() - 1) {
      // yes, advance
      ++current_subroute_index_;
      route_ = routes_[current_subroute_index_];
      publish_local_plan();
      route_position_ = std::make_shared<Route::Position>(*route_);
      route_position_->set_position({pose.pose.position.x, pose.pose.position.y});
    } else {
      const int throttle_ms = 2000;
      auto & clock = *this->clock_;
      RCLCPP_INFO_THROTTLE(logger_, clock, throttle_ms, "%s - %s", plugin_name_.c_str(), "done with route");
    }
  }

  // stop for any obstacles
  {
    auto costmap = costmap_ros_->getCostmap();
    // get robot collision status
    // initialize collision checker and set costmap
    auto footprint_collision_checker_ = std::make_unique<nav2_costmap_2d::
        FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap);
    footprint_collision_checker_->setCostmap(costmap);
    auto footprint = costmap_ros_->getRobotFootprint();

    std::vector<visualization_msgs::msg::Marker> markers;

    int marker_id = 0;
    auto min_lookahead = 0.5;
    auto max_lookahead = 2.0;
    auto lookahead_increment = 0.1;
    for(auto lookahead_meters = min_lookahead; lookahead_meters <= max_lookahead; lookahead_meters += lookahead_increment) {
      auto ahead = route_->get_position_ahead(*route_position_, lookahead_meters);
      auto ahead_pose = route_->get_pose_at_position(ahead.position);


      if(lookahead_meters + lookahead_increment > max_lookahead)
      {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = clock_->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = ahead_pose.position.x;
        msg.pose.position.y = ahead_pose.position.y;
        msg.pose.position.z = 0.0;
        msg.pose.orientation = yaw_to_quaternion(ahead_pose.heading);
        lookahead_pub_->publish(msg);
      }

      double cost = footprint_collision_checker_->footprintCostAtPose(ahead_pose.position.x, ahead_pose.position.y, ahead_pose.heading.radians(), footprint);
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = clock_->now();
        marker.ns = "collision";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
    
        marker.pose.position.x = ahead_pose.position.x;
        marker.pose.position.y = ahead_pose.position.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation = yaw_to_quaternion(ahead_pose.heading);
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        if(cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          marker.color.r = 1.0;
          marker.color.g = 0.0;
          marker.color.b = 0.0;
        } else {
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 1.0;
        }
        markers.push_back(marker);
      }


      if(cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
        RCLCPP_WARN(logger_, "%s - Collision ahead, stopping x: %.2f y: %.2f yaw:%1.2f, ahead x: %.2f y: %2f yaw: %.2f footprint cost: %.2f", 
          plugin_name_.c_str(),
          pose.pose.position.x,
          pose.pose.position.y,
          yaw.degrees(),
          ahead_pose.position.x,
          ahead_pose.position.y,
          ahead_pose.heading.degrees(),
          cost);

  
        // RCLCPP_WARN(logger_, "%s - %s", plugin_name_.c_str(), "collision ahead, stopping");

        stop = true;
      }
    }
    visualization_msgs::msg::MarkerArray msg;
    msg.markers = markers;
    marker_pub_->publish(msg);
  }



  if(fabs(route_position_->cte) > max_cte_) {
    RCLCPP_WARN(logger_, "%s - aborting control, cross track error is too high cte: %0.2f", plugin_name_.c_str(), route_position_->cte);
    Pose2d desired = route_->get_pose_at_position(*route_position_);
    RCLCPP_WARN(logger_, "%s - desired x: %0.2f y: %0.2f yaw: %0.2f", plugin_name_.c_str(), desired.position.x, desired.position.y, desired.heading.degrees());
    RCLCPP_WARN(logger_, "%s - current x: %0.2f y: %0.2f yaw: %0.2f", plugin_name_.c_str(), pose.pose.position.x, pose.pose.position.y, yaw.degrees());

    stop = true;
  }



  RCLCPP_DEBUG(
    logger_,
    "%s - Route position: index: %lu progress: %f cte: %f done: %d",
      plugin_name_.c_str(),
      route_position_->index,
      route_position_->progress,
      route_position_->cte,
      route_position_->done
  );



  double velocity = route_->get_velocity(*route_position_);



  auto route_yaw = route_->get_yaw(*route_position_);
  auto yaw_diff = route_yaw - yaw;
  yaw_diff.standardize();
  bool reverse = (fabs(yaw_diff.radians()) > M_PI/2);

  // std::cout<<"route_yaw: " << route_yaw.degrees() << " yaw: " << yaw.degrees() << " diff: " << yaw_diff.degrees() << " reverse: " << reverse << std::endl;

  // if reverse, yaw is opposite
  if(reverse) {
    yaw = yaw+Angle::radians(M_PI);
    yaw.standardize();
    velocity = std::min(velocity, max_reverse_velocity_);
  }
  auto yaw_error = route_yaw - yaw;
  yaw_error.standardize();


  auto route_curvature = route_->curvature_ahead(*route_position_, lookahead_distance_).radians();

  // yaw error is considered d_error
  auto d_error = std::sin (yaw_error.radians());
  auto p_error = route_position_->cte;

  auto d_contribution = d_error * steering_k_d_;
  auto p_contribution = p_error * steering_k_p_;
  auto curvature = route_curvature +  d_contribution + p_contribution;


  // auto curvature = 2.0 * goal_pose.position.y /
  //   (goal_pose.position.x * goal_pose.position.x + goal_pose.position.y * goal_pose.position.y);
  
  // enforce turn radius
  if(min_turn_radius_ > 0) {
    auto max_curvature = 1.0 / min_turn_radius_;
    curvature = std::clamp(curvature, -max_curvature, max_curvature);
  }
  
  double angular_velocity = velocity * curvature;

  // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = reverse ? -velocity : velocity;
  cmd_vel.twist.angular.z = angular_velocity;

  // instead of stopping, slow down immensely so steering is still possible
  if(stop) {
    cmd_vel.twist.linear.x /= 1E6;
    cmd_vel.twist.angular.z /= 1E6;
  }
  

  // std::cout<< 
  //   "curvature: " << curvature
  //   << " route_curvature: " << route_curvature
  //   << " d_error: " << d_error
  //   << " p_error: " << p_error
  //   << " d_contribution " << d_contribution
  //   << " p_contribution: " << p_contribution
  //   << " reverse: " << reverse
  //   << " vel: " << velocity
  //   << " twist.x: " << twist_pv.linear.x
  //   << std::endl;

  if(isnan(cmd_vel.twist.angular.z)) {
    RCLCPP_ERROR(logger_, "%s - %s", plugin_name_.c_str(), "nan twist");
    cmd_vel.twist.angular.z = 0.0;
  }


  return cmd_vel;
}


void LineFollowingController::publish_local_plan() {
  nav_msgs::msg::Path local_path;
  local_path.header.frame_id = global_plan_.header.frame_id;
  local_path.header.stamp = clock_->now();

  for(auto & node : route_->nodes) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = local_path.header.frame_id;
    pose.header.stamp = local_path.header.stamp;
    pose.pose.position.x = node.x;
    pose.pose.position.y = node.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, node.yaw);
    pose.pose.orientation.w = q.getW();
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();

    local_path.poses.push_back(pose);
  }
  local_plan_pub_->publish(local_path);
}

void LineFollowingController::setPlan(const nav_msgs::msg::Path & path)
{
  Route full_route;
  full_route.set_path(path);

  // calculate and optimize sub_routes at reversals
  routes_ = full_route.split_at_reversals();
  // std::cout << "route count: " << routes_.size() << std::endl;
  for(auto route : routes_) {
    route->smooth(route_smoothing_strength_);
    route->calc_angles_and_velocities(this->max_velocity_, this->max_deceleration_);
  }

  // std::cout << "publishing local path" << std::endl;

  global_plan_ = path;

  
  // set current route and position
  current_subroute_index_ = 0;
  route_ = routes_[current_subroute_index_];
  publish_local_plan();
  route_position_ = std::make_shared<Route::Position>(*route_);
}

void LineFollowingController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*percentage*/) {
}


}  // namespace nav2_line_following_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(nav2_line_following_controller::LineFollowingController, nav2_core::Controller)