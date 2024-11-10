#include "nav2_line_following_controller/route.h"
#include "gtest/gtest.h"

/*
Returns three node route:
(0,0) -> (1,0) -> (2,1)
*/

nav_msgs::msg::Path get_simple_test_path() {
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.header.stamp = rclcpp::Time(0);
  path.poses.resize(3);
  path.poses[0].pose.position.x = 0;
  path.poses[0].pose.position.y = 0;
  path.poses[0].pose.position.z = 0;
  path.poses[0].pose.orientation.x = 0;
  path.poses[0].pose.orientation.y = 0;
  path.poses[0].pose.orientation.z = 0;
  path.poses[0].pose.orientation.w = 1;
  path.poses[1].pose.position.x = 1;
  path.poses[1].pose.position.y = 0;
  path.poses[1].pose.position.z = 0;
  path.poses[1].pose.orientation.x = 0;
  path.poses[1].pose.orientation.y = 0;
  path.poses[1].pose.orientation.z = 0;
  path.poses[1].pose.orientation.w = 1;
  path.poses[2].pose.position.x = 2;
  path.poses[2].pose.position.y = 1;
  path.poses[2].pose.position.z = 0;
  path.poses[2].pose.orientation.x = 0;
  path.poses[2].pose.orientation.y = 0;
  path.poses[2].pose.orientation.z = 0;
  path.poses[2].pose.orientation.w = 1;
  return path;
}

TEST(route_tests, get_position_ahead) {
  const auto tolerance = 1e-6;
  Route route;
  nav_msgs::msg::Path path = get_simple_test_path();
  route.set_path(path);
  auto route_position = std::make_shared<Route::Position>(route);
  route_position->set_position({0, 0});
  
  {
    auto ahead = route.get_position_ahead(*route_position, 0.0);
    auto ahead_pose = route.get_pose_at_position(ahead.position);
    ASSERT_NEAR(ahead_pose.position.x, 0.0, tolerance);
  }

  {
    auto ahead = route.get_position_ahead(*route_position, 0.5);
    auto ahead_pose = route.get_pose_at_position(ahead.position);
    ASSERT_NEAR(ahead_pose.position.x, 0.5, tolerance);
  }

  {
    auto ahead = route.get_position_ahead(*route_position, 1.0 + sqrt(2)/2.0);
    auto ahead_pose = route.get_pose_at_position(ahead.position);
    ASSERT_NEAR(ahead_pose.position.x, 1.5, tolerance);
  }

  {
    auto ahead = route.get_position_ahead(*route_position, 5.0);
    auto ahead_pose = route.get_pose_at_position(ahead.position);
    ASSERT_NEAR(ahead_pose.position.x, 2.0, tolerance);
  }
}

TEST(route_tests, initialization) {
  Route route;
  nav_msgs::msg::Path path = get_simple_test_path();
  route.set_path(path);
  ASSERT_EQ(route.nodes.size(), 3U);
}

TEST(route_tests, traverse_route) {
  const auto tolerance = 1e-6;

  Route route;
  nav_msgs::msg::Path path = get_simple_test_path();
  route.set_path(path);

  double max_velocity = 2.0;
  double max_deceleration = 1.0;
  route.calc_angles_and_velocities(max_velocity, max_deceleration);


  auto route_position = std::make_shared<Route::Position>(route);
  route_position->set_position({0, 0});

  // start route
  ASSERT_EQ(route_position->index, 0U);
  ASSERT_EQ(route_position->cte, 0.0);
  ASSERT_EQ(route_position->progress, 0.0);
  ASSERT_NEAR(route.get_velocity(*route_position),  velocity_at_position(2.0, 1.0, 0.0), tolerance);

  // move along first segment with a little error above
  route_position->set_position({0.5, 0.1});
  ASSERT_EQ(route_position->index, 0U);
  ASSERT_EQ(route_position->cte, -0.1);
  ASSERT_EQ(route_position->progress, 0.5);

  // move further along first segment with a little error below
  route_position->set_position({0.6, -0.1});
  ASSERT_EQ(route_position->index, 0U);
  ASSERT_EQ(route_position->cte, 0.1);
  ASSERT_EQ(route_position->progress, 0.6);

  // move past first segment, below second
  route_position->set_position({2.0, 0.0});
  ASSERT_EQ(route_position->index, 1U);
  ASSERT_NEAR (route_position->cte, sqrt(2)/2.0, tolerance);
  ASSERT_EQ(route_position->progress, 0.5);

  // move further along second segment, no error
  route_position->set_position({1.9, 0.9});
  ASSERT_EQ(route_position->index, 1U);
  ASSERT_NEAR (route_position->cte, 0.0, tolerance);
  ASSERT_NEAR(route_position->progress, 0.9, tolerance);

  // move past end of route
  route_position->set_position({2.1, 1.1});
  ASSERT_EQ(route_position->index, 1U);
  ASSERT_EQ(route_position->done, true);
  ASSERT_NEAR (route_position->cte, 0.0, tolerance);
  ASSERT_NEAR(route_position->progress, 1.1, tolerance);
}

TEST(route_tests, smooth_route) {

  // start with a straight line route
  Route route;
  route.nodes.resize(5);
  for(auto i=0;i<5;++i) {
    route.nodes[i].x = i;
    route.nodes[i].y = 0.0;
  }

  {
    Route smoothed = route.smoothed(0.5);

    // check that smoothed route is identical
    for(int i=0;i<5;++i) {
      ASSERT_EQ(smoothed.nodes[i].x, route.nodes[i].x);
      ASSERT_EQ(smoothed.nodes[i].y, route.nodes[i].y);
    }
  }

  // middle nodes should be smoothed
  {
    route.nodes[2].y = 1.0;
    Route smoothed = route.smoothed(0.5);

    // end segments should not be touched
    ASSERT_EQ(smoothed.nodes[0].x, route.nodes[0].x);
    ASSERT_EQ(smoothed.nodes[0].y, route.nodes[0].y);
    ASSERT_EQ(smoothed.nodes[1].x, route.nodes[1].x);
    ASSERT_EQ(smoothed.nodes[1].y, route.nodes[1].y);
    ASSERT_EQ(smoothed.nodes[3].x, route.nodes[3].x);
    ASSERT_EQ(smoothed.nodes[3].y, route.nodes[3].y);
    ASSERT_EQ(smoothed.nodes[4].x, route.nodes[4].x);
    ASSERT_EQ(smoothed.nodes[4].y, route.nodes[4].y);

    // middle nodes should be smoothed
    ASSERT_LT(smoothed.nodes[2].y, 1.0);

    // original nodes should not be touched
    ASSERT_EQ(smoothed.nodes[2].x, route.nodes[2].x);

    // smooth (as opposed to smoothed) should modify the route nodes
    ASSERT_NE(route.nodes[2].y, smoothed.nodes[2].y);
    route.smooth(0.5);
    ASSERT_EQ(route.nodes[2].y, smoothed.nodes[2].y);

  }

  // higher smooth factor should cause more smoothing
  {
    auto smoothed_more = route.smoothed(0.9);
    auto smoothed_less = route.smoothed(0.1);
    ASSERT_LE(smoothed_more.nodes[2].y, 1.0);
    ASSERT_LE(smoothed_less.nodes[2].y, 1.0);
    ASSERT_GE(smoothed_more.nodes[2].y, 0.0);
    ASSERT_GE(smoothed_more.nodes[2].y, 0.0);


    ASSERT_LE(smoothed_more.nodes[2].y, smoothed_less.nodes[2].y);
  }
}