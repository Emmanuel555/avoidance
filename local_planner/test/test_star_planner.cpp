#include <gtest/gtest.h>

#include "../src/nodes/star_planner.h"
#include "../src/nodes/tree_node.h"

using namespace avoidance;

TEST(PlannerFunctions, buildTree) {
  // GIVEN
  ros::Time::init();
  StarPlanner star_planner;
  Box box(7.0f);
  float horizontal_fov = 90.0f;
  float vertical_fov = 45.0f;
  geometry_msgs::PoseStamped curr_pos;
  curr_pos.pose.position.x = 0.0f;
  curr_pos.pose.position.y = 0.0f;
  curr_pos.pose.position.z = 5.0f;
  curr_pos.pose.orientation.w = 1.0f;
  curr_pos.pose.orientation.x = 0.0f;
  curr_pos.pose.orientation.y = 0.0f;
  curr_pos.pose.orientation.z = 0.0f;
  star_planner.setPose(curr_pos);
  geometry_msgs::Point goal;
  goal.x = 2.0f;
  goal.y = 10.0f;
  goal.z = 5.0f;
  star_planner.setGoal(goal);

  avoidance::LocalPlannerNodeConfig config =
      avoidance::LocalPlannerNodeConfig::__getDefault__();
  config.childs_per_node_ = 2;
  config.n_expanded_nodes_ = 10;
  config.tree_node_distance_ = 1;
  config.tree_discount_factor_ = 0.8;
  star_planner.dynamicReconfigureSetStarParams(config, 1);

  const pcl::PointCloud<pcl::PointXYZ> reprojected_points;
  const std::vector<double> reprojected_points_age;
  const std::vector<double> reprojected_points_dist;
  const nav_msgs::GridCells path_waypoints;

  star_planner.setParams(10, 1.0, path_waypoints, 0.0, 0.2);
  star_planner.setFOV(horizontal_fov, vertical_fov);
  star_planner.setReprojectedPoints(reprojected_points, reprojected_points_age,
                                    reprojected_points_dist);
  star_planner.setCostParams(2.0, 1.5, 4.0, 4.0);
  star_planner.setBoxSize(box, 7.0f);
  std::vector<pcl::PointCloud<pcl::PointXYZ>> complete_cloud;
  float distance = 5.0f;
  float half_fov_y = distance * std::tan(horizontal_fov * M_PI / 180.f / 2.f);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (float y = -1.0f; y <= half_fov_y - 1.0f; y += 0.01f) {
    for (float z = -1.0f; z <= 1.0f; z += 0.1f) {
      cloud.push_back(pcl::PointXYZ(y, distance, z + 5));
    }
  }
  complete_cloud.push_back(std::move(cloud));
  star_planner.setCloud(complete_cloud);

  // WHEN
  star_planner.buildLookAheadTree();

  // THEN: none of the tree nodes is inside the obstacle
  for (int i = 0; i < star_planner.tree_.size(); i++) {
    Eigen::Vector3f node = star_planner.tree_[i].getPosition();
    bool node_inside_obstacle =
        node.x() >= -1.0f && node.x() < half_fov_y - 1.0f && node.y() >= 4.8f &&
        node.y() <= 5.2f && node.z() >= 4.0f && node.z() <= 6.0f;
    ASSERT_FALSE(node_inside_obstacle);
  }
}
