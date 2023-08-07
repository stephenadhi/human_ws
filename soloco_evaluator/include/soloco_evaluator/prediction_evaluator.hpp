#ifndef PREDICTION_EVALUATOR_HPP_
#define PREDICTION_EVALUATOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "soloco_interfaces/msg/evaluation_metrics.hpp"
#include "soloco_interfaces/msg/agent_future.hpp"
#include "soloco_interfaces/msg/agent_futures.hpp"

class PredictionEvaluator : public rclcpp::Node
{
public:
  PredictionEvaluator();

private:
  void local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void agent_futures_callback(const soloco_interfaces::msg::AgentFutures::SharedPtr msg);
  float evaluate_obstacle_agent_collision();
  float evaluate_agent_agent_collision();

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_subscriber_;
  rclcpp::Subscription<soloco_interfaces::msg::AgentFutures>::SharedPtr agent_futures_subscriber_;
  rclcpp::Publisher<soloco_interfaces::msg::EvaluationMetrics>::SharedPtr evaluation_metrics_publisher_;

  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;
  std::shared_ptr<const std::vector<soloco_interfaces::msg::AgentFuture>> agent_futures_;

  float agent_radius_;
};

#endif  // PREDICTION_EVALUATOR_HPP_