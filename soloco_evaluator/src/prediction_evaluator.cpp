#include "soloco_evaluator/prediction_evaluator.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

PredictionEvaluator::PredictionEvaluator() : Node("prediction_evaluator")
{
  std::string local_costmap_topic_, agent_futures_topic_;

  this->declare_parameter("agent_radius", 0.25);
  this->declare_parameter("local_costmap_topic", "local_costmap");
  this->declare_parameter("agent_futures_topic", "agent_futures");
  this->declare_parameter("evaluation_metrics_topic", "evaluation_metrics");

  agent_radius_ = this->get_parameter("agent_radius").as_double();
  local_costmap_topic_ = this->get_parameter("local_costmap_topic").as_string();
  agent_futures_topic_ = this->get_parameter("agent_futures_topic").as_string();

  local_costmap_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      local_costmap_topic_, 10, std::bind(&PredictionEvaluator::local_costmap_callback, this, std::placeholders::_1));

  agent_futures_subscriber_ = this->create_subscription<soloco_interfaces::msg::AgentFutures>(
      agent_futures_topic_, 10, std::bind(&PredictionEvaluator::agent_futures_callback, this, std::placeholders::_1));

  evaluation_metrics_publisher_ = this->create_publisher<soloco_interfaces::msg::EvaluationMetrics>(this->get_parameter("evaluation_metrics_topic").as_string(), 10);
}

void PredictionEvaluator::local_costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  local_costmap_ = msg;
}

void PredictionEvaluator::agent_futures_callback(const soloco_interfaces::msg::AgentFutures::SharedPtr msg)
{
  agent_futures_ = std::make_shared<const std::vector<soloco_interfaces::msg::AgentFuture>>(msg->futures);
  float obstacle_collision_percentage = evaluate_obstacle_agent_collision();
  float agent_collision_percentage = evaluate_agent_agent_collision();

  soloco_interfaces::msg::EvaluationMetrics evaluation_metrics_msg;
  evaluation_metrics_msg.agent_obstacle_collision = obstacle_collision_percentage;
  evaluation_metrics_msg.agent_agent_collision = agent_collision_percentage;
  evaluation_metrics_publisher_->publish(evaluation_metrics_msg);

  agent_futures_ = nullptr;
}

float PredictionEvaluator::evaluate_obstacle_agent_collision()
{
  int agent_radius_cells = agent_radius_ / local_costmap_->info.resolution;
  int total_agents = agent_futures_->size();
  int collision_agents = 0;

  for (const auto& future : *agent_futures_) {
    bool collision = false;
    for (const auto& pose : future.track.poses) {
      int x = pose.pose.position.x / local_costmap_->info.resolution;
      int y = pose.pose.position.y / local_costmap_->info.resolution;

      for (int dx = -agent_radius_cells; dx <= agent_radius_cells; ++dx) {
        for (int dy = -agent_radius_cells; dy <= agent_radius_cells; ++dy) {
          int nx = x + dx;
          int ny = y + dy;

          if (local_costmap_->data[ny * local_costmap_->info.width + nx] > 0) {
            collision = true;
            break;
          }
        }
        if (collision) {
          break;
        }
      }
      if (collision) {
        break;
      }
    }
    if (collision) {
      collision_agents++;
    }
  }

  return static_cast<float>(collision_agents) / total_agents * 100;
}

float PredictionEvaluator::evaluate_agent_agent_collision()
{
  int collision_free_futures = (*agent_futures_).size();
  
  for (size_t i = 0; i < (*agent_futures_).size(); ++i)
  {
    bool collision_detected = false;

    for (size_t j = i + 1; j < (*agent_futures_).size() && !collision_detected; ++j)
    {
      for (const auto& pose_i : (*agent_futures_)[i].track.poses)
      {
        for (const auto& pose_j : (*agent_futures_)[j].track.poses)
        {
          float dx = pose_i.pose.position.x - pose_j.pose.position.x;
          float dy = pose_i.pose.position.y - pose_j.pose.position.y;
          float distance = std::sqrt(dx*dx + dy*dy);

          if (distance < 2 * agent_radius_)
          {
            collision_free_futures--;
            collision_detected = true;
            break;
          }
        }

        if (collision_detected)
        {
          break;
        }
      }
    }
  }

  return static_cast<float>(collision_free_futures) / (*agent_futures_).size();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PredictionEvaluator>());
  rclcpp::shutdown();
  return 0;
}