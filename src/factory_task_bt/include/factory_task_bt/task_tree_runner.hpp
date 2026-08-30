#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "factory_task_bt/task_context.hpp"
#include "rclcpp/rclcpp.hpp"

namespace factory_task_bt
{

struct RobotTaskRequest
{
  std::string task_id;
  std::uint8_t task_kind{0};
  std::string machine_id;
  std::string part_id;
  bool auto_recharge{true};
};

struct TaskRunResult
{
  bool success{false};
  bool canceled{false};
  bool retryable{false};
  std::uint16_t error_code{PhysicalStep::Result::EXECUTION_FAILED};
  std::string message;
};

class TaskTreeRunner
{
public:
  using CancelPredicate = std::function<bool()>;
  using FeedbackCallback = std::function<void(const TaskProgress &)>;

  explicit TaskTreeRunner(rclcpp::Node * node);

  TaskRunResult run(
    const RobotTaskRequest & request,
    const CancelPredicate & cancel_requested,
    const FeedbackCallback & publish_feedback);

private:
  void registerStepNodes();
  bool abortSession(const std::string & task_id, std::string & detail);
  std::string treePath(std::uint8_t task_kind) const;

  rclcpp::Node * node_;
  PhysicalStepClient::SharedPtr physical_steps_;
  BT::BehaviorTreeFactory factory_;
  std::string tree_directory_;
};

}  // namespace factory_task_bt
