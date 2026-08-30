#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "factory_task_bt/task_context.hpp"

namespace factory_task_bt
{

class PhysicalStepAction : public BT::StatefulActionNode
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<PhysicalStep>;

  PhysicalStepAction(
    const std::string & name, const BT::NodeConfig & config,
    std::uint8_t step_kind);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  struct AsyncState
  {
    std::mutex mutex;
    GoalHandle::SharedPtr goal_handle;
    std::optional<GoalHandle::WrappedResult> result;
    std::string transport_error;
    std::chrono::steady_clock::time_point last_activity_at;
    bool cancel_requested{false};
  };

  PhysicalStep::Goal buildGoal() const;
  std::string phaseName() const;
  BT::NodeStatus readResult();

  std::uint8_t step_kind_;
  std::shared_ptr<TaskContext> context_;
  std::chrono::steady_clock::time_point started_at_;
  std::shared_ptr<AsyncState> async_;
  bool result_reported_{false};
  double result_timeout_sec_{0.0};
  bool result_recovery_requested_{false};
  std::chrono::steady_clock::time_point result_recovery_deadline_;
  static constexpr double kResultRecoveryGraceSec = 10.0;
};

}  // namespace factory_task_bt
