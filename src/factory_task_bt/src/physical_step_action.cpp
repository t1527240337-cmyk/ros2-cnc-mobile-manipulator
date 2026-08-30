#include "factory_task_bt/physical_step_action.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "action_msgs/msg/goal_status.hpp"

namespace factory_task_bt
{

PhysicalStepAction::PhysicalStepAction(
  const std::string & name, const BT::NodeConfig & config,
  std::uint8_t step_kind)
: BT::StatefulActionNode(name, config), step_kind_(step_kind)
{
  context_ = config.blackboard->get<std::shared_ptr<TaskContext>>("task_context");
  if (!context_) {
    throw BT::RuntimeError(name + ": task_context is unavailable");
  }
}

BT::PortsList PhysicalStepAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("task_id"),
    BT::InputPort<std::string>("station_id", std::string{}, ""),
    BT::InputPort<std::string>("machine_id", std::string{}, ""),
    BT::InputPort<std::string>("part_id", std::string{}, ""),
    BT::InputPort<bool>("auto_recharge", true, ""),
    BT::InputPort<bool>("stow_arm", false, ""),
    BT::InputPort<double>("result_timeout_sec", 60.0, ""),
  };
}

PhysicalStep::Goal PhysicalStepAction::buildGoal() const
{
  PhysicalStep::Goal goal;
  goal.step_kind = step_kind_;

  const auto task_id = getInput<std::string>("task_id");
  if (!task_id || task_id.value().empty()) {
    throw BT::RuntimeError(name() + ": task_id is required");
  }
  goal.task_id = task_id.value();
  goal.station_id = getInput<std::string>("station_id").value();
  goal.machine_id = getInput<std::string>("machine_id").value();
  goal.part_id = getInput<std::string>("part_id").value();
  goal.auto_recharge = getInput<bool>("auto_recharge").value();
  goal.stow_arm = getInput<bool>("stow_arm").value();
  return goal;
}

BT::NodeStatus PhysicalStepAction::onStart()
{
  if (!context_->client()->action_server_is_ready()) {
    const auto detail = phaseName() + ": physical step server is unavailable";
    context_->failed(
      phaseName(), detail, PhysicalStep::Result::DEPENDENCY_UNAVAILABLE, false);
    return BT::NodeStatus::FAILURE;
  }

  PhysicalStep::Goal goal;
  try {
    goal = buildGoal();
  } catch (const std::exception & error) {
    context_->failed(
      phaseName(), error.what(), PhysicalStep::Result::INVALID_REQUEST, false);
    return BT::NodeStatus::FAILURE;
  }

  result_timeout_sec_ = getInput<double>("result_timeout_sec").value();
  if (result_timeout_sec_ < 0.0) {
    context_->failed(
      phaseName(), "result_timeout_sec cannot be negative",
      PhysicalStep::Result::INVALID_REQUEST, false);
    return BT::NodeStatus::FAILURE;
  }
  result_recovery_requested_ = false;
  context_->started(phaseName());
  result_reported_ = false;
  started_at_ = std::chrono::steady_clock::now();
  async_ = std::make_shared<AsyncState>();
  async_->last_activity_at = started_at_;
  const std::weak_ptr<AsyncState> weak_state(async_);
  const std::weak_ptr<TaskContext> weak_context(context_);
  const auto phase = phaseName();

  PhysicalStepClient::SendGoalOptions options;
  options.goal_response_callback =
    [weak_state, weak_context](const GoalHandle::SharedPtr & handle) {
      const auto state = weak_state.lock();
      if (!state) {
        return;
      }
      std::lock_guard<std::mutex> lock(state->mutex);
      state->last_activity_at = std::chrono::steady_clock::now();
      if (!handle) {
        state->transport_error = "physical step goal was rejected";
        return;
      }
      state->goal_handle = handle;
      if (state->cancel_requested) {
        if (const auto context = weak_context.lock()) {
          context->client()->async_cancel_goal(handle);
        }
      }
    };
  options.feedback_callback =
    [weak_state, weak_context, phase](
      GoalHandle::SharedPtr,
      const std::shared_ptr<const PhysicalStep::Feedback> feedback) {
      if (const auto state = weak_state.lock()) {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->last_activity_at = std::chrono::steady_clock::now();
      }
      if (const auto context = weak_context.lock()) {
        context->feedback(phase, feedback->detail);
      }
    };
  options.result_callback =
    [weak_state](const GoalHandle::WrappedResult & result) {
      const auto state = weak_state.lock();
      if (!state) {
        return;
      }
      std::lock_guard<std::mutex> lock(state->mutex);
      state->last_activity_at = std::chrono::steady_clock::now();
      state->result = result;
    };

  try {
    context_->client()->async_send_goal(goal, options);
  } catch (const std::exception & error) {
    context_->failed(
      phaseName(), error.what(), PhysicalStep::Result::DEPENDENCY_UNAVAILABLE,
      false);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PhysicalStepAction::onRunning()
{
  return readResult();
}

BT::NodeStatus PhysicalStepAction::readResult()
{
  std::optional<GoalHandle::WrappedResult> wrapped;
  std::string transport_error;
  std::chrono::steady_clock::time_point last_activity_at;
  {
    std::lock_guard<std::mutex> lock(async_->mutex);
    wrapped = async_->result;
    transport_error = async_->transport_error;
    last_activity_at = async_->last_activity_at;
  }

  if (!transport_error.empty()) {
    context_->failed(
      phaseName(), phaseName() + ": " + transport_error,
      PhysicalStep::Result::DEPENDENCY_UNAVAILABLE, false);
    return BT::NodeStatus::FAILURE;
  }
  if (!wrapped) {
    const auto now = std::chrono::steady_clock::now();
    const double inactive_for = std::chrono::duration<double>(
      now - last_activity_at).count();
    if (
      result_timeout_sec_ > 0.0 && inactive_for >= result_timeout_sec_ &&
      !result_recovery_requested_)
    {
      GoalHandle::SharedPtr handle;
      {
        std::lock_guard<std::mutex> lock(async_->mutex);
        handle = async_->goal_handle;
      }
      if (!handle) {
        context_->failed(
          phaseName(), phaseName() + ": goal response was not retained",
          PhysicalStep::Result::DEPENDENCY_UNAVAILABLE, true);
        return BT::NodeStatus::FAILURE;
      }

      const std::weak_ptr<AsyncState> weak_state(async_);
      context_->client()->async_get_result(
        handle,
        [weak_state](const GoalHandle::WrappedResult & recovered) {
          const auto state = weak_state.lock();
          if (!state) {
            return;
          }
          std::lock_guard<std::mutex> lock(state->mutex);
          state->result = recovered;
        });
      result_recovery_requested_ = true;
      result_recovery_deadline_ = now + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(kResultRecoveryGraceSec));
      RCLCPP_WARN(
        context_->node()->get_logger(),
        "No feedback or terminal result for %s during %.1f s; querying "
        "the Action server's retained result",
        phaseName().c_str(), inactive_for);
      return BT::NodeStatus::RUNNING;
    }
    if (result_recovery_requested_ && now >= result_recovery_deadline_) {
      GoalHandle::SharedPtr handle;
      {
        std::lock_guard<std::mutex> lock(async_->mutex);
        handle = async_->goal_handle;
      }
      if (handle) {
        context_->client()->async_cancel_goal(handle);
      }
      context_->failed(
        phaseName(), phaseName() +
        ": Action server did not provide a terminal result after recovery",
        PhysicalStep::Result::DEPENDENCY_UNAVAILABLE, true);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  const bool succeeded =
    wrapped->code == rclcpp_action::ResultCode::SUCCEEDED &&
    wrapped->result && wrapped->result->success;
  if (succeeded) {
    if (!result_reported_) {
      const double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started_at_).count();
      RCLCPP_INFO(
        context_->node()->get_logger(),
        "BT physical step %s completed in %.3f s",
        phaseName().c_str(), elapsed);
      context_->feedback(phaseName(), wrapped->result->message);
      result_reported_ = true;
      return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::SUCCESS;
  }

  const auto error_code = wrapped->result ?
    wrapped->result->error_code : PhysicalStep::Result::EXECUTION_FAILED;
  const bool retryable = wrapped->result && wrapped->result->retryable;
  const std::string detail = wrapped->result ?
    wrapped->result->message : phaseName() + ": action returned no result";
  if (!result_reported_) {
    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started_at_).count();
    RCLCPP_ERROR(
      context_->node()->get_logger(),
      "BT physical step %s failed after %.3f s: %s",
      phaseName().c_str(), elapsed, detail.c_str());
    context_->failed(phaseName(), detail, error_code, retryable);
    result_reported_ = true;
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::FAILURE;
}

void PhysicalStepAction::onHalted()
{
  if (!async_) {
    return;
  }
  GoalHandle::SharedPtr handle;
  {
    std::lock_guard<std::mutex> lock(async_->mutex);
    async_->cancel_requested = true;
    handle = async_->goal_handle;
  }
  if (handle) {
    context_->client()->async_cancel_goal(handle);
  }
}

std::string PhysicalStepAction::phaseName() const
{
  return registrationName();
}

}  // namespace factory_task_bt
