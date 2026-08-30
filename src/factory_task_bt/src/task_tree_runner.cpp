#include "factory_task_bt/task_tree_runner.hpp"

#include <chrono>
#include <thread>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "factory_interfaces/action/execute_robot_task.hpp"
#include "factory_task_bt/physical_step_action.hpp"

namespace factory_task_bt
{

using namespace std::chrono_literals;
using ExecuteRobotTask = factory_interfaces::action::ExecuteRobotTask;

TaskTreeRunner::TaskTreeRunner(rclcpp::Node * node)
: node_(node)
{
  physical_steps_ = rclcpp_action::create_client<PhysicalStep>(
    node_, "/factory/execute_physical_step");
  tree_directory_ =
    ament_index_cpp::get_package_share_directory("factory_task_bt") +
    "/behavior_trees";
  registerStepNodes();
}

void TaskTreeRunner::registerStepNodes()
{
  const auto register_step =
    [this](const std::string & name, std::uint8_t kind) {
      factory_.registerNodeType<PhysicalStepAction>(
        name, PhysicalStepAction::providedPorts(), kind);
    };

  register_step("BeginPhysicalTask", PhysicalStep::Goal::BEGIN_TASK);
  register_step("EnsureEnergy", PhysicalStep::Goal::ENSURE_ENERGY);
  register_step("CheckMachineIdle", PhysicalStep::Goal::CHECK_MACHINE_IDLE);
  register_step("DockStation", PhysicalStep::Goal::DOCK);
  register_step("UndockStation", PhysicalStep::Goal::UNDOCK);
  register_step("PickPart", PhysicalStep::Goal::PICK);
  register_step("PlacePart", PhysicalStep::Goal::PLACE);
  register_step("OpenMachineDoor", PhysicalStep::Goal::OPEN_DOOR);
  register_step("CloseMachineDoor", PhysicalStep::Goal::CLOSE_DOOR);
  register_step("StartMachine", PhysicalStep::Goal::START_MACHINE);
  register_step("ConfirmMachineLoad", PhysicalStep::Goal::CONFIRM_LOAD);
  register_step("ConfirmMachineUnload", PhysicalStep::Goal::CONFIRM_UNLOAD);
  register_step("WaitMachineDone", PhysicalStep::Goal::WAIT_MACHINE_DONE);
  register_step("CommitPickRaw", PhysicalStep::Goal::COMMIT_PICK_RAW);
  register_step(
    "CommitPlaceFinished", PhysicalStep::Goal::COMMIT_PLACE_FINISHED);
  register_step("CompletePhysicalTask", PhysicalStep::Goal::COMPLETE_TASK);
  register_step("AbortPhysicalTask", PhysicalStep::Goal::ABORT_TASK);
}

TaskRunResult TaskTreeRunner::run(
  const RobotTaskRequest & request,
  const CancelPredicate & cancel_requested,
  const FeedbackCallback & publish_feedback)
{
  TaskRunResult outcome;
  const auto context = std::make_shared<TaskContext>(node_, physical_steps_);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("task_context", context);
  blackboard->set("task_id", request.task_id);
  blackboard->set("machine_id", request.machine_id);
  blackboard->set("part_id", request.part_id);
  blackboard->set("auto_recharge", request.auto_recharge);

  BT::Tree tree;
  try {
    tree = factory_.createTreeFromFile(
      treePath(request.task_kind), blackboard);
  } catch (const std::exception & error) {
    outcome.error_code = PhysicalStep::Result::INVALID_REQUEST;
    outcome.message = std::string("cannot construct task tree: ") + error.what();
    return outcome;
  }

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  TaskProgress last_published;
  auto next_periodic_feedback = std::chrono::steady_clock::now();
  try {
    do {
      if (cancel_requested()) {
        tree.haltTree();
        std::string abort_detail;
        const bool cleaned = abortSession(request.task_id, abort_detail);
        outcome.canceled = true;
        outcome.error_code = PhysicalStep::Result::CANCELLED;
        outcome.message = cleaned ?
          "task canceled and physical session aborted" :
          "task canceled but physical session cleanup failed: " + abort_detail;
        return outcome;
      }

      status = tree.tickOnce();
      const auto progress = context->snapshot();
      const auto now = std::chrono::steady_clock::now();
      if (
        progress.phase != last_published.phase ||
        progress.detail != last_published.detail ||
        now >= next_periodic_feedback)
      {
        if (progress.phase != last_published.phase) {
          RCLCPP_INFO(
            node_->get_logger(), "BT task %s phase %s: %s",
            request.task_id.c_str(), progress.phase.c_str(),
            progress.detail.c_str());
        }
        publish_feedback(progress);
        last_published = progress;
        next_periodic_feedback = now + 1s;
      }
      if (status == BT::NodeStatus::RUNNING) {
        std::this_thread::sleep_for(20ms);
      }
    } while (status == BT::NodeStatus::RUNNING);
  } catch (const std::exception & error) {
    tree.haltTree();
    std::string abort_detail;
    abortSession(request.task_id, abort_detail);
    outcome.error_code = PhysicalStep::Result::EXECUTION_FAILED;
    outcome.message = std::string("behavior tree exception: ") + error.what();
    return outcome;
  }

  const auto progress = context->snapshot();
  if (status == BT::NodeStatus::SUCCESS) {
    outcome.success = true;
    outcome.error_code = PhysicalStep::Result::OK;
    outcome.message = "behavior tree completed";
    return outcome;
  }

  outcome.retryable = progress.retryable;
  outcome.error_code = progress.first_error.empty() ?
    PhysicalStep::Result::EXECUTION_FAILED : progress.error_code;
  outcome.message = progress.first_error.empty() ?
    "behavior tree returned FAILURE without an error result" :
    progress.first_error;
  return outcome;
}

bool TaskTreeRunner::abortSession(
  const std::string & task_id, std::string & detail)
{
  if (!physical_steps_->wait_for_action_server(2s)) {
    detail = "physical step server unavailable";
    return false;
  }

  PhysicalStep::Goal goal;
  goal.task_id = task_id;
  goal.step_kind = PhysicalStep::Goal::ABORT_TASK;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    auto handle_future = physical_steps_->async_send_goal(goal);
    if (handle_future.wait_for(1s) != std::future_status::ready) {
      detail = "abort goal response timed out";
      continue;
    }
    const auto handle = handle_future.get();
    if (!handle) {
      detail = "abort goal rejected while previous step is stopping";
      std::this_thread::sleep_for(100ms);
      continue;
    }
    auto result_future = physical_steps_->async_get_result(handle);
    if (result_future.wait_for(2s) != std::future_status::ready) {
      detail = "abort result timed out";
      return false;
    }
    const auto wrapped = result_future.get();
    if (
      wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped.result && wrapped.result->success)
    {
      detail = wrapped.result->message;
      return true;
    }
    detail = wrapped.result ? wrapped.result->message : "abort returned no result";
    return false;
  }
  return false;
}

std::string TaskTreeRunner::treePath(std::uint8_t task_kind) const
{
  if (task_kind == ExecuteRobotTask::Goal::LOAD_RAW) {
    return tree_directory_ + "/load_raw.xml";
  }
  if (task_kind == ExecuteRobotTask::Goal::UNLOAD_FINISHED) {
    return tree_directory_ + "/unload_finished.xml";
  }
  throw std::invalid_argument("unknown robot task kind");
}

}  // namespace factory_task_bt
