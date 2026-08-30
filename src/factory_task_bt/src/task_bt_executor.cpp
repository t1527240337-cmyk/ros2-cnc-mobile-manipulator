#include <algorithm>
#include <chrono>
#include <memory>
#include <future>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "factory_interfaces/action/execute_order.hpp"
#include "factory_interfaces/action/execute_robot_task.hpp"
#include "factory_interfaces/msg/machine_state.hpp"
#include "factory_interfaces/srv/get_factory_state.hpp"
#include "factory_task_bt/order_tree_runner.hpp"
#include "factory_task_bt/task_tree_runner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace factory_task_bt
{

using namespace std::chrono_literals;
using ExecuteOrder = factory_interfaces::action::ExecuteOrder;
using ExecuteRobotTask = factory_interfaces::action::ExecuteRobotTask;
using MachineState = factory_interfaces::msg::MachineState;
using GetFactoryState = factory_interfaces::srv::GetFactoryState;
using BatteryState = sensor_msgs::msg::BatteryState;
using OrderGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteOrder>;
using TaskGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteRobotTask>;

class TaskBtExecutor : public rclcpp::Node
{
public:
  TaskBtExecutor()
  : Node("factory_task_bt_executor")
  {
    declare_parameter<std::vector<std::string>>(
      "machine_ids", {"machine_1", "machine_2", "machine_3"});
    declare_parameter<std::vector<std::string>>(
      "default_machine_order", {"machine_2", "machine_1", "machine_3"});
    declare_parameter<double>("machine_selection_timeout", 30.0);
    declare_parameter<double>("machine_state_max_age", 2.0);
    declare_parameter<int>("finished_bin_capacity", 4);

    machine_ids_ = get_parameter("machine_ids").as_string_array();
    default_machine_order_ =
      get_parameter("default_machine_order").as_string_array();
    validateMachineConfiguration();

    task_runner_ = std::make_unique<TaskTreeRunner>(this);
    order_runner_ = std::make_unique<OrderTreeRunner>(this, task_runner_.get());
    factory_state_client_ = create_client<GetFactoryState>("/factory/get_state");

    for (const auto & machine_id : machine_ids_) {
      machine_subscriptions_.push_back(create_subscription<MachineState>(
        "/" + machine_id + "/state", rclcpp::QoS(10),
        [this](const MachineState & message) {rememberMachine(message);}));
    }
    battery_subscription_ = create_subscription<BatteryState>(
      "/battery_state", rclcpp::QoS(10),
      [this](const BatteryState & message) {
        if (message.percentage >= 0.0 && message.percentage <= 1.0) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          battery_percentage_ = message.percentage;
        }
      });

    task_server_ = rclcpp_action::create_server<ExecuteRobotTask>(
      this, "/factory/execute_robot_task",
      [this](const rclcpp_action::GoalUUID &, const auto goal) {
        return acceptTask(goal);
      },
      [](const std::shared_ptr<TaskGoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<TaskGoalHandle> handle) {
        std::thread([this, handle]() {executeTask(handle);}).detach();
      });
    order_server_ = rclcpp_action::create_server<ExecuteOrder>(
      this, "/factory/execute_order",
      [this](const rclcpp_action::GoalUUID &, const auto goal) {
        return acceptOrder(goal);
      },
      [](const std::shared_ptr<OrderGoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<OrderGoalHandle> handle) {
        std::thread([this, handle]() {executeOrder(handle);}).detach();
      });

    RCLCPP_INFO(
      get_logger(),
      "BehaviorTree.CPP executor ready; XML owns order and task flow");
  }

private:
  struct TimedMachineState
  {
    MachineState message;
    std::chrono::steady_clock::time_point received;
  };

  class BusyRelease
  {
  public:
    explicit BusyRelease(TaskBtExecutor * owner) : owner_(owner) {}
    ~BusyRelease() {owner_->releaseRobot();}
  private:
    TaskBtExecutor * owner_;
  };

  rclcpp_action::GoalResponse acceptTask(
    const std::shared_ptr<const ExecuteRobotTask::Goal> goal)
  {
    if (!validTask(*goal)) {
      RCLCPP_WARN(get_logger(), "Rejected malformed robot task %s", goal->task_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return reserveRobot() ?
      rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
      rclcpp_action::GoalResponse::REJECT;
  }

  rclcpp_action::GoalResponse acceptOrder(
    const std::shared_ptr<const ExecuteOrder::Goal> goal)
  {
    if (!validOrder(*goal)) {
      RCLCPP_WARN(get_logger(), "Rejected malformed physical order %s", goal->order_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    return reserveRobot() ?
      rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
      rclcpp_action::GoalResponse::REJECT;
  }

  bool validTask(const ExecuteRobotTask::Goal & goal) const
  {
    const bool known_kind =
      goal.task_kind == ExecuteRobotTask::Goal::LOAD_RAW ||
      goal.task_kind == ExecuteRobotTask::Goal::UNLOAD_FINISHED;
    return !goal.task_id.empty() && known_kind &&
      knownMachine(goal.machine_id) && !goal.part_id.empty();
  }

  bool validOrder(const ExecuteOrder::Goal & goal) const
  {
    if (goal.order_id.empty() || goal.quantity < 1) {
      return false;
    }
    return std::all_of(
      goal.allowed_machine_ids.begin(), goal.allowed_machine_ids.end(),
      [this](const std::string & id) {return knownMachine(id);});
  }

  bool knownMachine(const std::string & id) const
  {
    return std::find(machine_ids_.begin(), machine_ids_.end(), id) != machine_ids_.end();
  }

  bool reserveRobot()
  {
    std::lock_guard<std::mutex> lock(busy_mutex_);
    if (robot_busy_) {
      RCLCPP_WARN(get_logger(), "Rejected goal because the physical robot is busy");
      return false;
    }
    robot_busy_ = true;
    return true;
  }

  void releaseRobot()
  {
    std::lock_guard<std::mutex> lock(busy_mutex_);
    robot_busy_ = false;
  }

  void executeTask(const std::shared_ptr<TaskGoalHandle> goal_handle)
  {
    BusyRelease release(this);
    const auto request = toRequest(*goal_handle->get_goal());
    const auto result = task_runner_->run(
      request,
      [goal_handle]() {return goal_handle->is_canceling();},
      [goal_handle, &request](const TaskProgress & progress) {
        auto feedback = std::make_shared<ExecuteRobotTask::Feedback>();
        feedback->phase = progress.phase;
        feedback->machine_id = request.machine_id;
        feedback->detail = progress.detail;
        goal_handle->publish_feedback(feedback);
      });

    auto response = std::make_shared<ExecuteRobotTask::Result>();
    response->success = result.success;
    response->retryable = result.retryable;
    response->error_code = result.error_code;
    response->message = result.success ?
      (request.task_kind == ExecuteRobotTask::Goal::LOAD_RAW ?
      "load task behavior tree completed" :
      "unload task behavior tree completed") : result.message;
    if (result.canceled) {
      goal_handle->canceled(response);
    } else if (result.success) {
      goal_handle->succeed(response);
    } else {
      goal_handle->abort(response);
    }
  }

  void executeOrder(const std::shared_ptr<OrderGoalHandle> goal_handle)
  {
    BusyRelease release(this);
    const auto goal = goal_handle->get_goal();
    const auto inventory = readInventory();
    if (!inventory) {
      auto result = std::make_shared<ExecuteOrder::Result>();
      result->success = false;
      result->completed = 0;
      result->error_code = PhysicalStep::Result::PRECONDITION_FAILED;
      result->message = "live factory inventory is unavailable or invalid";
      goal_handle->abort(result);
      return;
    }

    const OrderRequest request{
      goal->order_id, goal->quantity,
      allowedMachines(goal->allowed_machine_ids), goal->auto_recharge,
      inventory->first, inventory->second};

    const auto run = order_runner_->run(
      request,
      [goal_handle]() {return goal_handle->is_canceling();},
      [this, goal_handle](const OrderProgress & progress) {
        auto feedback = std::make_shared<ExecuteOrder::Feedback>();
        feedback->phase = progress.phase;
        feedback->current_machine_id = progress.machine_id;
        feedback->completed = progress.completed;
        feedback->total = progress.total;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          feedback->battery_percentage =
            static_cast<float>(battery_percentage_ * 100.0);
        }
        feedback->detail = progress.detail;
        goal_handle->publish_feedback(feedback);
      },
      [this](
        const std::vector<std::string> & allowed,
        const std::set<std::string> & reserved,
        const OrderTreeRunner::CancelPredicate & cancel_requested,
        bool wait_for_machine)
      {
        return selectIdleMachine(
          allowed, reserved, cancel_requested, wait_for_machine);
      });

    auto result = std::make_shared<ExecuteOrder::Result>();
    result->success = run.success;
    result->completed = run.completed;
    result->error_code = run.error_code;
    result->message = run.message;
    if (run.canceled) {
      goal_handle->canceled(result);
    } else if (run.success) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  std::optional<std::string> selectIdleMachine(
    const std::vector<std::string> & allowed,
    const std::set<std::string> & reserved,
    const OrderTreeRunner::CancelPredicate & cancel_requested,
    bool wait_for_machine)
  {
    const auto timeout = std::chrono::duration<double>(
      get_parameter("machine_selection_timeout").as_double());
    const auto max_age = std::chrono::duration<double>(
      get_parameter("machine_state_max_age").as_double());
    const auto deadline = std::chrono::steady_clock::now() +
      (wait_for_machine ? timeout : std::chrono::duration<double>(0.0));
    do {
      if (cancel_requested()) {
        return std::nullopt;
      }
      const auto now = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (const auto & id : allowed) {
          if (reserved.count(id) != 0) {
            continue;
          }
          const auto found = machine_states_.find(id);
          if (
            found != machine_states_.end() &&
            now - found->second.received <= max_age &&
            found->second.message.state == MachineState::IDLE &&
            !found->second.message.part_present)
          {
            return id;
          }
        }
      }
      if (wait_for_machine) {
        std::this_thread::sleep_for(100ms);
      }
    } while (wait_for_machine && std::chrono::steady_clock::now() < deadline);
    return std::nullopt;
  }

  RobotTaskRequest toRequest(const ExecuteRobotTask::Goal & goal) const
  {
    return RobotTaskRequest{
      goal.task_id, goal.task_kind, goal.machine_id, goal.part_id,
      goal.auto_recharge};
  }

  std::vector<std::string> allowedMachines(
    const std::vector<std::string> & requested) const
  {
    const auto & source = requested.empty() ? default_machine_order_ : requested;
    std::vector<std::string> unique;
    for (const auto & id : source) {
      if (std::find(unique.begin(), unique.end(), id) == unique.end()) {
        unique.push_back(id);
      }
    }
    return unique;
  }

  std::optional<std::pair<unsigned, unsigned>> readInventory()
  {
    if (!factory_state_client_->wait_for_service(2s)) {
      return std::nullopt;
    }
    auto future = factory_state_client_->async_send_request(
      std::make_shared<GetFactoryState::Request>());
    if (future.wait_for(2s) != std::future_status::ready) {
      return std::nullopt;
    }
    const auto response = future.get();
    const auto capacity = get_parameter("finished_bin_capacity").as_int();
    if (capacity < 1 || response->finished_part_count >
      static_cast<unsigned>(capacity))
    {
      return std::nullopt;
    }
    return std::make_pair(
      static_cast<unsigned>(response->raw_part_count),
      static_cast<unsigned>(capacity) -
      static_cast<unsigned>(response->finished_part_count));
  }

  void rememberMachine(const MachineState & message)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    machine_states_[message.machine_id] =
      TimedMachineState{message, std::chrono::steady_clock::now()};
  }

  void validateMachineConfiguration() const
  {
    if (machine_ids_.empty() || default_machine_order_.empty()) {
      throw std::invalid_argument("machine configuration cannot be empty");
    }
    for (const auto & id : default_machine_order_) {
      if (!knownMachine(id)) {
        throw std::invalid_argument("default_machine_order contains " + id);
      }
    }
  }

  std::unique_ptr<TaskTreeRunner> task_runner_;
  std::unique_ptr<OrderTreeRunner> order_runner_;
  std::vector<std::string> machine_ids_;
  std::vector<std::string> default_machine_order_;
  std::vector<rclcpp::Subscription<MachineState>::SharedPtr> machine_subscriptions_;
  rclcpp::Subscription<BatteryState>::SharedPtr battery_subscription_;
  rclcpp_action::Server<ExecuteRobotTask>::SharedPtr task_server_;
  rclcpp_action::Server<ExecuteOrder>::SharedPtr order_server_;

  mutable std::mutex busy_mutex_;
  bool robot_busy_{false};
  mutable std::mutex state_mutex_;
  rclcpp::Client<GetFactoryState>::SharedPtr factory_state_client_;
  std::unordered_map<std::string, TimedMachineState> machine_states_;
  double battery_percentage_{0.0};
};

}  // namespace factory_task_bt

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<factory_task_bt::TaskBtExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
