#include "factory_task_bt/order_tree_runner.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/control_node.h"
#include "factory_interfaces/action/execute_robot_task.hpp"

namespace factory_task_bt
{

using ExecuteRobotTask = factory_interfaces::action::ExecuteRobotTask;

namespace
{

class RepeatWhile : public BT::ControlNode
{
public:
  RepeatWhile(const std::string & name, const BT::NodeConfig & config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    if (childrenCount() != 2) {
      throw BT::LogicError(name() + ": RepeatWhile requires condition and body children");
    }
    setStatus(BT::NodeStatus::RUNNING);

    const auto condition = children_nodes_.at(0)->executeTick();
    if (condition == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    if (condition == BT::NodeStatus::FAILURE || condition == BT::NodeStatus::SKIPPED) {
      resetChildren();
      return BT::NodeStatus::SUCCESS;
    }
    if (condition != BT::NodeStatus::SUCCESS) {
      throw BT::LogicError(name() + ": condition returned an invalid status");
    }

    const auto body = children_nodes_.at(1)->executeTick();
    if (body == BT::NodeStatus::SUCCESS) {
      resetChildren();
      return BT::NodeStatus::RUNNING;
    }
    if (body == BT::NodeStatus::FAILURE) {
      resetChildren();
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
};

struct OrderAssignment
{
  std::string machine_id;
  std::string part_id;
};

class OrderContext
{
public:
  OrderContext(
    rclcpp::Node * node, TaskTreeRunner * task_runner,
    OrderRequest request, OrderTreeRunner::CancelPredicate cancel_requested,
    OrderTreeRunner::FeedbackCallback publish_feedback,
    OrderTreeRunner::MachineSelector select_machine)
  : node_(node), task_runner_(task_runner), request_(std::move(request)),
    cancel_requested_(std::move(cancel_requested)),
    publish_feedback_(std::move(publish_feedback)),
    select_machine_(std::move(select_machine))
  {
  }

  bool initialize()
  {
    if (request_.quantity < 1) {
      fail(
        PhysicalStep::Result::INVALID_REQUEST,
        "order quantity must be positive");
      return false;
    }
    if (request_.quantity > request_.available_raw_parts) {
      fail(
        PhysicalStep::Result::INVALID_REQUEST,
        "order quantity exceeds the live raw inventory");
      return false;
    }
    if (request_.quantity > request_.available_finished_slots) {
      fail(
        PhysicalStep::Result::INVALID_REQUEST,
        "order quantity exceeds available finished-bin slots");
      return false;
    }
    if (request_.allowed_machine_ids.empty()) {
      fail(
        PhysicalStep::Result::INVALID_REQUEST,
        "order has no allowed machines");
      return false;
    }
    publish("OrderInitialized", "", "BehaviorTree.CPP order workflow started");
    return true;
  }

  bool hasRemainingParts() const
  {
    return next_part_index_ < request_.quantity;
  }

  bool prepareNextBatch()
  {
    if (!assignments_.empty()) {
      fail(
        PhysicalStep::Result::EXECUTION_FAILED,
        "cannot prepare a new batch before finishing the current batch");
      return false;
    }

    const unsigned remaining = request_.quantity - next_part_index_;
    const unsigned batch_size = std::min<unsigned>(
      remaining, request_.allowed_machine_ids.size());
    std::set<std::string> reserved;
    for (unsigned offset = 0; offset < batch_size; ++offset) {
      const auto machine = select_machine_(
        request_.allowed_machine_ids, reserved, cancel_requested_,
        assignments_.empty());
      if (!machine) {
        if (!assignments_.empty() && !cancel_requested_()) {
          publish(
            "BatchCapacityReached", "",
            "no additional IDLE machine is available; running a partial batch");
          break;
        }
        if (cancel_requested_()) {
          fail(PhysicalStep::Result::CANCELLED, "order canceled during machine selection");
        } else {
          fail(
            PhysicalStep::Result::EXECUTION_FAILED,
            "no fresh IDLE machine became available before timeout");
        }
        return false;
      }

      reserved.insert(*machine);
      const unsigned serial = next_part_index_ + offset + 1;
      const std::string logical_part_id =
        request_.order_id + ":part:" + std::to_string(serial);
      assignments_.push_back(OrderAssignment{*machine, logical_part_id});

      const bool reassigned =
        offset == 0 && *machine != request_.allowed_machine_ids.front();
      publish(
        reassigned ? "MachineReassigned" : "MachineAssigned", *machine,
        reassigned ?
        "preferred machine unavailable; selected a fresh IDLE machine" :
        "selected a fresh IDLE machine");
    }

    load_index_ = 0;
    unload_index_ = 0;
    publish("BatchPrepared", "", "assigned all parts in the next production batch");
    return true;
  }

  bool hasPendingLoads() const
  {
    return load_index_ < assignments_.size();
  }

  bool hasPendingUnloads() const
  {
    return unload_index_ < assignments_.size();
  }

  bool executeNextLoad()
  {
    if (!hasPendingLoads()) {
      fail(PhysicalStep::Result::EXECUTION_FAILED, "load cursor exceeded its batch");
      return false;
    }
    const auto & assignment = assignments_.at(load_index_);
    const auto result = runTask(assignment, ExecuteRobotTask::Goal::LOAD_RAW);
    if (!result.success) {
      fail(result.error_code, result.message);
      return false;
    }
    ++load_index_;
    return true;
  }

  bool executeNextUnload()
  {
    if (!hasPendingUnloads()) {
      fail(PhysicalStep::Result::EXECUTION_FAILED, "unload cursor exceeded its batch");
      return false;
    }
    const auto & assignment = assignments_.at(unload_index_);
    const auto result = runTask(assignment, ExecuteRobotTask::Goal::UNLOAD_FINISHED);
    ++unload_index_;

    if (result.canceled) {
      fail(PhysicalStep::Result::CANCELLED, result.message);
      return false;
    }
    if (!result.success) {
      if (unload_failures_.empty()) {
        failure_error_code_ = result.error_code;
      }
      unload_failures_.push_back(assignment.machine_id + ": " + result.message);
      publish(
        "UnloadFailed", assignment.machine_id,
        "recorded unload failure; continuing recovery of other loaded machines");
      return true;
    }

    ++completed_;
    publish("PartCompleted", assignment.machine_id, "finished part committed to output bin");
    return true;
  }

  bool finishBatch()
  {
    if (hasPendingLoads() || hasPendingUnloads()) {
      fail(
        PhysicalStep::Result::EXECUTION_FAILED,
        "behavior tree attempted to finish an incomplete batch");
      return false;
    }
    if (!unload_failures_.empty()) {
      std::string detail = "batch recovery incomplete";
      for (const auto & failure : unload_failures_) {
        detail += "; " + failure;
      }
      // Other already-loaded machines were recovered first, but a held or
      // unplaced part makes it unsafe to begin another raw-material load.
      fail(failure_error_code_, detail);
      return false;
    }
    next_part_index_ += assignments_.size();
    assignments_.clear();
    load_index_ = 0;
    unload_index_ = 0;
    publish("BatchCompleted", "", "production batch completed");
    return true;
  }

  bool finalize()
  {
    if (next_part_index_ != request_.quantity || !assignments_.empty()) {
      fail(
        PhysicalStep::Result::EXECUTION_FAILED,
        "behavior tree reached finalization with unfinished assignments");
      return false;
    }
    if (!unload_failures_.empty()) {
      std::string detail = "order incomplete";
      for (const auto & failure : unload_failures_) {
        detail += "; " + failure;
      }
      fail(failure_error_code_, detail);
      return false;
    }
    if (completed_ != request_.quantity) {
      fail(
        PhysicalStep::Result::EXECUTION_FAILED,
        "completed count does not match the requested quantity");
      return false;
    }
    publish("OrderCompleted", "", "all physical batches completed");
    return true;
  }

  OrderRunResult result(bool tree_succeeded, bool canceled) const
  {
    OrderRunResult result;
    result.success = tree_succeeded && !canceled;
    result.canceled = canceled;
    result.completed = completed_;
    if (canceled) {
      result.error_code = PhysicalStep::Result::CANCELLED;
      result.message = "physical order canceled";
    } else if (result.success) {
      result.error_code = PhysicalStep::Result::OK;
      result.message = "BehaviorTree.CPP physical order completed";
    } else {
      result.error_code = failure_error_code_;
      result.message = failure_message_.empty() ?
        "order behavior tree returned FAILURE without an error" : failure_message_;
    }
    return result;
  }

private:
  TaskRunResult runTask(const OrderAssignment & assignment, std::uint8_t task_kind)
  {
    const bool loading = task_kind == ExecuteRobotTask::Goal::LOAD_RAW;
    const RobotTaskRequest task{
      request_.order_id + (loading ? ":load:" : ":unload:") + assignment.part_id,
      task_kind, assignment.machine_id, assignment.part_id,
      request_.auto_recharge};

    return task_runner_->run(
      task, cancel_requested_,
      [this, &assignment](const TaskProgress & progress) {
        publish(progress.phase, assignment.machine_id, progress.detail);
      });
  }

  void publish(
    const std::string & phase, const std::string & machine_id,
    const std::string & detail) const
  {
    publish_feedback_(OrderProgress{
      phase, machine_id, completed_, request_.quantity, detail});
  }

  void fail(std::uint16_t error_code, const std::string & detail)
  {
    if (failure_message_.empty()) {
      failure_error_code_ = error_code;
      failure_message_ = detail;
    }
    RCLCPP_ERROR(node_->get_logger(), "Order BT failure: %s", detail.c_str());
  }

  rclcpp::Node * node_;
  TaskTreeRunner * task_runner_;
  OrderRequest request_;
  OrderTreeRunner::CancelPredicate cancel_requested_;
  OrderTreeRunner::FeedbackCallback publish_feedback_;
  OrderTreeRunner::MachineSelector select_machine_;
  std::vector<OrderAssignment> assignments_;
  std::vector<std::string> unload_failures_;
  unsigned next_part_index_{0};
  std::size_t load_index_{0};
  std::size_t unload_index_{0};
  unsigned completed_{0};
  std::uint16_t failure_error_code_{PhysicalStep::Result::EXECUTION_FAILED};
  std::string failure_message_;
};

std::shared_ptr<OrderContext> orderContext(const BT::NodeConfig & config)
{
  auto context = config.blackboard->get<std::shared_ptr<OrderContext>>("order_context");
  if (!context) {
    throw BT::RuntimeError("order_context is unavailable");
  }
  return context;
}

class InitializeOrder : public BT::SyncActionNode
{
public:
  InitializeOrder(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config), context_(orderContext(config)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return context_->initialize() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<OrderContext> context_;
};

class OrderHasRemainingParts : public BT::ConditionNode
{
public:
  OrderHasRemainingParts(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config), context_(orderContext(config)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return context_->hasRemainingParts() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<OrderContext> context_;
};

class PrepareNextBatch : public BT::SyncActionNode
{
public:
  PrepareNextBatch(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config), context_(orderContext(config)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return context_->prepareNextBatch() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<OrderContext> context_;
};

class BatchHasPendingLoads : public BT::ConditionNode
{
public:
  BatchHasPendingLoads(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config), context_(orderContext(config)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return context_->hasPendingLoads() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<OrderContext> context_;
};

class ExecuteNextLoad : public BT::SyncActionNode
{
public:
  ExecuteNextLoad(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config), context_(orderContext(config)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return context_->executeNextLoad() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<OrderContext> context_;
};

class BatchHasPendingUnloads : public BT::ConditionNode
{
public:
  BatchHasPendingUnloads(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config), context_(orderContext(config)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return context_->hasPendingUnloads() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<OrderContext> context_;
};

class ExecuteNextUnload : public BT::SyncActionNode
{
public:
  ExecuteNextUnload(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config), context_(orderContext(config)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return context_->executeNextUnload() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<OrderContext> context_;
};

class FinishBatch : public BT::SyncActionNode
{
public:
  FinishBatch(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config), context_(orderContext(config)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return context_->finishBatch() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<OrderContext> context_;
};

class FinalizeOrder : public BT::SyncActionNode
{
public:
  FinalizeOrder(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config), context_(orderContext(config)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus tick() override
  {
    return context_->finalize() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
private:
  std::shared_ptr<OrderContext> context_;
};

}  // namespace

OrderTreeRunner::OrderTreeRunner(rclcpp::Node * node, TaskTreeRunner * task_runner)
: node_(node), task_runner_(task_runner)
{
  tree_path_ =
    ament_index_cpp::get_package_share_directory("factory_task_bt") +
    "/behavior_trees/execute_order.xml";
  registerOrderNodes();
}

void OrderTreeRunner::registerOrderNodes()
{
  factory_.registerNodeType<RepeatWhile>("RepeatWhile");
  factory_.registerNodeType<InitializeOrder>("InitializeOrder");
  factory_.registerNodeType<OrderHasRemainingParts>("OrderHasRemainingParts");
  factory_.registerNodeType<PrepareNextBatch>("PrepareNextBatch");
  factory_.registerNodeType<BatchHasPendingLoads>("BatchHasPendingLoads");
  factory_.registerNodeType<ExecuteNextLoad>("ExecuteNextLoad");
  factory_.registerNodeType<BatchHasPendingUnloads>("BatchHasPendingUnloads");
  factory_.registerNodeType<ExecuteNextUnload>("ExecuteNextUnload");
  factory_.registerNodeType<FinishBatch>("FinishBatch");
  factory_.registerNodeType<FinalizeOrder>("FinalizeOrder");
}

OrderRunResult OrderTreeRunner::run(
  const OrderRequest & request,
  const CancelPredicate & cancel_requested,
  const FeedbackCallback & publish_feedback,
  const MachineSelector & select_machine)
{
  const auto context = std::make_shared<OrderContext>(
    node_, task_runner_, request, cancel_requested, publish_feedback, select_machine);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("order_context", context);

  BT::Tree tree;
  try {
    tree = factory_.createTreeFromFile(tree_path_, blackboard);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot construct order tree: %s", error.what());
    OrderRunResult result;
    result.error_code = PhysicalStep::Result::INVALID_REQUEST;
    result.message = std::string("cannot construct order tree: ") + error.what();
    return result;
  }

  try {
    BT::NodeStatus status = BT::NodeStatus::IDLE;
    do {
      if (cancel_requested()) {
        tree.haltTree();
        return context->result(false, true);
      }
      status = tree.tickOnce();
      // ExecuteNextLoad/Unload tick their nested physical task tree
      // synchronously. Cancellation can therefore arrive during tickOnce();
      // re-check it before interpreting the returned FAILURE as an error.
      if (cancel_requested()) {
        tree.haltTree();
        return context->result(false, true);
      }
    } while (status == BT::NodeStatus::RUNNING);
    return context->result(status == BT::NodeStatus::SUCCESS, false);
  } catch (const std::exception & error) {
    tree.haltTree();
    RCLCPP_ERROR(node_->get_logger(), "Order tree exception: %s", error.what());
    OrderRunResult result;
    result.completed = context->result(false, false).completed;
    result.error_code = PhysicalStep::Result::EXECUTION_FAILED;
    result.message = std::string("order behavior tree exception: ") + error.what();
    return result;
  }
}

}  // namespace factory_task_bt
