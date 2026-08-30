#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "factory_task_bt/task_tree_runner.hpp"

namespace factory_task_bt
{

struct OrderRequest
{
  std::string order_id;
  unsigned quantity{0};
  std::vector<std::string> allowed_machine_ids;
  bool auto_recharge{true};
  unsigned available_raw_parts{0};
  unsigned available_finished_slots{0};
};

struct OrderProgress
{
  std::string phase{"idle"};
  std::string machine_id;
  unsigned completed{0};
  unsigned total{0};
  std::string detail;
};

struct OrderRunResult
{
  bool success{false};
  bool canceled{false};
  unsigned completed{0};
  std::uint16_t error_code{PhysicalStep::Result::EXECUTION_FAILED};
  std::string message;
};

class OrderTreeRunner
{
public:
  using CancelPredicate = std::function<bool()>;
  using FeedbackCallback = std::function<void(const OrderProgress &)>;
  using MachineSelector = std::function<std::optional<std::string>(
      const std::vector<std::string> &, const std::set<std::string> &,
      const CancelPredicate &, bool)>;

  OrderTreeRunner(rclcpp::Node * node, TaskTreeRunner * task_runner);

  OrderRunResult run(
    const OrderRequest & request,
    const CancelPredicate & cancel_requested,
    const FeedbackCallback & publish_feedback,
    const MachineSelector & select_machine);

private:
  void registerOrderNodes();

  rclcpp::Node * node_;
  TaskTreeRunner * task_runner_;
  BT::BehaviorTreeFactory factory_;
  std::string tree_path_;
};

}  // namespace factory_task_bt
