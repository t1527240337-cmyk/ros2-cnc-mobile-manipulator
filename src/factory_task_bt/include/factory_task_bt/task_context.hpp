#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "factory_interfaces/action/execute_physical_step.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace factory_task_bt
{

using PhysicalStep = factory_interfaces::action::ExecutePhysicalStep;
using PhysicalStepClient = rclcpp_action::Client<PhysicalStep>;

struct TaskProgress
{
  std::string phase{"idle"};
  std::string detail;
  std::string first_error;
  std::uint16_t error_code{PhysicalStep::Result::OK};
  bool retryable{false};
};

class TaskContext
{
public:
  TaskContext(rclcpp::Node * node, PhysicalStepClient::SharedPtr client)
  : node_(node), client_(std::move(client))
  {
  }

  rclcpp::Node * node() const {return node_;}
  const PhysicalStepClient::SharedPtr & client() const {return client_;}

  void started(const std::string & phase)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    progress_.phase = phase;
    progress_.detail = phase + " started";
  }

  void feedback(const std::string & phase, const std::string & detail)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    progress_.phase = phase;
    progress_.detail = detail;
  }

  void failed(
    const std::string & phase, const std::string & detail,
    std::uint16_t error_code, bool retryable)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    progress_.phase = phase;
    progress_.detail = detail;
    if (progress_.first_error.empty()) {
      progress_.first_error = detail;
      progress_.error_code = error_code;
      progress_.retryable = retryable;
    }
  }

  TaskProgress snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return progress_;
  }

private:
  rclcpp::Node * node_;
  PhysicalStepClient::SharedPtr client_;
  mutable std::mutex mutex_;
  TaskProgress progress_;
};

}  // namespace factory_task_bt
