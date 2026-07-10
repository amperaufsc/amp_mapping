#pragma once

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// CLIENTS
#include "lifecycle_msgs/msg/transition_event.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "smacc2/client_behaviors/cb_wait_topic_message.hpp"
#include "state_machine_as/orthogonals/or_notsystemchecks.hpp"
#include "state_machine_as/orthogonals/or_not_ebs.hpp"

namespace state
{
// SMACC2 classes
using smacc2::Transition;
using smacc2::EvStateRequestFinish;
using smacc2::default_transition_tags::SUCCESS;
using smacc2::client_behaviors::CbWaitTopicMessage;

//using state::OrNOTSystemChecks;
using state::OrNOTEbs;

// STATE DECLARATION
struct AsEmergency : smacc2::SmaccState<AsEmergency, State>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef boost::mpl::list<
    Transition<
      smacc2::EvCbSuccess<
        smacc2::client_behaviors::CbWaitTopicMessage<lifecycle_msgs::msg::TransitionEvent>,
        OrNOTEbs>,
      AsOff,
      SUCCESS
    >
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrNOTEbs, CbWaitTopicMessage<lifecycle_msgs::msg::TransitionEvent>>(
      "NOT_Ebs");
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_WARN(getLogger(), "ON AS_EMERGENCY");
    if (!state_pub_) {
      state_pub_ = getNode()->create_publisher<std_msgs::msg::String>("/AMP/as_status_indicator", 10);
    }

    std_msgs::msg::String msg;
    msg.data = "as_emergency";
    state_pub_->publish(msg);

    if (!as_emergency_pub_) {
      as_emergency_pub_ = getNode()->create_publisher<std_msgs::msg::Bool>("/AMP/as_emergency", 10);
    }

    std_msgs::msg::Bool emergency_msg;
    emergency_msg.data = true;
    as_emergency_pub_->publish(emergency_msg);
  }
  void onExit()
  {
    std::string output_message_note;
    getGlobalSMData("output_message_note", output_message_note);
    RCLCPP_INFO(getLogger(), (output_message_note + " On Exit!").c_str());
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr as_emergency_pub_;
};
}
