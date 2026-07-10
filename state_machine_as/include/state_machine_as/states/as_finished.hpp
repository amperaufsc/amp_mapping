#pragma once

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// CLIENTS
#include "lifecycle_msgs/msg/transition_event.hpp"
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
struct AsFinished : smacc2::SmaccState<AsFinished, State>
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
    RCLCPP_WARN(getLogger(), "ON AS_FINISHED");  }

  void onExit()
  {
    std::string output_message_note;
    getGlobalSMData("output_message_note", output_message_note);
    RCLCPP_INFO(getLogger(), (output_message_note + " On Exit!").c_str());
  }
};
}
