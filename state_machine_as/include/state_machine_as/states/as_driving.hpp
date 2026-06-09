#pragma once

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// CLIENTS
#include <std_msgs/msg/bool.hpp>
#include "smacc2/client_behaviors/cb_wait_topic_message.hpp"
#include "state_machine_as/orthogonals/or_notsystemchecks.hpp"
#include "state_machine_as/orthogonals/or_r2d.hpp"
#include "state_machine_as/orthogonals/or_ebs_missioncomplete.hpp"
#include "state_machine_as/orthogonals/or_ebs_notmissioncomplete.hpp"
#include "state_machine_as/orthogonals/or_not_ebs.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

namespace state
{
// SMACC2 classes
using smacc2::Transition;
using smacc2::EvStateRequestFinish;
using smacc2::default_transition_tags::SUCCESS;
using smacc2::client_behaviors::CbWaitTopicMessage;

//using state::OrNOTSystemChecks;
using state::OrEbsMissionFinished;
using state::OrEbsNOTMissionFinished;
//using state::OrR2D;
using CbWaitEmergency = CbWaitTopicMessage<lifecycle_msgs::msg::TransitionEvent>;
using CbWaitFinished = CbWaitTopicMessage<lifecycle_msgs::msg::TransitionEvent>;

// STATE DECLARATION
struct AsDriving : smacc2::SmaccState<AsDriving, State>
{
  using SmaccState::SmaccState;

  // TRANSITION TABLE
  typedef boost::mpl::list<
    Transition<
      smacc2::EvCbSuccess<CbWaitEmergency, OrEbsNOTMissionFinished>,
      AsEmergency,
      SUCCESS
    >,
    Transition<
      smacc2::EvCbSuccess<CbWaitFinished, OrEbsMissionFinished>,
      AsFinished,
      SUCCESS
    >
  > reactions;

  // STATE FUNCTIONS
  static void staticConfigure()
  {
    configure_orthogonal<OrEbsNOTMissionFinished, CbWaitEmergency>("EbsNOTMissionFinished");

    configure_orthogonal<OrEbsMissionFinished, CbWaitFinished>("EbsMissionFinished");
  }

  void runtimeConfigure() {}

  void onEntry()
  {
    RCLCPP_WARN(getLogger(), "ON AS_DRIVING");

    if (!as_driving_pub_) {
      as_driving_pub_ = getNode()->create_publisher<std_msgs::msg::Bool>("/AMP/as_driving", 10);
    }

    std_msgs::msg::Bool msg;
    msg.data = true;
    as_driving_pub_->publish(msg);
  }

  void onExit()
  {
    std::string output_message_note;
    getGlobalSMData("output_message_note", output_message_note);
    RCLCPP_INFO(getLogger(), (output_message_note + " On Exit!").c_str());
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr as_driving_pub_;
};
}
