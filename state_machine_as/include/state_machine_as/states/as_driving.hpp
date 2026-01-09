#pragma once

#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"

// CLIENTS
#include "ros_timer_client/cl_ros_timer.hpp"
#include "ros_timer_client/client_behaviors/cb_timer_countdown_loop.hpp"
#include "ros_timer_client/client_behaviors/cb_timer_countdown_once.hpp"
#include <std_msgs/msg/bool.hpp>
#include "smacc2/client_behaviors/cb_wait_topic_message.hpp"
#include "state_machine_as/orthogonals/or_notsystemchecks.hpp"
#include "state_machine_as/orthogonals/or_r2d.hpp"
#include "state_machine_as/orthogonals/or_ebs_missioncomplete.hpp"
#include "state_machine_as/orthogonals/or_not_ebs.hpp" // so declarando aqui pra funcionar (tinha q ser "criado" antes)
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
  }

  void onExit()
  {
    std::string output_message_note;
    getGlobalSMData("output_message_note", output_message_note);
    RCLCPP_INFO(getLogger(), (output_message_note + " On Exit!").c_str());
  }
};
}
