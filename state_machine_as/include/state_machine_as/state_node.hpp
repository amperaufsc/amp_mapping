// Copyright 2021 MyName/MyCompany Inc.
// Copyright 2021 RobosoftAI Inc. (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>

#include "smacc2/smacc.hpp"

// ORTHOGONALS
#include "state_machine_as/orthogonals/or_timer.hpp"
#include "state_machine_as/orthogonals/or_notsystemchecks.hpp"
#include "state_machine_as/orthogonals/or_r2d.hpp"
#include "state_machine_as/orthogonals/or_systemchecks.hpp"
#include "state_machine_as/orthogonals/or_ebs_missioncomplete.hpp"
#include "state_machine_as/orthogonals/or_ebs_notmissioncomplete.hpp"
#include "state_machine_as/orthogonals/or_notsystemchecks.hpp"
#include "state_machine_as/orthogonals/or_not_ebs.hpp"

namespace state
{
// SMACC2 clases
using state::OrTimer;  // This is example variable - feel free to delete it.

//STATES
struct AsOff;
struct AsReady;
struct AsDriving;
struct AsEmergency;
struct AsFinished;
//--------------------------------------------------------------------
//STATE_MACHINE
struct State
: public smacc2::SmaccStateMachineBase<State, AsOff>
{
  using SmaccStateMachineBase::SmaccStateMachineBase;

  void onInitialize() override
  {
    // START: Example code - change or delete as needed
    this->createOrthogonal<OrNOTSystemChecks>();
    this->createOrthogonal<OrR2D>();
    this->createOrthogonal<OrSystemChecks>();
    this->createOrthogonal<OrEbsMissionFinished>();
    this->createOrthogonal<OrEbsNOTMissionFinished>();
    this->createOrthogonal<OrNOTEbs>();
    // Use Blackboard to store global state-machine data - example - feel free to delete it.
    setGlobalSMData(
      "output_message_note", std::string("{I am very cool smacc2 SM called 'carlos'}"));
    // END: Example code - change or delete as needed
  }
};

}  // namespace carlos

//STATES
#include "states/as_finished.hpp"
#include "states/as_emergency.hpp"
#include "states/as_driving.hpp"
#include "states/as_ready.hpp"
#include "states/as_off.hpp"

// A ORDEM IMPORTA. COLOCA OS ESTADOS QUE SERAO REQUISITADOS POR ULTIMO EM PRIMEIRO
