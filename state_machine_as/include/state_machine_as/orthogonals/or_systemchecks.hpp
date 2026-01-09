#pragma once

#include <std_msgs/msg/bool.hpp>
#include <smacc2/smacc.hpp>

namespace state
{
class OrSystemChecks : public smacc2::Orthogonal<OrSystemChecks>
{
public:
  void onInitialize() override
  {
    // Não criar client explicitamente aqui.
    // O client será criado implicitamente via configure_orthogonal no estado.
  }
};
}  // namespace carlos
