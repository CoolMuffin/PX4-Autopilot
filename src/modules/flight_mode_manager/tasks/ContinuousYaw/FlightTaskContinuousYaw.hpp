#pragma once

#include "FlightTask.hpp"

class FlightTaskContinuousYaw : public FlightTask
{
public:
  FlightTaskContinuousYaw() = default;
  virtual ~FlightTaskContinuousYaw() = default;

 bool update() override;
 bool activate(const trajectory_setpoint_s& last_setpoint) override;

private:
  float _origin_z{0.f};
};
