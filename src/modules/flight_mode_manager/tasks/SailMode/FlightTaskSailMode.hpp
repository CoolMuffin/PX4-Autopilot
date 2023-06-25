#pragma once

#include <matrix/matrix/math.hpp>
#include "FlightTask.hpp"
#include "flight_mode_manager/tasks/Utility/Sticks.hpp"
#include "control_allocator/ControlAllocation/ControlAllocation.hpp"
#include "control_allocator/ActuatorEffectiveness/ActuatorEffectiveness.hpp"


//   class ContAll : public ControlAllocation {
//   public:
//       ContAll() = default;
//       virtual ~ContAll() = default;
// 	static constexpr uint8_t NUM_ACTUATORS = ActuatorEffectiveness::NUM_ACTUATORS;
// 	static constexpr uint8_t NUM_AXES = ActuatorEffectiveness::NUM_AXES;

// 	typedef matrix::Vector<float, NUM_ACTUATORS> ActuatorVector;

//  };



class FlightTaskSailMode : public FlightTask
{
public:
    //FlightTaskSailMode(ControlAllocation& control_allocation) : _control_allocation(control_allocation) {} // Pass ControlAllocation object in constructor
    FlightTaskSailMode() = default;
    virtual ~FlightTaskSailMode() = default;

    bool update() override;
    bool updateInitialize() override;
    bool activate(const trajectory_setpoint_s& last_setpoint) override;

private:
    //ControlAllocation& _control_allocation;
    float _origin_z{0.f};
    matrix::Vector<float, 16> _actuator_setpoints{};
    Sticks _sticks{this}; // Declare a Sticks object
};
