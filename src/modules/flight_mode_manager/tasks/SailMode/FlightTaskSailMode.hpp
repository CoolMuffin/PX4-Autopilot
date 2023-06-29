#pragma once

#include <matrix/matrix/math.hpp>
#include "FlightTask.hpp"
#include "flight_mode_manager/tasks/Utility/Sticks.hpp"
#include "control_allocator/ControlAllocation/ControlAllocation.hpp"
#include "control_allocator/ActuatorEffectiveness/ActuatorEffectiveness.hpp"
#include <uORB/topics/actuator_motors.h>




class FlightTaskSailMode : public FlightTask
{
public:
    FlightTaskSailMode() = default;
    virtual ~FlightTaskSailMode() = default;
    virtual void dummySetpoints();
    virtual void saturate();
    virtual void motorRollMapping();
    virtual void motorPitchMapping();
    virtual void motorkill();
    bool update() override;
    bool updateInitialize() override;
    bool activate(const trajectory_setpoint_s& last_setpoint) override;

private:
    orb_advert_t _actuators_0_pub = nullptr; // Add this line to declare the uORB publisher
    struct actuator_motors_s _actuators {}; // Actuator controls
    float k_p = 0.5; //proportional gain
    float x = 0.0; //stick roll input
    float y = 0.0; //stick pitch input
    uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)}; // Declare the subscription object
    struct actuator_motors_s _updated_motors; // Declare the structure to hold the data
    Sticks _sticks{this}; // Declare a Sticks object
};
