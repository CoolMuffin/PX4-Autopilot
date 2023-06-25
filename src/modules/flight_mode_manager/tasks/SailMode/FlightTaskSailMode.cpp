/***************************************************************************************************************************
 *
 * want to control a drone that has landed on water and has its motors turned 90 degrees manually to act like a hovercraft. The control logic you want to implement is:
 *   When the stickRoll is -1 (left), the right motor activates at a certain speed.
 * When the stickRoll is +1 (right), the left motor activates at a certain speed.
 * This will allow the drone to navigate on water using different motors than the ones used when flying.
 *
 *
 * ********************************************************/



#include "FlightTaskSailMode.hpp"

//the flight task directly changes the actuator values, according to the RC input.

// initializing the rc stick inputs
bool FlightTaskSailMode::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();
  //_actuator_setpoints.fill(NAN);
	_sticks.checkAndUpdateStickInputs();
 return ret;
}

// activating the SailMode flight task

bool FlightTaskSailMode::activate(const trajectory_setpoint_s& last_setpoint)
{
  bool ret = FlightTask::activate(last_setpoint);

  _velocity_setpoint(2) = -1.0f;
  return ret;
}

//Updating the Flight Task



bool FlightTaskSailMode::update()
{

  float x = _sticks.getRollExpo();
  float y = _sticks.getPitch();
        // Get processed stick inputs
        //matrix::Vector4f stick_positions = _sticks.getPositionExpo();

        // Use stick_positions to set actuator setpoints
        if (y > 0.0f) { // stick pushed forward
            _actuator_setpoints(2) = 0.2f; // actuator 3
            _actuator_setpoints(4) = 0.2f; // actuator 5
        } else if (y < 0.0f) { // stick pushed backward
            _actuator_setpoints(3) = 0.2f; // actuator 4
            _actuator_setpoints(5) = 0.2f; // actuator 6
        } else if (x > 0.0f) { // stick pushed right
            _actuator_setpoints(4) = 0.2f; // actuator 5
            _actuator_setpoints(5) = 0.2f; // actuator 6
        } else if (x < 0.0f) { // stick pushed left
            _actuator_setpoints(2) = 0.2f; // actuator 3
            _actuator_setpoints(3) = 0.2f; // actuator 4
        }

        // Set actuator setpoints
        //_control_allocation.setActuatorSetpoint(_actuator_setpoints);


    return true;
}
