// custom flight task that takes input from the rc controller roll and pitch and uses it to directly control the motors.
// it's a mode that will be used with a hybrid drone that can sail on water surface. So when roll is <0, then the rigth motors activate
// to make it turn right for example.


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
    // Check for updated stick inputs
    if (_sticks.checkAndUpdateStickInputs()) {
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
        _control_allocation.setActuatorSetpoint(_actuator_setpoints);
    }

    return true;
}
