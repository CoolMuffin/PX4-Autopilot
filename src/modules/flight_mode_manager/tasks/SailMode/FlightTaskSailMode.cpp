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
#include <uORB/topics/actuator_motors.h>
#include <uORB/uORB.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>

//the flight task directly changes the actuator values, according to the RC input.

bool FlightTaskSailMode::updateInitialize()
{
	bool ret = FlightTask::updateInitialize();

  //setting the timestamp
  _actuators.timestamp = hrt_absolute_time();

  //initializing the sticks
	_sticks.checkAndUpdateStickInputs();
  _constraints.want_takeoff = true;
 return ret;
}

// activating the SailMode flight task
bool FlightTaskSailMode::activate(const trajectory_setpoint_s& last_setpoint)
{
  bool ret = FlightTask::activate(last_setpoint);
  //setting the position setpoint to the current position
  _position_setpoint(0) = _position(0);
  _position_setpoint(1) = _position(1);
  _position_setpoint(2) = _position(2);

  //advertising the actuator controls
  _actuators_0_pub = orb_advertise(ORB_ID(actuator_motors), &_actuators);

  //print message to see if the task is activated
  PX4_INFO("SailMode activated");
  return ret;
}


//Updating the Flight Task
bool FlightTaskSailMode::update()
{
  //getting the stick inputs
  x = _sticks.getRollExpo();
  y = _sticks.getPitch();

  //PX4_INFO("I get the values ret: %f %f", (double)x, (double)y); //print stick inputs to see if they are working

  //getting the motor values from the subscription
  _actuator_motors_sub.update(&_updated_motors);

  //PX4_INFO("Motor 1 value: %f %f %f %f", (double)_updated_motors.control[0], (double)_updated_motors.control[1], (double)_updated_motors.control[2], (double)_updated_motors.control[3]);


  //setting the motor values according to the stick inputs
  if (fabs(x)>0.09f)
  {
    motorRollMapping();
  }
  else if (fabs(y)>0.09f)
  {
    motorPitchMapping();
  }
  else
  {
    motorkill();
  }

  //saturating the motor values
  saturate();

  //print outputs to see if they are working
  PX4_INFO("Stick:%f motors: %f %f %f %f", (double)x, (double)_actuators.control[0], (double)_actuators.control[1], (double)_actuators.control[2], (double)_actuators.control[3]);

  // Publish the actuator controls
  orb_publish(ORB_ID(actuator_motors), _actuators_0_pub, &_actuators);

  //updating the setpoints
  dummySetpoints();

  return true;
}



void FlightTaskSailMode::dummySetpoints()
{
  //updating current position and velocity as dummy setpoints
  _position_setpoint(0) = _position(0);
  _position_setpoint(1) = _position(1);
  _position_setpoint(2) = _position(2);
  _velocity_setpoint(0) = _velocity(0);
  _velocity_setpoint(1) = _velocity(1);
  _velocity_setpoint(2) = _velocity(2);
}


void FlightTaskSailMode::saturate()
{
  //add saturation to all the motor values. if its above 1, set it to 1. if its below 0, set it to 0.
    for ( int i = 0; i < 4; i++)
    {
      if (_actuators.control[i] > 1.0f)
      {
        _actuators.control[i] = 1.0f;
      }
      else if (_actuators.control[i] < 0.0f)
      {
        _actuators.control[i] = 0.0f;
      }
    }
}
void FlightTaskSailMode::motorPitchMapping()
{
  //update the motor values with gain control
     if (y < 0.1f)
    {
      //update the motor values with gain control
      _actuators.control[0] = fabs(y) * k_p; // Motor 1 on
      _actuators.control[2] = fabs(y) * k_p; // Motor 3 on
      _actuators.control[1] = 0 ; // Motor 2 off
      _actuators.control[3] = 0 ; // Motor 4 off

    }

    else if (y > 0.1f)
    {
      _actuators.control[1] = y * k_p; // Motor 2 on
      _actuators.control[3] = y * k_p; // Motor 4 on
      _actuators.control[2] = 0; // Motor 3 off
      _actuators.control[0] = 0; // Motor 1 off

    }
}

void FlightTaskSailMode::motorRollMapping()
{
  //update the motor values with gain control
   if (x < 0.1f)
    {
      _actuators.control[0] = fabs(x) * k_p; // Motor 1 on
      _actuators.control[3] = fabs(x) * k_p; // Motor 4 on
      _actuators.control[1] = 0 ; // Motor 2 off
      _actuators.control[2] = 0 ; // Motor 3 off

    }

    else if (x > 0.1f)
    {
      _actuators.control[1] = x * k_p; // Motor 2 on
      _actuators.control[2] = x * k_p; // Motor 3 on
      _actuators.control[0] = 0; // Motor 1 off
      _actuators.control[3] = 0; // Motor 4 off

    }
}

void FlightTaskSailMode::motorkill()
{
  //kill the motors
  _actuators.control[0] = 0.0f;
  _actuators.control[1] = 0.0f;
  _actuators.control[2] = 0.0f;
  _actuators.control[3] = 0.0f;
}
