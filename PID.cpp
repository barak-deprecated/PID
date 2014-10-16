/*
  PID.h - Library for Arduino PID control
  Created by Barak Alon, October 5, 2014.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Control.h"

PID::PID(long *error, long *output, float kp, float ki, float kd)
{
  _error = error;
  _output = output;
  _kp = kp;
  _ki = ki;
  _kd = kd;
  
  _iCutoff = false;  
  _antiWindupThreshold = 0;
  _derivativeThreshold = 0;
  
  lastTime = micros();
  errorSum = 0;
  lastError = 0;
}

void PID::compute()
{
  time = micros();
  
  // Proportional control
  // -------------------
  pComponent = long(_kp * *_error);
  
  // Integral control
  // ---------------
  // If the user turn integral cutoff on, immediately zero out the integral
  // component when the set point is reached. helping prevent overshoot.
  if(_iCutoff == true && *_error*lastError < 0)
    errorSum = 0;
  // If the user set a anti windup threshold, only include integral control
  // when the error is within a certain range. This prevents windup overshoot
  // from integral buildup. 
  else if(_antiWindupThreshold != 0 && abs(*_error) >= _antiWindupThreshold)
    errorSum = 0;
  else
    errorSum += *_error * (time-lastTime);
  iComponent += long(_ki * errorSum);
  
  // Derivative Control
  // -----------------
  // If the user set a derivative threshold, only include derivative control
  // when the error is within a certain range. This limits the derivative control
  // to only slow the output down when close to the setpoint.
  if(_derivativeThreshold != 0 && abs(*_error) >= _derivativeThreshold)
    errorDerivative = 0;
  else
    errorDerivative = (*_error-lastError) / (time-lastTime); 
  dComponent = long(_kd * errorDerivative);
  
  // Update the output variable with the new PID control signal
  *_output = pComponent + iComponent + dComponent;
  
  lastTime = time;
  lastError = *_error;
}

void PID::setIntegralCutoff(boolean iCutoff)
{
  _iCutoff = iCutoff;
}

void PID::setAntiWindupThreshold(long antiWindupThreshold)
{
  _antiWindupThreshold = antiWindupThreshold;
}

void PID::setDerivativeThreshold(long derivativeThreshold)
{
  _derivativeThreshold = derivativeThreshold;
}