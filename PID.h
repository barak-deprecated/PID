/*
  PID.h - Library for Arduino PID control
  Created by Barak Alon, October 5, 2014.
  Released into the public domain.
*/

#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID
{
  public:
    PID(long *error, long *output, float kp, float ki, float kd);
    void compute();
    void setIntegralCutoff(boolean iCutoff);
    void setAntiWindupThreshold(long antiWindupThreshold);
    void setDerivativeThreshold(long derivativeThreshold);
    
    // ! Add "get" functions...
    
  private: 
	long *_error; 
	long *_output;
	float _kp, _ki, _kd;
	long _antiWindupThreshold, _derivativeThreshold;
	boolean _iCutoff;

	long errorSum, errorDerivative, lastError, pComponent, iComponent, dComponent;
	long time, lastTime;
};

#endif
