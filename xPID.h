/*************************************************************************
A PID controller is a feedback control loop that calculates an error
value as the difference between a setpoint and a measured variable in
a process.The PID controller minimizes the error by adjusting
the input.
The initialization constructor allows to declare an object globally before
the setup() function and to insert the PID settings.
				Version 1.0		Author: Carlos Carvalho
*************************************************************************/

#pragma once

#ifndef xPID_H_INCLUDED
#define xPID_H_INCLUDED

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif



class xPID
{

public:

xPID() { _sampleTime = 1000; };// 1s default constructor

void SetTerms(float Kp, float Ki, float Kd);
void SetWindup(const int MAX, const int MIN);
void SetSampleTime(unsigned int sampleTime);
double GetKp()const { return _Kp; }// get functions
double GetKi()const { return _Ki; }
double GetKd()const { return _Kd; }
int GetSampleTime()const { return _sampleTime; }
int GetMAX()const { return _MAX; }
int GetMIN()const { return _MIN; }
	
void CalcPID(double* const input, const double setPoint, double* const PID);
void KillIntegralError() { _Ki = 0; };


private:
	
	double* _input;
	double _setPoint;
	double _Kp;
	double _Ki;
	double _Kd;
	unsigned int _sampleTime;
	double* _PID;
	int _MIN;
	int _MAX;
};

#endif