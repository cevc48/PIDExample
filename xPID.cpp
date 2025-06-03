
#include "xPID.h"


void xPID::SetTerms(float Kp, float Ki, float Kd) {
		_Kp = Kp;
		_Ki = Ki;
		_Kd = Kd;
	}

	void xPID::SetWindup(const int MAX, const int MIN) {
		_MAX = MAX;
		_MIN = MIN;
	}

	void xPID::SetSampleTime(unsigned int sampleTime) {

		_sampleTime = sampleTime;

	}
	
	
	void xPID::CalcPID(double* const input, const double setPoint, double* const PID)
	{
	
	   _input = input;
	   _setPoint = setPoint;
	   _PID = PID;

		unsigned long lastTime = 0.0;
		double error, lastError;
		double epsilon = 0.05;// error treshold variable
		double I = 0.0;//integral variable
		double D;// derivative variable

		error = abs(_setPoint - *_input);
		 

		if ((millis() - lastTime) >= _sampleTime) { // run PID if time is @ sample time
			if (abs(error) > epsilon) {// if the error is too small stop integration
				I += error; // add current error to the running total
			}// end if epsilon

			D = (error - lastError); // calculate slope of input

			*_PID = _Kp * error + _Ki * I + _Kd * D; // output variable

			if (*_PID > _MAX) *PID = _MAX;//windup guard output
			if (*_PID < _MIN) *PID = _MIN;

			lastError = error; // save for next time
			lastTime = millis();
		}// end if (timechange > sampletime)
	
	}

