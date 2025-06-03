const int MAX = 360;// max windup value
const int MIN = 0; // min windup value

class xPID {
public:

	void SetTerms(double Kp, double Ki, double Kd) {
		_Kp = Kp;
		_Ki = Ki;
		_Kd = Kd;
	}

	void SetWindup(const int MAX, const int MIN) {
		_MAX = MAX;
		_MIN = MIN;
	}

	void SetSampleTime(unsigned int sampleTime) {
		_sampleTime = sampleTime;
	}

	double GetKp() { return _Kp; }// get functions
	double GetKi() { return _Ki; }
	double GetKd() { return _Kd; }
	int GetSampleTime() { return _sampleTime; }
	int GetMAX() { return _MAX; }
	int GetMIN() { return _MIN; }

	xPID() { _sampleTime = 1000; };// 1s default constructor

	void CalcPID(double* const input, const double setPoint, double* const PID)
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

xPID myPid;// create an object

double Input = 180.0; // actual input value
double SetPoint = 185.0; // setpoint value
double PID;// PID output value

void setup()
{
	Serial.begin(9600);

	myPid.SetTerms(34.0, 2.5, 0.5);// set coefficients
	myPid.SetWindup(360, 0);// set windup guard values
	myPid.SetSampleTime(1000);// set sample time = 1s
}

void loop()
{
	myPid.CalcPID(&Input, SetPoint, &PID);// calculate PID
	Serial.println(PID);// print PID output
}