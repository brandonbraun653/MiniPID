/**
* Small, easy to use PID implementation with advanced controller capability.<br> 
* Minimal usage:<br>
* setPID(p,i,d); <br>
* ...looping code...{ <br>
* output=getOutput(sensorvalue,target); <br>
* }
* 
* @see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/improving-the-beginners-pid-introduction
*
* FORKED FROM: https://github.com/tekdemo/MiniPID 
*/

#include "MiniPID.h"

//**********************************
//Constructor functions
//**********************************
MiniPID::MiniPID(float fSample, float Kp, float Ki, float Kd) {
	init();
	
	sampleFrequency = fSample;
	sample_dT = 1.0f / sampleFrequency;
	
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	F = 0;
}

void MiniPID::init(){
	Kp=0;
	Ki=0;
	Kd=0;
	F=0;
	
	PTerm = 0.0f;
	ITerm = 0.0f;
	DTerm = 0.0f;

	maxIOutput=0;
	maxError=0;
	errorSum=0;
	maxOutput=0; 
	minOutput=0;
	setpoint=0;
	lastInput=0;
	firstRun=true;
	reversed=false;
	outputRampRate=0;
	lastOutput=0;
	outputFilter=0;
	setpointRange=0;
}

//**********************************
//Configuration functions
//**********************************

/**
 * Configure the Proportional gain parameter. <br>
 * this->responds quicly to changes in setpoint, and provides most of the initial driving force
 * to make corrections. <br>
 * Some systems can be used with only a P gain, and many can be operated with only PI.<br>
 * For position based controllers, this->is the first parameter to tune, with I second. <br>
 * For rate controlled systems, this->is often the second after F.
 *
 * @param p Proportional gain. Affects output according to <b>output+=P*(setpoint-current_value)</b>
 */
void MiniPID::setP(float p){
	Kp=p;
	checkSigns();
}

/**
 * Changes the I parameter <br>
 * this->is used for overcoming disturbances, and ensuring that the controller always gets to the control mode. 
 * Typically tuned second for "Position" based modes, and third for "Rate" or continuous based modes. <br>
 * Affects output through <b>output+=previous_errors*Igain ;previous_errors+=current_error</b>
 * 
 * @see {@link #setMaxIOutput(float) setMaxIOutput} for how to restrict
 *
 * @param i New gain value for the Integral term
 */
void MiniPID::setI(float i){
	if(Ki!=0){
		errorSum=errorSum*Ki/i;
		}
	if(maxIOutput!=0){
		maxError=maxIOutput/i;
	}
	Ki=i;
	checkSigns();
	 /* Implementation note: 
	 * this->Scales the accumulated error to avoid output errors. 
	 * As an example doubling the I term cuts the accumulated error in half, which results in the 
	 * output change due to the I term constant during the transition. 
	 *
	 */
} 

void MiniPID::setD(float d){
	Kd=d;
	checkSigns();
}

/**Configure the FeedForward parameter. <br>
 * this->is excellent for Velocity, rate, and other	continuous control modes where you can 
 * expect a rough output value based solely on the setpoint.<br>
 * Should not be used in "position" based control modes.
 * 
 * @param f Feed forward gain. Affects output according to <b>output+=F*Setpoint</b>;
 */
void MiniPID::setF(float f){
	F=f;
	checkSigns();
}

/** Create a new PID object. 
 * @param p Proportional gain. Large if large difference between setpoint and target. 
 * @param i Integral gain.	Becomes large if setpoint cannot reach target quickly. 
 * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
 */
void MiniPID::setPID(float p, float i, float d){
	Kp=p;Ki=i;Kd=d;
	checkSigns();
}

void MiniPID::setPID(float p, float i, float d,float f){
	Kp=p;Ki=i;Kd=d;F=f;
	checkSigns();
}

/**Set the maximum output value contributed by the I component of the system
 * this->can be used to prevent large windup issues and make tuning simpler
 * @param maximum. Units are the same as the expected output value
 */
void MiniPID::setMaxIOutput(float maximum){
	/* Internally maxError and Izone are similar, but scaled for different purposes. 
	 * The maxError is generated for simplifying math, since calculations against 
	 * the max error are far more common than changing the I term or Izone. 
	 */
	maxIOutput=maximum;
	if(Ki!=0){
		maxError=maxIOutput/Ki;
	}
}

/**Specify a maximum output. If a single parameter is specified, the minimum is 
 * set to (-maximum).
 * @param output 
 */
void MiniPID::setOutputLimits(float output){ setOutputLimits(-output,output);}

/**
 * Specify a maximum output.
 * @param minimum possible output value
 * @param maximum possible output value
 */
void MiniPID::setOutputLimits(float minimum,float maximum){
	if(maximum<minimum)return;
	maxOutput=maximum;
	minOutput=minimum;

	// Ensure the bounds of the I term are within the bounds of the allowable output swing
	if(maxIOutput==0 || maxIOutput>(maximum-minimum) ){
		setMaxIOutput(maximum-minimum);
	}
}

/** Set the operating direction of the PID controller
 * @param reversed Set true to reverse PID output
 */
void MiniPID::setDirection(bool reversed){
	this->reversed=reversed;
}

//**********************************
//Primary operating functions
//**********************************

/**Set the target for the PID calculations
 * @param setpoint
 */
void MiniPID::setSetpoint(float setpoint){
	this->setpoint=setpoint;
} 

/** Calculate the PID value needed to hit the target setpoint. 
* Automatically re-calculates the output at each call. 
* @param actual The monitored value
* @param target The target value
* @return calculated output value for driving the actual to the target 
*/
float MiniPID::getOutput(float input, float setpoint)
{
	float output;

	/* Compute all the working error variables */
	float error = (setpoint - input);
	float dInput = (input - lastInput);
	
	
	
	PTerm = (Kp * error);	
	
	ITerm += (Ki * error);
	ITerm = clamp(ITerm, -1, 1);
	
	DTerm = (Kd * dInput);
	

	//And, finally, we can just add the terms up
	output = PTerm + ITerm + DTerm;
	output = clamp(output, minOutput, maxOutput);
	

	lastOutput=output;
	lastInput = input;
	return output;
}

	
/**
 * Resets the controller. this->erases the I term buildup, and removes D gain on the next loop.
 */
void MiniPID::reset(){
	firstRun=true;
	errorSum=0;
}

/**Set the maximum rate the output can increase per cycle. 
 * @param rate
 */
void MiniPID::setOutputRampRate(float rate){
	outputRampRate=rate;
}



//**************************************
// Helper functions
//**************************************

/**
 * Forces a value into a specific range
 * @param value input value
 * @param min maximum returned value
 * @param max minimum value in range
 * @return Value if it's within provided range, min or max otherwise 
 */
float MiniPID::clamp(float value, float min, float max){
	if(value > max){ return max;}
	if(value < min){ return min;}
	return value;
}

/**
 * Test if the value is within the min and max, inclusive
 * @param value to test
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return
 */
bool MiniPID::bounded(float value, float min, float max){
		return (min<value) && (value<max);
}

/**
 * To operate correctly, all PID parameters require the same sign,
 * with that sign depending on the {@literal}reversed value
 */
void MiniPID::checkSigns(){
	if(reversed){	//all values should be below zero
		if(Kp>0) Kp*=-1.0f;
		if(Ki>0) Ki*=-1.0f;
		if(Kd>0) Kd*=-1.0f;
		if(F>0) F*=-1.0f;
	}
	else{	//all values should be above zero
		if(Kp<0) Kp*=-1.0f;
		if(Ki<0) Ki*=-1.0f;
		if(Kd<0) Kd*=-1.0f;
		if(F<0) F*=-1.0f;
	}
}
