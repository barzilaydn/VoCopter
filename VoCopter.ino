// VoCopter
// 2014-07-20 by Dan Barzilay
//
// Change log:
//     2014-07-20 - initial release

/* ============================================
VoCopter code is placed under the MIT license
Copyright (c) 2014 Dan Barzilay

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sub-license, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#ifdef CORE_TEENSY
	#include <LowPower_Teensy3.h>
#endif

const int IO_PIN_AMOUNT = 34;

//define PID values
double pitch_setpoint, pitch_input, pitch_output, pitch_kpid{/* kp,ki,kd*/};
double roll_setpoint, roll_input, roll_output, roll_kpid{/* kp,ki,kd*/};
double yaw_setpoint, yaw_input, yaw_output, yaw_kpid{/* kp,ki,kd*/};

//specify the links and initial tuning parameters
PID pitch_PID(&pitch_input, &pitch_output, &pitch_setpoint,pitch_kpid[0], pitch_kpid[1],pitch_kpid[2], DIRECT);
PID  roll_PID(&roll_input, &roll_output, &roll_output,roll_kpid[0], roll_kpid[1],roll_kpid[2], DIRECT);
PID  yaw_PID(&yaw_input, &yaw_output, &yaw_output,yaw_kpid[0], yaw_kpid[1],yaw_kpid[2], DIRECT);

void setup(void) {
	pitch_PID.SetOutputLimits(0,100);
	roll_PID.SetOutputLimits(0,100);
	yaw_PID.SetOutputLimits(0,100);
}

void loop(void) {
	//TODO: implement a state-machine.
	//TODO: remember using the low power library.
	//TODO: implement idle state wake-up on RX receive via pin (!! pin 11 !!). see DeepSleep_Simple.ino example.
	
	
	/*
	 * Set thrust
	 */
	if (thrust > thrust_setpoint)      //if thrust is bigger than thrust_setpoint
	{
		thrust--; //decrease thrust
	}
	else if (thrust < thrust_setpoint) //if thrust is smaller than thrust_setpoint
	{
		thrust++; //increase thrust
	}
	else;                              //thrust matches thrust_setpoint
	
	//set engines values with thrust
	FR = thrust; //front right
	FL = thrust; //front left
	BR = thrust; //back right
	BL = thrust; //back left
	
	/*
	 * Pitch
	 */
	
	//initalize pitch_PID Setpont and Process Variable
	pitch_input =  map(GET_PITCH,-90,90,0,100);
	pitch_setpoint = GET_PITCH_SETPOINT;

	//turn the pitch_PID on
	pitch_PID.SetMode(AUTOMATIC);
	
	//compute PID
	pitch_PID.Compute();
	
	if (pitch_setpoint > pitch_input)
	{
		//pitch axis is on the back
		//reduce thrust on the front
		FL -= output;
		FR -= output;
	}
	else
	{
		//pitch axis is on the front
		//reduce thrust on the back
		BL -= output;
		BR -= output;
	}
	
	/*
	 * Roll
	 */
	
	//initalize roll_PID Setpont and Process Variable
	roll_input =  map(GET_ROLL,-90,90,0,100);
	roll_setpoint = GET_ROLL_SETPOINT;

	//turn the roll_PID on
	roll_PID.SetMode(AUTOMATIC);
	
	//compute PID
	roll_PID.Compute();
	
	if (roll_setpoint > roll_input)
	{
		//roll axis is on the right
		//reduce thrust on the left
		FL -= output;
		BL -= output;
	}
	else
	{
		//roll axis is on the left
		//reduce thrust on the right
		FR -= output;
		BR -= output;
	}
	
	/*
	 * Yaw
	 */
	
	//initalize yaw_PID Setpont and Process Variable
	yaw_input =  map(GET_YAW,-90,90,0,100);
	yaw_setpoint = GET_YAW_SETPOINT;

	//turn the yaw_PID on
	yaw_PID.SetMode(AUTOMATIC);
	
	//compute PID
	yaw_PID.Compute();
	
	if (yaw_setpoint > yaw_input)
	{
		//yaw to the right
		FR -= output;
		BL -= output;
	}
	else
	{
		//yaw to the left
		FL -= output;
		BR -= output;
	}
}

/**
 * Read the battery level from an analog pin
 * @param pin The number of the analog pin to read from.
 * @return Returns the battery level measured on the pin.
 */
int BatLevel(int pin) {
	return mVtoL(1195 * 4096 /analogRead(pin));
}

/**
 * Convert millivolts to battery level in %. (Calibrated for LiPo 3.7v 300mah)
 * @param mV The voltage in millivolts.
 * @return Returns battery level in %.
 */
int mVtoL(int mV) {
	double l = -191.04 * pow(1000 * mV, 3) + 2132.6 * pow(1000 * mV, 2) - 7778.9 * 1000 * mV + 9309;
	if (l > 100)
		l = 100;
	else if (l < 0)
		l = 0;
	return (int)l;
}

/**
 * Setup for the Low Voltage Warning interrupt. Available for Teensy only.
 */


 