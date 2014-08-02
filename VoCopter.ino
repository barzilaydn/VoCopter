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

#include <TimerOne.h>
#include <PID_v2.h>
#include <PIDMaster.h>

//PID variables
volatile double Pitch_S, Pitch_I, Pitch_O;
volatile double Roll_S, Roll_I, Roll_O;
volatile double Yaw_S, Yaw_I, Yaw_O;

//PID for each rotation type.
PID PitchPID(&Pitch_I, &Pitch_O, &Pitch_S, 0.0, 0.0, 0.0, false, 10);
PID RollPID (&Roll_I , &Roll_O , &Roll_S , 0.0, 0.0, 0.0, false, 10);
PID YawPID  (&Yaw_I  , &Yaw_O  , &Yaw_S  , 0.0, 0.0, 0.0, false, 10);
PIDMaster masterPID (&PitchPID, &RollPID, &YawPID);

void setup(void) {
    PitchPID.SetOutputLimits(0,100);
    RollPID.SetOutputLimits(0,100);
    YawPID.SetOutputLimits(0,100);
    masterPID.Start();
}

void loop(void) {
    //TODO: implement a state-machine.
    //TODO: remember using the low power library.
    //TODO: implement idle state wake-up on RX receive via pin (!! pin 11 !!). see DeepSleep_Simple.ino example.
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




