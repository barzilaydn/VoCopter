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
#if ARDUINO < 100
    #include "WProgram.h"
#else
    #include "Arduino.h"
#endif

#include <EEPROM.h>

//#ifdef CORE_TEENSY
//    #include <LowPower_Teensy3.h>
//#else
//    #include <LowPower.h>
//#endif

#include "Quad.h"

#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include <i2c_t3.h>
//#include <Wire.h>

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#undef USE_SIMULATION

/**
* States
*/
#define SLEEP     0
#define FLY       1
#define CALIBRATE 2
#define TUNE      3
#define MOVE      4
#define IDLE      5
/*---------*/


Quad VoCopter(20, 0.5, 1, 10, X_CONF, 20); // (OutputStep, Noise, LookBackSec, SampleTime(ms), Configuration, MPU6050 Interrupt Pin)
int STATE = SLEEP;
int PARAMS[10];

void setup(void)
{
    Serial3.begin(115200);
    
    int motorPins[] = { 5, 6, 9, 10 }; // {FrontLeft, FrontRight, BackRight, BackLeft}
    VoCopter.Setup(motorPins); //(Motors)
}

void loop(void)
{
    //TODO: implement landing via altitude sensor.    
    
    //TODO: remember using the low power library.
    //TODO: implement idle state wake-up on RX receive via pin (!! pin 11 !!). see DeepSleep_Simple.ino example.
    
    //Here we should receive user data.
    //
    //--
    
    // The main state machine
    switch(STATE)
    {
        case FLY:
            //TODO: update PID to run as fast as possible? or at least have it with correct time samples..
            VoCopter.Fly();
            break;
            
        case TUNE:
            if(VoCopter.TunePID(PARAMS[0]))
            {
                //TODO: Output that we finished tuning..
                STATE = IDLE;
            }
            break;
        
        case CALIBRATE:
            if(VoCopter.Calibrate())
            {
                //TODO: Output that we finished tuning..
                STATE = IDLE;
            }
            break;
        
        case MOVE:
            VoCopter.SetBaseThrust(PARAMS[0]);
            VoCopter.SetPitchS(PARAMS[1]);
            VoCopter.SetPitchS(PARAMS[2]);
            VoCopter.SetPitchS(PARAMS[3]);
            STATE = IDLE;
            break;
            
        case SLEEP:
            //Land, if finished -> sleep..
            VoCopter.SetBaseThrust(0);
            //TODO: make Sleep blocking
            if(VoCopter.GetBaseThrust() != 0)            
                VoCopter.Stop();
            else
                Sleep();
            break;
            
        default:
        case IDLE:
            VoCopter.Stop();
            break;
    }
}

void Sleep()
{
    //TODO: implement..
}


/**
* Read the battery level from an analog pin
* @param pin : The number of the analog pin to read from.
* @return Returns the battery level measured on the pin.
*/
int BatLevel(int pin) {
    return mVtoL(1195 * 4096 / analogRead(pin));
}

/**
* Convert millivolts to battery level in %. (Calibrated for LiPo 3.7v 300mah)
* @param mV : The voltage in millivolts.
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
