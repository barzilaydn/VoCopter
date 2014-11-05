// VoCopter.ino
// by Dan Barzilay (at barzilaydn @ gmail.com)
//
// Change log:
//     2014-11-05 - Makeover, adjusted for FreeIMU, new states.
//     2014-07-20 - initial release
//
// TODO:
//    -* Quad: Make calibration real and better (check with FreeIMU software).
//     * Quad: Make more tests, check param if it's ok.
//     * Verify params are good for states
//     * Make SLEEP state.
//     * Add SETTINGS state where user can change definitions (i.e motor pins).
//     * Think about making all Serial communication outside of the state machine (before it).
//    ~* Change WIRE to i2c_t3
//
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

/*-----------------
    Includes
  -----------------*/

#if ARDUINO < 100
    #include "WProgram.h"
#else
    #include "Arduino.h"
#endif

/*---LowPower---*/
//#ifdef CORE_TEENSY
//    #include <LowPower_Teensy3.h>
//#else
//    #include <LowPower.h>
//#endif
/*------*/

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#undef USE_SIMULATION

/*-----------------
    FreeIMU Includes
  -----------------*/
#include <AP_Math_freeimu.h>
#include <Filter.h>    // Filter library
#include <Butter.h>    // Butterworth filter
#include <iCompass.h>

/**
 * FreeIMU library serial communication protocol
*/

#include <ADXL345.h>
#include <HMC58X3.h>
#include <LSM303.h>
#include <ITG3200.h>
#include <bma180.h>
#include <MS561101BA.h>
#include <BMP085.h>
#include <I2Cdev.h>
#include <MPU60X0.h>
#include <AK8975.h>
#include <AK8963.h>
#include <L3G.h>
#include <LPS331.h> 
#include <AP_Baro_MS5611.h>

#include <Wire.h>
//#include <i2c_t3.h> //For Teensy 3.0 and 3.1
#include <SPI.h>

#if defined(__AVR__)
	#include <EEPROM.h>
#endif

//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FilteringScheme.h"
#include "MovingAvarageFilter.h"
#include "RunningAverage.h"
#include "FreeIMU.h"

#include "Quad.h"

/*-----------------
    Quad States
  -----------------*/
#define SLEEP     0
#define FLY       1
#define CALIBRATE 2
#define TUNE      3
#define MOVE      4
#define TEST      5

/*---------------------------------------------------------------------
    Main Code
  ---------------------------------------------------------------------*/
Quad VoCopter(20, 0.5, 1, 10, X_CONF); // Quad(OutputStep, Noise, LookBackSec, SampleTime(ms), Configuration)
int STATE = SLEEP;
int PARAMS[10];
bool STATE_FLAG = false;

void setup(void)
{
    Serial3.begin(115200);
    
    int motorPins[] = { 5, 6, 9, 10 }; // {FrontLeft, FrontRight, BackRight, BackLeft}
    VoCopter.SetupMotors(motorPins); //(Motors)
    VoCopter.Init(false);
}

void loop(void)
{
    //TODO: implement landing via altitude sensor.    
    
    //TODO: implement idle state wake-up on RX receive via pin (!! pin 11 !!). see DeepSleep_Simple.ino example.
    
    //Here we should send/receive user data.
    //
    //--
    
    // The main state machine
    switch(STATE)
    {
        case TUNE:
            STATE_FLAG = !STATE_FLAG ? VoCopter.TunePID(PARAMS[0]) : STATE_FLAG;
            
            if(STATE_FLAG)
            {
                //TODO: Output that we finished tuning..
                
                //Land Quad and than change to FLY mode.
                if(VoCopter.Stop())
                {
                    STATE = FLY;
                    !STATE_FLAG;
                }
            }
            break;
        
        case CALIBRATE:
            VoCopter.Calibrate(); //Blocking.
            
            //TODO: Output that we finished calibrating..
                
            if(VoCopter.Stop())                
                STATE = FLY;
            break;
        
        case MOVE:
            VoCopter.SetBaseThrust(PARAMS[0]);
            VoCopter.SetPitchS(PARAMS[1]);
            VoCopter.SetRollS(PARAMS[2]);
            VoCopter.SetYawS(PARAMS[3]);
            STATE = FLY;
            break;
            
        case SLEEP:
            //Land, if finished -> sleep..
            //TODO: make Sleep blocking
            if(VoCopter.Stop())            
                STATE = FLY;            
            break;
        
        case TEST:
            VoCopter.Test(PARAMS);
            break;
        
        default:
        case FLY:
            //TODO: update PID to run as fast as possible? or at least have it with correct time samples..
            VoCopter.Fly();
            break;            
    }
}

void Sleep()
{
    //TODO: implement..
    
    //When waking-up:
    VoCopter.Init(true);
    STATE = FLY;
}


/**
* Read the battery level from an analog pin
* @param pin : The number of the analog pin to read from.
* @return Returns the battery level measured on the pin.
*/
int level = 0;
void UpdateBatLevel(int pin) {
    level = level*.6 + (mVtoL(1195 * 4096 / analogRead(pin)))*.4;
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
