// VoCopter.ino
// by Dan Barzilay (at barzilaydn @ gmail.com)
//
// Change log:
//     2014-11-05 - Makeover, adjusted for FreeIMU, new states.
//     2014-07-20 - initial release
//
// TODO:
//    !* Fix comms to work with a lot of commands such that CMD size is >64bytes - filling the serial buffer. I need to keep reading the bytes as soon as they come in.
//    !* Create easy to change constants for P.I.D and use same P (I,D=0) for angle to rate controllers.
//    !* Re-do PID tuning, copy from ArduCopter: https://github.com/diydrones/ardupilot/blob/master/ArduCopter/control_autotune.pde
//     * Quad: Finish calibration.
//     
//     * Add SETTINGS state where user can change definitions (i.e motor pins).
//     * Implement landing via altitude sensor.  
//     * Encrypt communication.  
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

#include "Qconfig.h"

/*---LowPower---*/
#ifdef CORE_TEENSY
#include <Snooze.h>
SnoozeBlock config;
#else
//#include <LowPower.h>
#endif
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
#include <EEPROM.h>
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

/**
* FreeIMU library serial communication protocol
*/
//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FilteringScheme.h"
#include "MovingAvarageFilter.h"
#include "RunningAverage.h"
#include "FreeIMU.h"

#include "Quad.h"

/*---------------------------------------------------------------------
    Main Code
---------------------------------------------------------------------*/
Quad VoCopter(OUTPUT_STEP, NOISE, LOOK_BACK_SEC, SAMPLE_TIME); // Quad(OutputStep, Noise, LookBackSec, SampleTime(ms), Configuration)
int32_t STATE = SLEEP;
int32_t PARAMS[MAX_CNTRL_PARAMS];
bool STATE_FLAG = false;
unsigned long uptime;
unsigned long zero_time;
int BatLvl = 100;

void setup(void)
{
    analogReference(EXTERNAL);
    analogReadResolution(12);
    analogReadAveraging(32);
    zero_time = millis();
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    pinMode(21, INPUT);
    QDEBUG_BEGIN(115200);
    SERIAL_BEGIN(230400);    
    
    delay(1000);
    
    QDEBUG_PRINTLN(F("<--------------------------------------------->"));
    QDEBUG_PRINTLN(F("|    0xVC: Welcome to my debugging output!    |"));
    QDEBUG_PRINTLN(F("<--------------------------------------------->"));
    QDEBUG_PRINTLN(F(""));
    
    #ifdef CORE_TEENSY
    config.pinMode(RX_PIN, INPUT, LOW); //pin, mode, type
    #else
    pinMode(RX_PIN, INPUT);
    #endif
    
    VoCopter.Init(false);
    QDEBUG_PRINTLN(F("Waiting for client to connect."));
}

#include "Comms.h"
void loop(void)
{    
    BatLvl = VoCopter.UpdateBatLevel(39);
    uptime = millis();
    
    /*
    * Send / Receive user data.
    */
    receiveData();
    sendStatus();
    
    // Go to sleep if too low battery.
    if(BatLvl < SLEEP_BAT)
    {
        SERIAL_PRINTLN(Q_ALERT, 1, Q_LOW_BAT); // ALERT: Low Battery, BatLvl
        QDEBUG_PRINTLN(F("Low Battery")); // Low Battery
        STATE = SLEEP;
    }
    
    // Go to sleep if no commands received for more than GO_TO_SLEEP_TIME seconds.
    if(uptime - zero_time >= 1000 * GO_TO_SLEEP_TIME)
    {
        zero_time = uptime;
        SERIAL_PRINTLN(Q_ALERT, 1, Q_TIME_OUT); // // ALERT: Time out
        QDEBUG_PRINTLN(F("Alert: Time out.")); // Time out
        STATE = SLEEP;
    }
    
    /**
    * The main state machine.
    */
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
                    STATE_FLAG = !STATE_FLAG;
                }
            }
            break;
            
        case CALIBRATE:
            VoCopter.Calibrate(PARAMS[0]);
            break;
            
        case MOVE:
            VoCopter.SetBaseThrust(PARAMS[0]);
            VoCopter.SetYawS(PARAMS[1]);
            VoCopter.SetPitchS(PARAMS[2]);
            VoCopter.SetRollS(PARAMS[3]);
            VoCopter.Fly();
            STATE = FLY;
            break;
            
        case SLEEP:
            //Land, if finished -> sleep..
            QDEBUG_PRINTLN(F("Stopping VoCopter."));
            if(VoCopter.Stop())
            {
                SERIAL_PRINTLN(Q_SLEEPING, 0, 0); // "OK SLEEPING"
                QDEBUG_PRINTLN(F("Going to sleep.")); // "OK SLEEPING"
                Sleep();
            }
            break;
            
        case TEST:
            VoCopter.Test(PARAMS);
            break;
        
        case SETTINGS:
            QDEBUG_PRINTLN(F("Not implemented yet."));
            STATE = FLY;
            break;
        
        default:
        case FLY:          
            //TODO: update PID to run as fast as possible? or at least have it with correct time samples..
            VoCopter.Fly();
            break;            
    }    
    
    // Handle LED blinking.
    if(static_cast<int>(uptime / 1000) % (STATE + 2) == 0 || digitalRead(21))
    {
        digitalWrite(13, HIGH);
        //QDEBUG_PRINT(F("STATE:"));
        //QDEBUG_PRINTLN(STATE);
    }
    else
    digitalWrite(13, LOW);    
}

/*------------------------------
    Power Management
------------------------------*/

void wakeUp()
{
    // On wake up
    VoCopter.Init(true);
    QDEBUG_BEGIN(115200);
    SERIAL_BEGIN(230400);
    
    delay(1000);
    
    while(SERIAL_AVAILABLE() > 0) // Clear read buffer
        SERIAL_READ();
    
    // Clear params
    for (int c = 0; c < MAX_CNTRL_PARAMS; c++)    
        PARAMS[c] = 0;
    STATE = FLY;
    QDEBUG_PRINTLN(F("Client connected, setting STATE to FLY."));
    clientStarted = true;
    data_received = true;
    
    last_recv = uptime;
    QDEBUG_PRINTLN(F("Acknowledging command."));
    SERIAL_PRINTLN(Q_ACK, 0, 0);

    QDEBUG_PRINTLN(F("Woke up.")); // "OK SLEEPING"
}

void Sleep()
{    
    digitalWrite(13, HIGH);
    #ifdef CORE_TEENSY
    // Set DeepSleep on RX pin so that unit will wake up on SERIAL receive.
    Snooze.deepSleep( config );
    //-- BLOCKING sleep --//
    wakeUp();
    #else
    attachInterrupt(RX_PIN, wakeUp, LOW);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
    //-- BLOCKING sleep --//
    detachInterrupt(0);
    #endif    
}