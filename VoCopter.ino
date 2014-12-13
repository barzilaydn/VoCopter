// VoCopter.ino
// by Dan Barzilay (at barzilaydn @ gmail.com)
//
// Change log:
//     2014-11-05 - Makeover, adjusted for FreeIMU, new states.
//     2014-07-20 - initial release
//
// TODO:
//     * Quad: Verify that Test function input is valid.
//     * Quad: Make more tests in Test function.
//    -* Quad: Make calibration real and better (check with FreeIMU software).
//     * Verify params are good for states
//     * Add SETTINGS state where user can change definitions (i.e motor pins).
//     * Implement landing via altitude sensor.  
//     * Encrypt communication.  
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

/**
 * FreeIMU library serial communication protocol
*/

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
#define SETTINGS  6

/*---------------------------------------------------------------------
    Main Code
  ---------------------------------------------------------------------*/
Quad VoCopter(OUTPUT_STEP, NOISE, LOOK_BACK_SEC, SAMPLE_TIME); // Quad(OutputStep, Noise, LookBackSec, SampleTime(ms), Configuration)
int STATE = 0;
int PARAMS[MAX_CNTRL_PARAMS];
bool STATE_FLAG = false;
unsigned long uptime;
unsigned long zero_time;
int BatLvl = 100;
char sts_str[COMMAND_STAT_SIZE];
char cntrl_str[COMMAND_CNTRL_SIZE];

void setup(void)
{
    analogReference(EXTERNAL);
    analogReadResolution(12);
    analogReadAveraging(32);
    zero_time = millis();
    pinMode(13, OUTPUT);
    QDEBUG_BEGIN(115200);
    SERIAL_BEGIN(230400);
    VoCopter.Init(false);
    
    #ifdef CORE_TEENSY
        config.pinMode(RX_PIN, INPUT, LOW);//pin, mode, type
    #else
        pinMode(RX_PIN, INPUT);
    #endif
}


void loop(void)
{
    UpdateBatLevel(39);
    uptime = millis();
    
    // Clear params
    for (int c = 0; c < MAX_CNTRL_PARAMS; c++)
    {
        PARAMS[c] = NULL;     
    }
/*
 * Send / Receive user data.
*/
    /// --- RECEIVE --- ///
    
    if(SERIAL_AVAILABLE() > 0)
    {
        int i = 0;
        for(; SERIAL_AVAILABLE() > 0 && i < COMMAND_CNTRL_SIZE; i++)
        {
            cntrl_str[i] = SERIAL_READ();
            delay(1); // Wait for the next byte.
        }
        cntrl_str[i+1] = NULL; // Add null byte for end.
        // FORMAT: ' VCCTRL ; STATE ; PARAMS,PARAMS,PARAMS.. '
        char* command = strtok(cntrl_str, ";");

        if(strcmp(command,"VCCTRL") == 0)
        {
            // Set the state.
            command = strtok(NULL, ";");
            int st = atoi(command);
            if(st >= SLEEP && st <= SETTINGS)
                STATE = st; 
            
            command = strtok(NULL, ";"); // Find the next param in command string.
            char* paramstr = strtok(command, ","); // Find the next param in command string.
            //Set the params.
            for (int c = 0; paramstr != NULL && c < MAX_CNTRL_PARAMS; c++)
            {
                PARAMS[c] = atoi(paramstr);     
                paramstr = strtok(NULL, ","); // Find the next param in command string.
            }
            
        }        
        zero_time = uptime;
    }

    if(STATE != CALIBRATE)
    {
        ///  --- SEND ---  ///
        float* q = VoCopter.GetQ();
        int* thrusts = VoCopter.GetMotors();
        // FORMAT: ' VCSTAT ; UP_TIME ; STATE ; 100*q[0],100*q[1],100*q[2],100*q[3],HEADING,ALTITUDE ; FRONT_LEFT,FRONT_RIGHT,BACK_LEFT,BACK_RIGHT ; BAT_LVL '
        sprintf(sts_str, "VCSTAT;%u;%d;%d,%d,%d,%d,%d,%d;%d,%d,%d,%d;%d",
                                                        (unsigned int)(uptime / 1000),
                                                        (int)STATE,
                                                        (int)(q[0] * 100),
                                                        (int)(q[1] * 100),
                                                        (int)(q[2] * 100),
                                                        (int)(q[3] * 100),
                                                        (int)VoCopter.GetHeading(),
                                                        (int)VoCopter.GetAltitude(),
                                                        (int)thrusts[0],
                                                        (int)thrusts[1],
                                                        (int)thrusts[2],
                                                        (int)thrusts[3],
                                                        (int)BatLvl);
        SERIAL_PRINTLN(sts_str);
    }
    // Go to sleep if too low battery.
    if(BatLvl < SLEEP_BAT)
    {
        SERIAL_PRINTLN("VCALRT;LB"); // Low Battery
        QDEBUG_PRINTLN("VCALRT;LB"); // Low Battery
        STATE = SLEEP;
    }
    
    // Go to sleep if no commands received for more than 30 Secs.
    if(uptime - zero_time >= 1000 * GO_TO_SLEEP_TIME)
    {
        zero_time = uptime;
        SERIAL_PRINTLN("VCALRT;TO"); // Time out
        QDEBUG_PRINTLN("VCALRT;TO"); // Time out
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
                    !STATE_FLAG;
                }
            }
            break;
        
        case CALIBRATE:
            VoCopter.Calibrate(PARAMS[0]);
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
            if(VoCopter.Stop())
            {
                SERIAL_PRINTLN("VCSLEP;OK"); // "OK SLEEPING"
                QDEBUG_PRINTLN("VCSLEP;OK"); // "OK SLEEPING"
                Sleep();
            }
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

    if((int)(uptime / 1000) % (STATE+1) == 0)
    {
        digitalWrite(13, HIGH);
        //QDEBUG_PRINT(F("STATE:"));
        //QDEBUG_PRINTLN(STATE);
    }
    else
        digitalWrite(13, LOW);    
}

void wakeUp()
{
    // On wake up
    VoCopter.Init(true); //Restart systems.
    STATE = FLY;
    SERIAL_PRINTLN("VCSLEP;WAKEUP"); // "WAKEUP"
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

/**
* Convert millivolts to battery level in %. (Calibrated for LiPo 3.7v 300mah)
* @param mV : The voltage in millivolts.
* @return Returns battery level in %.
*/
int mVtoL(double mV) {
    return (int)map(constrain(mV, 3000, 3600), 3000, 3600, 0, 100);
}

/**
* Read the battery level from an analog pin
* @param pin : The number of the analog pin to read from.
* @return Returns the battery level measured on the pin.
*/
void UpdateBatLevel(int pin) {
    uint32_t x = analogRead(39);
    BatLvl = mVtoL((178*x*x + 2688757565 - 1184375 * x) / 372346);
}