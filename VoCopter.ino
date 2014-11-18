// VoCopter.ino
// by Dan Barzilay (at barzilaydn @ gmail.com)
//
// Change log:
//     2014-11-05 - Makeover, adjusted for FreeIMU, new states.
//     2014-07-20 - initial release
//
// TODO:
//    ~* Setup for the Low Voltage Warning interrupt. Available for Teensy only.
//     * Quad: Make more tests in Test function.
//     * Quad: Verify that Test function input is valid.
//    -* Quad: Make calibration real and better (check with FreeIMU software).
//     * Verify params are good for states
//     * Add SETTINGS state where user can change definitions (i.e motor pins).
//     * Implement landing via altitude sensor.  
//     * Encrypt communication.  
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

#include "Qconfig.h"

/*---LowPower---*/
#ifdef CORE_TEENSY
    #include <LowPower_Teensy3.h>
    TEENSY3_LP LP = TEENSY3_LP();
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
int STATE = SLEEP;
int PARAMS[MAX_CNTRL_PARAMS];
bool STATE_FLAG = false;
unsigned long uptime;
unsigned long zero_time;
RunningAverage BatLvl(10);
char sts_str[COMMAND_STAT_SIZE + 1];
char cntrl_str[COMMAND_CNTRL_SIZE + 1];

void setup(void)
{
    zero_time = millis();

    SERIAL_BEGIN(115200);
    VoCopter.Init(false);
}


void loop(void)
{
    uptime = millis();

/**
 * Send / Receive user data.
*/

    ///  --- SEND ---  ///

    UpdateBatLevel(39);
    // FORMAT: ' VCSTAT ; UP_TIME ; YAW,PITCH,ROLL,HEADING,ALTITUDE ; BAT_LVL '
    sprintf(sts_str, "VCSTAT;%u;%d,%d,%d,%d,%d;%d",
                                                    (unsigned int)(uptime * 1000),
                                                    (int)VoCopter.GetYawI(),
                                                    (int)VoCopter.GetPitchI(),
                                                    (int)VoCopter.GetRollI(),
                                                    (int)VoCopter.GetHeading(),
                                                    (int)VoCopter.GetAltitude(),
                                                    BatLvl.getAverage());
    SERIAL_PRINTLN(sts_str);
    
    /// --- RECEIVE --- ///
    
    for(int i = 0; SERIAL_AVAILABLE() > 0 && i < COMMAND_CNTRL_SIZE; i++)          
        cntrl_str[i] = SERIAL_READ();
    cntrl_str[i+1] = NULL; // Add null byte for end.
        
    // FORMAT: ' VCCTRL ; STATE ; PARAMS,PARAMS,PARAMS.. '
    char* command = strtok(cntrl_str, ";");

    if(command == "VCCTRL")
    {
        // Set the state.
        command = strtok(NULL, ";");
        int st = atoi(command);
        if(st > SLEEP && st < SETTINGS)
            STATE = st; 
        
        //Set the params.
        for (int c = 0; command != NULL && c < MAX_CNTRL_PARAMS; c++)
        {
            PARAMS[c] = atoi(command);     
            command = strtok(NULL, ";"); // Find the next param in command string.
        }
        
        zero_time = uptime;
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
            if(VoCopter.Stop())            
                //TODO: send OK sleeping
                Sleep();
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
    
    // Go to sleep if no commands received for more than 30 Secs.
    if(uptime - zero_time > 1000 * GO_TO_SLEEP_TIME)
        STATE = SLEEP;    
}

void wakeUp()
{
    // On wake up
    VoCopter.Init(true); //Restart systems.
    STATE = FLY;
}

void Sleep()
{
    #ifdef CORE_TEENSY
        // Set DeepSleep on RX pin so that unit will wake up on SERIAL receive.
        pinMode(RX_PIN, INPUT_PULLUP);
        LP.DeepSleep(GPIO_WAKE, PIN_RX_PIN); // Go to sleep... ZzZz
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
int mVtoL(int mV) {
    double l = -191.04 * pow(1000 * mV, 3) + 2132.6 * pow(1000 * mV, 2) - 7778.9 * 1000 * mV + 9309;
    if (l > 100)
        l = 100;
    else if (l < 0)
        l = 0;
    return (int)l;
}

/**
* Read the battery level from an analog pin
* @param pin : The number of the analog pin to read from.
* @return Returns the battery level measured on the pin.
*/
void UpdateBatLevel(int pin) {
    BatLvl.addValue(mVtoL(1195 * 4096 / analogRead(pin)));
}