// VoCopter.ino
// by Dan Barzilay (at barzilaydn @ gmail.com)
//
// Change log:
//     2014-11-05 - Makeover, adjusted for FreeIMU, new states.
//     2014-07-20 - initial release
//
// TODO:
//    !* Test if angles should be YPR or Euler.
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

/*---------------------------------------------------------------------
    Main Code
  ---------------------------------------------------------------------*/
Quad VoCopter(OUTPUT_STEP, NOISE, LOOK_BACK_SEC, SAMPLE_TIME); // Quad(OutputStep, Noise, LookBackSec, SampleTime(ms), Configuration)
int32_t STATE = 0;
int32_t PARAMS[MAX_CNTRL_PARAMS];
bool STATE_FLAG = false;
unsigned long uptime;
unsigned long zero_time;
int BatLvl = 100;
char sts_str[COMMAND_SIZE];
char cntrl_str[COMMAND_SIZE];

void setup(void)
{
    analogReference(EXTERNAL);
    analogReadResolution(12);
    analogReadAveraging(32);
    zero_time = millis();
    pinMode(13, OUTPUT);
    pinMode(21, INPUT);
    QDEBUG_BEGIN(115200);
    SERIAL_BEGIN(230400);
    VoCopter.Init(false);
    
    #ifdef CORE_TEENSY
        config.pinMode(RX_PIN, INPUT, LOW); //pin, mode, type
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
        PARAMS[c] = NULL;
    
    /*
     * Send / Receive user data.
    */
    /// --- RECEIVE --- ///
    
    if(SERIAL_AVAILABLE() >= COMMAND_SIZE)
    {
        // Read up command:
        SERIAL_READ_BYTES(cntrl_str, COMMAND_SIZE);
        
        // If matching "signature": (means that it is really a command)
        if(cntrl_str[0] == SERIAL_HEAD[0] && cntrl_str[COMMAND_SIZE-1] == SERIAL_TAIL[1])
            for(int i = 2; i < MAX_CNTRL_PARAMS * 2; i=i+4)
            {
                // Little-endian byte array to unsigned int conversion
                int32_t st = (int32_t)cntrl_str[i] + (int32_t)(cntrl_str[i+1] << 8)
                                + (int32_t)(cntrl_str[i+2] << 16) + (int32_t)(cntrl_str[i+3] << 24);
                
                if(i == 2)
                    if(st >= SLEEP && st <= NUM_STATES-1)
                        STATE = st; 
                else
                    PARAMS[(i-2)/4 - 1] = (int32_t)st;
            }
        
        // Update time since inactive:
        zero_time = uptime;
    }

    if(STATE != CALIBRATE)
    {
        ///  --- SEND ---  ///
        float* q = VoCopter.GetQ();
        int* thrusts = VoCopter.GetMotors();
        // FORMAT: ' UP_TIME ; STATE ; 10000*q[0],10000*q[1],10000*q[2],10000*q[3],HEADING,ALTITUDE ; FRONT_LEFT,FRONT_RIGHT,BACK_LEFT,BACK_RIGHT ; BAT_LVL '
        SERIAL_PRINTLN(Q_STATUS, 13, 
                                    static_cast<int32_t>((uptime / 1000)),
                                    static_cast<int32_t>(STATE),
                                    static_cast<int32_t>((q[0] * 10000)),
                                    static_cast<int32_t>((q[1] * 10000)),
                                    static_cast<int32_t>((q[2] * 10000)),
                                    static_cast<int32_t>((q[3] * 10000)),
                                    static_cast<int32_t>(VoCopter.GetHeading()),
                                    static_cast<int32_t>(VoCopter.GetAltitude()),
                                    static_cast<int32_t>(thrusts[0]),
                                    static_cast<int32_t>(thrusts[1]),
                                    static_cast<int32_t>(thrusts[2]),
                                    static_cast<int32_t>(thrusts[3]),
                                    static_cast<int32_t>(BatLvl));
    }
    
    // Go to sleep if too low battery.
    if(BatLvl < SLEEP_BAT)
    {
        SERIAL_PRINTLN(Q_ALERT,1,Q_LOW_BAT); // ALERT: Low Battery, BatLvl
        QDEBUG_PRINTLN("Low Battery"); // Low Battery
        STATE = SLEEP;
    }
    
    // Go to sleep if no commands received for more than 30 Secs.
    if(uptime - zero_time >= 1000 * GO_TO_SLEEP_TIME)
    {
        zero_time = uptime;
        SERIAL_PRINTLN(Q_ALERT,1,Q_TIME_OUT); // // ALERT: Time out
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
            VoCopter.SetYawS(PARAMS[1]);
            VoCopter.SetPitchS(PARAMS[2]);
            VoCopter.SetRollS(PARAMS[3]);
            VoCopter.Fly();
            STATE = FLY;
            break;
            
        case SLEEP:
            //Land, if finished -> sleep..
            if(VoCopter.Stop())
            {
                SERIAL_PRINTLN(Q_SLEEPING, 0); // "OK SLEEPING"
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

    if((int)(uptime / 1000) % (STATE+1) == 0 || digitalRead(21))
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
    SERIAL_PRINTLN(Q_ALERT, 1, Q_WOKE_UP); // "WOKE UP"
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
    int32_t x = analogRead(pin);
    BatLvl = mVtoL((178*x*x + 2688757565 - 1184375 * x) / 372346);
}

/*------------------------------
    Serial command builder
  ------------------------------*/

void SERIAL_PRINTLN(unsigned long cmd, int len, ...)
{
    //Declare a va_list macro and initialize it with va_start
    va_list argList;
    va_start(argList, len);
    
    unsigned char buffer[COMMAND_SIZE];
    int count;
    int32_t param;
    
    //Send head bytes
    insertToCmdBuffer(buffer, 2, SERIAL_HEAD, 0);
    //SERIAL_WRITE(SERIAL_HEAD, 2);
    
    //Send CMD
    param = static_cast<int32_t>(cmd);
    insertToCmdBuffer(buffer, 4, reinterpret_cast<const unsigned char*>(&param), 2);
    //SERIAL_WRITE(reinterpret_cast<const unsigned char*>(cmd), 4);
    
    //Send arguments
    for(count = 0; count < len; count++)
    {
        if(count < MAX_CNTRL_PARAMS)
        {
            param = va_arg(argList, int32_t);
            insertToCmdBuffer(buffer, 4, reinterpret_cast<const unsigned char*>(&param), 6 + 4 * count);
            //SERIAL_WRITE(va_arg(argList, const unsigned char*), 4);
        }
    }
    
    //Fill up buffer
    for(; count < MAX_CNTRL_PARAMS; count++)
        insertToCmdBuffer(buffer, 4, SERIAL_EMPTY, 6 + 4 * count);
        //SERIAL_WRITE(SERIAL_EMPTY, 4);
    
    //Send end bytes
    insertToCmdBuffer(buffer, 2, SERIAL_TAIL, COMMAND_SIZE-2);
    //SERIAL_WRITE(SERIAL_TAIL, 2);
    
    // Send the data:
    SERIAL_WRITE(buffer, COMMAND_SIZE);
    
    va_end(argList);
}

void insertToCmdBuffer (unsigned char buffer[], int dataLen,  const unsigned char data[], int startingFrom)
{
    for(int i = 0; i < dataLen; i++)    
        if(startingFrom + i < COMMAND_SIZE)
            buffer[startingFrom + i] = data[i];   
}