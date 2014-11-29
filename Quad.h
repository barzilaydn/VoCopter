// Quad.h
// by Dan Barzilay (at barzilaydn @ gmail.com)
//
// Change log:
//     2014-11-05 - Makeover, adjusted for FreeIMU, new states.
//     2014-07-20 - initial release
//
// TODO:
//     * Quad: Make calibration real and better (check with FreeIMU software).
//     * Quad: Make more tests, check param if it's ok.
//     * Verify params are good for states
//     * Make SLEEP state.
//     * Add SETTINGS state where user can change definitions (i.e motor pins).
//     * Think about making all Serial communication outside of the state machine (before it).
//     * Change WIRE to i2c_t3
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
#ifndef Quad_h
#define Quad_h

/*-----------------
    Defines
  -----------------*/  
#define PITCH 0
#define ROLL  1
#define YAW   2

/*-----------------
    Includes
  -----------------*/
#if ARDUINO < 100
    #include "WProgram.h"
#else
    #include "Arduino.h"
#endif

#include "Qconfig.h"

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

#include <EEPROM.h>

//#define DEBUG
#include "DebugUtils.h"
#include "CommunicationUtils.h"
#include "FilteringScheme.h"
#include "MovingAvarageFilter.h"
#include "RunningAverage.h"
#include "FreeIMU.h"

#define HAS_GPS 0
/*-----------------
    Main Class Header
  -----------------*/
class Quad {
    public:
        Quad(const double, const double, const int, const int);
        void SetupMotors();
        void Init(bool);
        bool TunePID(int);
        bool Fly();
        bool Stop();
        bool Calibrate();
        void Test(int*);

        //Getters / Setters
        int* GetThrust();
        
        void SetBaseThrust(int);
        int GetBaseThrust();
        
        float* GetQ();
        
        void SetPitchS(double);
        double GetPitchI();
        double GetPitchO();
        
        void SetRollS(double);
        double GetRollI();
        double GetRollO();
        
        void SetYawS(double);
        double GetYawI();
        double GetYawO();   
        
        int GetTemp();
        float GetHeading();
        float GetAltitude();
        
        int* GetMotors();

    private:     
        void UpdateIMU();
        void restoreOffSets();

        void CancelTune();
        void changeAutoTune(int);
        void AutoTuneHelper(int, bool);
        PID* ChoosePID(int);
        PID_ATune* ChooseTuner(int);
        
        void SetMotors(bool forceThrust = true);
                
        //Motors
        int motors[4];
        int thrust[4];
        int baseThrust;
        int baseThrust_S;
        long time_since_start = 0;
        long last_write = 0;
        
        //Tuner parameters
        byte ATuneModeRemember;
        bool tuning;
        const double aTuneStep;
        const double aTuneNoise; //Sensor approx. noise (in deg)
        const int aTuneLookBack;
        const int sampleTime;
        
        //PID variables
        double Pitch_S, Pitch_I, Pitch_O = 0;
        double Roll_S, Roll_I, Roll_O = 0;
        double Yaw_S, Yaw_I, Yaw_O = 0;
        
        //A PID controller for each rotation axis.
        PID       PitchPID; 
        PID_ATune PitchTune;

        PID       RollPID; 
        PID_ATune RollTune;

        PID       YawPID;
        PID_ATune YawTune;
        
        //IMU
        FreeIMU my3IMU;
        float q[4];
        float val[12];
        int raw_values[11];
        char str[128];
        float ypr[3]; // yaw pitch roll
        float temperature;
        float heading;
        float altitude;
};
#endif