// Quad.h
// by Dan Barzilay (at barzilaydn @ gmail.com)
/*
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
        void Fly();
        bool Stop();
        void Calibrate(int);
        void Test(int32_t*);
        int  UpdateBatLevel(int pin);
        
        //Getters / Setters   
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
        char serial_busy_wait();
        void writeArr(void * varr, uint8_t arr_length, uint8_t type_bytes);
        void writeVar(void * val, uint8_t type_bytes);
        
        void CancelTune();
        void changeAutoTune(int);
        void AutoTuneHelper(int, bool);
        PID* ChoosePID(int);
        PID_ATune* ChooseTuner(int);
        
        void SetMotors(bool forceThrust = true);
        
        void arr3_rad_to_deg(float * arr);
        void frame_conversion_ef_to_bf(float * ef, float * bf);
        
        int mVtoL(double mV);
        
        //Motors
        int motors[4] = { FRONT_LEFT, FRONT_RIGHT, BACK_RIGHT, BACK_LEFT };
        int thrust[4] = {0, 0, 0, 0};
        int baseThrust;
        int baseThrust_S;
        
        //Tuner parameters
        byte ATuneModeRemember;
        bool tuning;
        const double aTuneStep;
        const double aTuneNoise; //Sensor approx. noise (in deg)
        const int aTuneLookBack;
        const int sampleTime;
        
        //PID variables:
        double Yaw   , Pitch   , Roll    = 0;
            // Earth-ref angle PIDs
        double EYaw_S, EPitch_S, ERoll_S = 0;          
        double EYaw_I, EPitch_I, ERoll_I = 0;
        double EYaw_O, EPitch_O, ERoll_O = 0;        
            // Body-ref rate PIDs
        double BYawRate_S, BPitchRate_S, BRollRate_S = 0;          
        double BYawRate_I, BPitchRate_I, BRollRate_I = 0;
        double BYawRate_O, BPitchRate_O, BRollRate_O = 0;
        
        //A PID controller for each rotation axis:
            // Earth-ref angle PIDs
        PID       EPitchPID;
        PID       ERollPID;
        PID       EYawPID;
            // Body-ref angle PIDs
        PID       BPitchRatePID;
        PID_ATune BPitchRateTune;
        
        PID       BRollRatePID;
        PID_ATune BRollRateTune;

        PID       BYawRatePID;
        PID_ATune BYawRateTune;
        
        //IMU
        FreeIMU my3IMU;
        float q[4];
        float val[10];
        int raw_values[11];
        char str[128];
        float ypr[3]; // yaw pitch roll
        float brpy[3]; // body yaw pitch roll rate
        float temperature;
        float heading;
        float altitude;
        float starting_altitude;

};
#endif