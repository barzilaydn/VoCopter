// Quad
// by Dan Barzilay (at barzilaydn@gmail.com)
//
// Change log:
//     2014-08-09 - initial release

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

#if ARDUINO < 100
    #include "WProgram.h"
#else
    #include "Arduino.h"
#endif

#include <EEPROM.h>
#define ACTIVE 200 //Random value, just to mark that settings are already written.

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#undef USE_SIMULATION

#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#include <i2c_t3.h>
//#include <Wire.h>

#define X_CONF    0
#define PLUS_CONF 1

//#define QDEBUG
#ifdef QDEBUG
    #define QDEBUG_PRINT(x) Serial3.print(x)
    #define QDEBUG_PRINTF(x, y) Serial3.print(x, y)
    #define QDEBUG_PRINTLN(x) Serial3.println(x)
    #define QDEBUG_PRINTLNF(x, y) Serial3.println(x, y)
#else
    #define QDEBUG_PRINT(x)
    #define QDEBUG_PRINTF(x, y)
    #define QDEBUG_PRINTLN(x)
    #define QDEBUG_PRINTLNF(x, y)
#endif 

/**
* Axis
*/
#define PITCH 0
#define ROLL  1
#define YAW   2
/*---------*/

#define EAST 1
#define WEST -1

#define MAG_DECLINATION     EAST
#define MAG_DECLINATION_DEG 4

class Quad {
    public:
        Quad(const double, const double, const int, const int, const int, const int);
        void Setup(int*);
        bool TunePID(int);
        bool Fly();
        void Stop();
        bool Calibrate();

        //Getters / Setters
        int* GetThrust();
        
        void SetBaseThrust(int);
        int GetBaseThrust();
        
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
        
        //MPU6050 Interrupt flag
        static volatile bool mpuInterrupt;

    private:
        void StartDMP();
        void StopDMP();
        void OrientationUpdate();
        void restoreOffSets();
        void meanSensors();

        void CancelTune();
        void changeAutoTune(int);
        void AutoTuneHelper(int, bool);
        PID* ChoosePID(int);
        PID_ATune* ChooseTuner(int);
        
        void SetMotors();
                
        //Motors
        int *motors;
        int *thrust;
        int baseThrust;
        int baseThrust_S;
        const int config;
        
        //Tuner parameters
        byte ATuneModeRemember;
        bool tuning;
        const double aTuneStep;
        const double aTuneNoise; //Sensor approx. noise (in deg)
        const int aTuneLookBack;
        const int sampleTime;
        
        //PID variables
        double Pitch_S, Pitch_I, Pitch_O;
        double Roll_S, Roll_I, Roll_O;
        double Yaw_S, Yaw_I, Yaw_O;
        
        //A PID controller for each rotation axis.
        PID       PitchPID; 
        PID_ATune PitchTune;

        PID       RollPID; 
        PID_ATune RollTune;

        PID       YawPID;
        PID_ATune YawTune;
        
        //Sensors
        MPU6050 mpu;
        HMC5883L mag;
        
        //Calibrating variables
        bool finishedCal;
        unsigned long lastTime;
        int calState;
        const int buffersize;    //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
        const int acel_deadzone; //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
        const int gyro_deadzone; //gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
        int16_t ax, ay, az, gx, gy, gz;
        int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;
        int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;
        
        //Mag vars
        int16_t mx, my, mz;
        float heading;
        float init_heading;
        
        // MPU control/status vars
        const int intPin;
        bool dmpReady;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer
        uint16_t temperature;
        
        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements
        VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
        VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
        VectorFloat gravity;    // [x, y, z]            gravity vector
        float euler[3];         // [psi, theta, phi]    Euler angle container
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

};
void dmpDataReady();
#endif