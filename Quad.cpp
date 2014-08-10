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

#if ARDUINO < 100
    #include "WProgram.h"
#else
    #include "Arduino.h"
#endif
#include "Quad.h"

//Constructor
Quad::Quad(const double aStep, const double aNoise, const int aLookBack, const int ST, const int conf, const int pin) :
                PitchPID(&Pitch_I, &Pitch_O, &Pitch_S, 1.0, 0.0, 0.0, DIRECT),
                PitchTune(&Pitch_I, &Pitch_O),
                RollPID(&Roll_I, &Roll_O, &Roll_S, 1.0, 0.0, 0.0, DIRECT),
                RollTune(&Roll_I , &Roll_O),
                YawPID(&Yaw_I, &Yaw_O, &Yaw_S, 1.0, 0.0, 0.0, DIRECT),
                YawTune(&Yaw_I  , &Yaw_O),
                aTuneStep(aStep),
                aTuneNoise(aNoise),
                aTuneLookBack(aLookBack),
                sampleTime(ST),
                config(conf),
                intPin(pin)
{}

void Quad::Setup(int *Motors){

    QDEBUG_PRINTLN(F("DEBUG: Initializing Motor pins..."));
    for(int i = 0; i < 4; i++)
    {
        pinMode(Motors[i], OUTPUT);
        thrust[i] = baseThrust;
    }
    motors = Motors;
    
    baseThrust = 0;
    baseThrust_S = 0;
    
    //Sensors Setup
    dmpReady = false;
    
    //Start I2C Master mode on pins 18,19 with a rate of 400kHz
    #ifdef CORE_TEENSY && __arm__
        Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    #else
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #endif
    
    QDEBUG_PRINTLN(F("DEBUG: Initializing I2C devices..."));
    mpu.initialize();
    
    QDEBUG_PRINTLN(F("DEBUG: Testing device connections..."));
    QDEBUG_PRINTLN(mpu.testConnection() ? F("DEBUG: MPU6050 connection successful") : F("DEBUG: MPU6050 connection failed"));
    
    // load and configure the DMP
    QDEBUG_PRINTLN(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default.

    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) 
       QDEBUG_PRINTLN(F("DMP Initialization was successful"));
    else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        QDEBUG_PRINT(F("DMP Initialization has failed (code "));
        QDEBUG_PRINT(devStatus);
        QDEBUG_PRINTLN(F(")"));
    }
    
    
    QDEBUG_PRINTLN(F("DEBUG: Configuring PIDs and Tuners..."));
    //Prepare the PID controllers:
    PitchPID.SetSampleTime(sampleTime);
    RollPID.SetSampleTime(sampleTime);
    YawPID.SetSampleTime(sampleTime);
    
    //Prepare the PID controller tuners:
    PitchTune.SetControlType(PID_ATune::ZIEGLER_NICHOLS_PID);
    RollTune.SetControlType(PID_ATune::ZIEGLER_NICHOLS_PID);
    YawTune.SetControlType(PID_ATune::ZIEGLER_NICHOLS_PID);
    tuning = false;
}

bool Quad::Fly()
{
    if(tuning) Quad::CancelTune();
    if(!dmpReady) StartDMP(); //Start the DMP if it's not already running.

    //Update Inputs.
    Quad::OrientationUpdate();
    
    //Compute new output values    
    PitchPID.Compute();
    RollPID.Compute();
    YawPID.Compute();
    
    //Smoothly get to the baseThrust setpoint.
    baseThrust += (baseThrust_S - baseThrust > 0) - (baseThrust_S - baseThrust < 0);
    
    Quad::SetMotors();
}

void Quad::CancelTune()
{
    //Cancel all tuners
    tuning = true;
    Quad::changeAutoTune(0);
    tuning = true;
    Quad::changeAutoTune(1);
    tuning = true;
    Quad::changeAutoTune(2);
}

PID* Quad::ChoosePID(int axis)
{
    switch(axis)
    {
        case PITCH:
            return &PitchPID;
        case ROLL:
            return &RollPID;
        default:
        case YAW:
            return &YawPID;
    }
}

PID_ATune* Quad::ChooseTuner(int axis)
{
    switch(axis)
    {
        case PITCH:
            return &PitchTune;
        case ROLL:
            return &RollTune;
        default:
        case YAW:
            return &YawTune;
    }
}

/**
* Turns the AutoTuner on/off.
* @param pid   : The PID controller.
* @param tuner : The AutoTuner to use.
* @return Returns whether the AutoTuner finished.
*/
bool Quad::TunePID(int axis)
{
    if(!dmpReady) StartDMP(); //Start the DMP if it's not already running.
    
    PID* pid = Quad::ChoosePID(axis);
    PID_ATune* tuner = Quad::ChooseTuner(axis);
    tuning = true;
    
    //Update Inputs.
    Quad::OrientationUpdate();
    baseThrust = 70;
    
    byte val = ((*tuner).Runtime());
    Quad::SetMotors();
    
    if (val != 0) //Finished tuning
    {
        tuning = false;

        // Set the tuning parameters
        (*pid).SetTunings((*tuner).GetKp(), (*tuner).GetKi(), (*tuner).GetKd());
        Quad::AutoTuneHelper(axis, false);
        
        return true;
    }
    return false;
}

/**
* Turns the AutoTuner on/off.
* @param pid   : The PID controller.
* @param tuner : The AutoTuner to turn on/off.
*/
void Quad::changeAutoTune(int axis)
{
    PID* pid = Quad::ChoosePID(axis);
    PID_ATune* tuner = Quad::ChooseTuner(axis);
    if (!tuning)
    {
        // Set the outputs to their default values.
        Pitch_O = 0;
        Roll_O = 0;
        Yaw_O = 0;
        
        (*tuner).SetNoiseBand(aTuneNoise);
        (*tuner).SetOutputStep(aTuneStep);
        (*tuner).SetLookbackSec(aTuneLookBack);
        Quad::AutoTuneHelper(axis, true);
        tuning = true;
    }
    else
    { 
        // Cancel AutoTune
        (*tuner).Cancel();
        tuning = false;
        Quad::AutoTuneHelper(axis, false);
    }
}

/**
* Restores/Saves the PID controller mode
* @param pid   : The PID controller.
* @param start : Whether to restore or save the controller mode.
*/
void Quad::AutoTuneHelper(int axis, bool start)
{
    PID* pid = Quad::ChoosePID(axis);
    if (start)
        ATuneModeRemember = (*pid).GetMode();
    else
        (*pid).SetMode(ATuneModeRemember);
}


void Quad::StartDMP()
{
    // Turn on the DMP.
    QDEBUG_PRINTLN(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Enable Arduino interrupt detection
    QDEBUG_PRINTLN(F("Enabling interrupt detection..."));
    attachInterrupt(intPin, dmpDataReady, RISING);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    QDEBUG_PRINTLN(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
}

void Quad::StopDMP()
{
    // Turn off the DMP.
    QDEBUG_PRINTLN(F("Disabling DMP..."));
    mpu.setDMPEnabled(false);

    // Disable Arduino interrupt detection
    QDEBUG_PRINTLN(F("Disabling interrupt detection..."));
    detachInterrupt(intPin);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    QDEBUG_PRINTLN(F("DMP Disabled!"));
    dmpReady = false;
}

void Quad::OrientationUpdate()
{
    if(Quad::mpuInterrupt || fifoCount >= packetSize)
    {
        // reset interrupt flag and get INT_STATUS byte
        Quad::mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();

        // get current FIFO count
        fifoCount = mpu.getFIFOCount();

        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            QDEBUG_PRINTLN(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;

            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Yaw_I   = ypr[0] * 180/M_PI;
            Pitch_I = ypr[1] * 180/M_PI;
            Roll_I  = ypr[2] * 180/M_PI;
            
            #ifdef QDEBUG
                // display initial world-frame acceleration, adjusted to remove gravity
                // and rotated based on known orientation from quaternion
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                QDEBUG_PRINT("aworld\t");
                QDEBUG_PRINT(aaWorld.x);
                QDEBUG_PRINT("\t");
                QDEBUG_PRINT(aaWorld.y);
                QDEBUG_PRINT("\t");
                QDEBUG_PRINTLN(aaWorld.z);
            #endif
        }
    }
}

void Quad::SetMotors()
{
    //Assign to motors
    if(config == X_CONF)
    {
        thrust[0] = (baseThrust != 0) ? constrain(baseThrust - Pitch_O + Roll_O + Yaw_O, 0, 255) : 0;
        thrust[1] = (baseThrust != 0) ? constrain(baseThrust - Pitch_O - Roll_O - Yaw_O, 0, 255) : 0;
        thrust[2] = (baseThrust != 0) ? constrain(baseThrust + Pitch_O - Roll_O + Yaw_O, 0, 255) : 0;
        thrust[3] = (baseThrust != 0) ? constrain(baseThrust + Pitch_O + Roll_O - Yaw_O, 0, 255) : 0;
    }
    else //+_CONF
    {
        thrust[0] = (baseThrust != 0) ? constrain(baseThrust + Pitch_O + Yaw_O, 0, 255) : 0;
        thrust[1] = (baseThrust != 0) ? constrain(baseThrust - Roll_O  - Yaw_O, 0, 255) : 0;
        thrust[2] = (baseThrust != 0) ? constrain(baseThrust - Pitch_O + Yaw_O, 0, 255) : 0;
        thrust[3] = (baseThrust != 0) ? constrain(baseThrust + Roll_O  - Yaw_O, 0, 255) : 0;
    }
    
    analogWrite(motors[0], thrust[0]);
    analogWrite(motors[1], thrust[1]);
    analogWrite(motors[2], thrust[2]);
    analogWrite(motors[3], thrust[3]);
}

/**
* Getters and Setters:
*/
int*   Quad::GetThrust()         { return thrust; }

void   Quad::SetBaseThrust(int t){ baseThrust_S = constrain(t, 0, 255); }
int    Quad::GetBaseThrust()     { return baseThrust; }

void   Quad::SetPitchS(double p) { Pitch_S = constrain(p, -90, 90); }
double Quad::GetPitchI()         { return Pitch_I; }
double Quad::GetPitchO()         { return Pitch_O; }

void   Quad::SetRollS(double p)  { Roll_S = constrain(p, -90, 90); }
double Quad::GetRollI()          { return Roll_I; }
double Quad::GetRollO()          { return Roll_O; }

void   Quad::SetYawS(double p)   { Yaw_S = constrain(p, -90, 90); }
double Quad::GetYawI()           { return Yaw_I; }
double Quad::GetYawO()           { return Yaw_O; } 

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool Quad::mpuInterrupt = false;
void dmpDataReady() {
    Quad::mpuInterrupt = true;
}