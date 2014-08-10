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
                intPin(pin),
                buffersize(10000),
                acel_deadzone(8),
                gyro_deadzone(1)
{}

void Quad::Setup(int *Motors)
{
    QDEBUG_PRINTLN(F("DEBUG: Initializing Motor pins..."));
    for(int i = 0; i < 4; i++)
    {
        pinMode(Motors[i], OUTPUT);
        thrust[i] = baseThrust;
    }
    motors = Motors;
    
    baseThrust = 0;
    baseThrust_S = 0;
    
    //Start I2C Master mode on pins 18,19 with a rate of 400kHz
    #ifdef CORE_TEENSY && __arm__
        Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);
    #else
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #endif
    
    QDEBUG_PRINTLN(F("DEBUG: Initializing I2C devices..."));
    mpu.initialize();
    mag.initialize();
    
    QDEBUG_PRINTLN(F("DEBUG: Testing device connections..."));
    QDEBUG_PRINTLN(mpu.testConnection() ? F("DEBUG: MPU6050 connection successful") : F("DEBUG: MPU6050 connection failed"));
    QDEBUG_PRINTLN(mag.testConnection() ? F("DEBUG: HMC5883L connection successful") : F("DEBUG: HMC5883L connection failed"));
    
    // Get starting heading
    mag.getHeading(&mx, &my, &mz);
    float heading = atan2(my, mx);
    if(heading < 0)
        heading += 2 * M_PI;
    heading *= 180/M_PI;
    heading += MAG_DECLINATION * MAG_DECLINATION_DEG; //Compensate for the magnetic declination
    init_heading = heading;
    
    QDEBUG_PRINT(F("DEBUG: Init. Heading is "));
    QDEBUG_PRINT(heading);
    QDEBUG_PRINTLN(F(" degrees"));

    // load and configure the DMP
    QDEBUG_PRINTLN(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    //Sensors Setup
    dmpReady = false;
    calState = -1;
    finishedCal = false;
    
    // Scaled for min sensitivity; Read from EEPROM.
    if(EEPROM.read(0) != ACTIVE) //Check if it's a first-time setup of the settings
    {
        EEPROM.write(0, ACTIVE);
        EEPROM.write(1, 220);
        EEPROM.write(2, 76);
        EEPROM.write(3, -85);
        EEPROM.write(4, 0);
        EEPROM.write(5, 0);
        EEPROM.write(6, 1788); // 1688 factory default.
    }
    Quad::restoreOffSets();
    
    // Make sure it worked (returns 0 if so)
    if (devStatus == 0) 
       QDEBUG_PRINTLN(F("DEBUG: DMP Initialization was successful"));
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        QDEBUG_PRINT(F("DEBUG: DMP Initialization has failed (code "));
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
    Quad::restoreOffSets();

    //Update Inputs.
    Quad::OrientationUpdate();
    
    if(PitchPID.GetMode() != 1)
        PitchPID.SetMode(AUTOMATIC);
    if(RollPID.GetMode() != 1)
        RollPID.SetMode(AUTOMATIC);
    if(YawPID.GetMode() != 1)
        YawPID.SetMode(AUTOMATIC);
    
    //Compute new output values    
    PitchPID.Compute();
    RollPID.Compute();
    YawPID.Compute();
    
    //Smoothly get to the baseThrust setpoint.
    baseThrust += (baseThrust_S - baseThrust > 0) - (baseThrust_S - baseThrust < 0);
    
    Quad::SetMotors();
}

void Quad::Stop()
{
    if(tuning) Quad::CancelTune();
    if(!dmpReady) Quad::StopDMP();
    Quad::SetBaseThrust(0);
    Quad::SetPitchS(0);
    Quad::SetRollS(0);
    Quad::SetYawS(0);
    PitchPID.SetMode(0);
    RollPID.SetMode(0);
    YawPID.SetMode(0);
    
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

bool Quad::Calibrate()
{    
    if(tuning) Quad::CancelTune();
    if(!dmpReady) StartDMP(); //Start the DMP if it's not already running.
    
    unsigned long now = millis();
    unsigned long timeChange = (now - lastTime);
    
    if(now < 180000)
    {   
        QDEBUG_PRINTLN(F("DEBUG: 3 minutes need to pass (To warm up the MPU)"));
        finishedCal = false;
        return false;
    }
    
    if(calState == -1)
    {
        QDEBUG_PRINTLN(F("DEBUG: Calibrating..."));

        mpu.setXGyroOffset(0);
        mpu.setYGyroOffset(0);
        mpu.setZGyroOffset(0);
        mpu.setXAccelOffset(0);
        mpu.setYAccelOffset(0);
        mpu.setZAccelOffset(0);
        
        Quad::meanSensors()
        
        QDEBUG_PRINTLN(F("Calculating offsets..."));
        
        ax_offset=-mean_ax/8;
        ay_offset=-mean_ay/8;
        az_offset=(16384-mean_az)/8;

        gx_offset=-mean_gx/4;
        gy_offset=-mean_gy/4;
        gz_offset=-mean_gz/4;
        
        lastTime = now;
        calState = 0;
    }

    if(timeChange >= 1000) // > 1sec
    {
        mpu.setXAccelOffset(ax_offset);
        mpu.setYAccelOffset(ay_offset);
        mpu.setZAccelOffset(az_offset);

        mpu.setXGyroOffset(gx_offset);
        mpu.setYGyroOffset(gy_offset);
        mpu.setZGyroOffset(gz_offset);

        Quad::meanSensors();
        
        QDEBUG_PRINTLN(F("DEBUG: ..."));
        
        if (abs(mean_ax)<=acel_deadzone) calState++;
        else ax_offset=ax_offset-mean_ax/acel_deadzone;

        if (abs(mean_ay)<=acel_deadzone) calState++;
        else ay_offset=ay_offset-mean_ay/acel_deadzone;

        if (abs(16384-mean_az)<=acel_deadzone) calState++;
        else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

        if (abs(mean_gx)<=gyro_deadzone) calState++;
        else gx_offset=gx_offset-mean_gx/(gyro_deadzone+1);

        if (abs(mean_gy)<=gyro_deadzone) calState++;
        else gy_offset=gy_offset-mean_gy/(gyro_deadzone+1);

        if (abs(mean_gz)<=gyro_deadzone) calState++;
        else gz_offset=gz_offset-mean_gz/(gyro_deadzone+1);

        if (calState==6)
        {
            EEPROM.write(0, ACTIVE);
            EEPROM.write(1, gx_offset);
            EEPROM.write(2, gy_offset);
            EEPROM.write(3, gz_offset);
            EEPROM.write(4, ax_offset);
            EEPROM.write(5, ay_offset);
            EEPROM.write(6, az_offset);
            
            QDEBUG_PRINTLN(F("DEBUG: Finished calibrating!!"));

            finishedCal = true;
            calState = -1;
        }
    }
    else    
        finishedCal = false;
    
    QDEBUG_PRINT(F("DEBUG: XGyroOffset: "));
    QDEBUG_PRINTLN(mpu.getXAccelOffset());
    QDEBUG_PRINT(F("DEBUG: YGyroOffset: "));
    QDEBUG_PRINTLN(mpu.getYGyroOffset());
    QDEBUG_PRINT(F("DEBUG: ZGyroOffset: "));
    QDEBUG_PRINTLN(mpu.getZGyroOffset());
    QDEBUG_PRINT(F("DEBUG: ZAccelOffset: "));
    QDEBUG_PRINTLN(mpu.getZAccelOffset());
    
    return finishedCal;
}

void Quad::meanSensors()
{
    long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

    while (i<(buffersize+101))
    {
        // read raw accel/gyro measurements from device
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        if (i>100 && i<=(buffersize+100)) //First 100 measures are discarded
        { 
            buff_ax=buff_ax+ax;
            buff_ay=buff_ay+ay;
            buff_az=buff_az+az;
            buff_gx=buff_gx+gx;
            buff_gy=buff_gy+gy;
            buff_gz=buff_gz+gz;
        }
        if (i==(buffersize+100))
        {
            mean_ax=buff_ax/buffersize;
            mean_ay=buff_ay/buffersize;
            mean_az=buff_az/buffersize;
            mean_gx=buff_gx/buffersize;
            mean_gy=buff_gy/buffersize;
            mean_gz=buff_gz/buffersize;
        }
        i++;
        delay(2); //Needed so we don't get repeated measures
    }
}

void Quad::restoreOffSets()
{
    mpu.setXGyroOffset(EEPROM.read(1));
    mpu.setYGyroOffset(EEPROM.read(2));
    mpu.setZGyroOffset(EEPROM.read(3));
    mpu.setXAccelOffset(EEPROM.read(4));
    mpu.setYAccelOffset(EEPROM.read(5));
    mpu.setZAccelOffset(EEPROM.read(6));
    
    QDEBUG_PRINT(F("DEBUG: XGyroOffset: "));
    QDEBUG_PRINTLN(EEPROM.read(1));
    QDEBUG_PRINT(F("DEBUG: YGyroOffset: "));
    QDEBUG_PRINTLN(EEPROM.read(2));
    QDEBUG_PRINT(F("DEBUG: ZGyroOffset: "));
    QDEBUG_PRINTLN(EEPROM.read(3));
    QDEBUG_PRINT(F("DEBUG: XAccelOffset: "));
    QDEBUG_PRINTLN(EEPROM.read(4));
    QDEBUG_PRINT(F("DEBUG: YAccelOffset: "));
    QDEBUG_PRINTLN(EEPROM.read(5));
    QDEBUG_PRINT(F("DEBUG: ZAccelOffset: "));
    QDEBUG_PRINTLN(EEPROM.read(6));
    
    finishedCal = true;
}

void Quad::StartDMP()
{
    // Turn on the DMP.
    QDEBUG_PRINTLN(F("DEBUG: Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // Enable Arduino interrupt detection
    QDEBUG_PRINTLN(F("DEBUG: Enabling interrupt detection..."));
    attachInterrupt(intPin, dmpDataReady, RISING);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    QDEBUG_PRINTLN(F("DEBUG: DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
}

void Quad::StopDMP()
{
    // Turn off the DMP.
    QDEBUG_PRINTLN(F("DEBUG: Disabling DMP..."));
    mpu.setDMPEnabled(false);

    // Disable Arduino interrupt detection
    QDEBUG_PRINTLN(F("DEBUG: Disabling interrupt detection..."));
    detachInterrupt(intPin);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    QDEBUG_PRINTLN(F("DEBUG: DMP Disabled!"));
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
            QDEBUG_PRINTLN(F("DEBUG: FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & 0x02) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
            
            // Get Pitch Roll Yaw values
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
                QDEBUG_PRINT("DEBUG: aWorld\t");
                QDEBUG_PRINT(aaWorld.x);
                QDEBUG_PRINT("\t");
                QDEBUG_PRINT(aaWorld.y);
                QDEBUG_PRINT("\t");
                QDEBUG_PRINTLN(aaWorld.z);
            #endif
        }
    }
    
    //Get temp
    temperature = mpu.getTemperature();
    QDEBUG_PRINT(F("DEBUG: Temperature is "));
    QDEBUG_PRINT(temperature);
    QDEBUG_PRINTLN(F("C degrees"));
    
    // Get values from Mag
    mag.getHeading(&mx, &my, &mz);
    float heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;
    heading *= 180/M_PI;
    
    heading += MAG_DECLINATION * MAG_DECLINATION_DEG; //Compensate for the magnetic declination
    
    //Yaw_I = 
     
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

int    Quad::GetTemp()           { return (int)temperature; }

float  Quad::GetHeading()        { return heading; }

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool Quad::mpuInterrupt = false;
void dmpDataReady() {
    Quad::mpuInterrupt = true;
}