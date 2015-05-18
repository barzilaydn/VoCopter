// Quad.cpp
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

#if ARDUINO < 100
    #include "WProgram.h"
#else
    #include "Arduino.h"
#endif
#include "Quad.h"

//Constructor
Quad::Quad(const double aStep, const double aNoise, const int aLookBack, const int ST) :
                aTuneStep(aStep),
                aTuneNoise(aNoise),
                aTuneLookBack(aLookBack),
                sampleTime(ST),
                
                EPitchPID(&EPitch_I, &EPitch_O, &EPitch_S, 1.0, 0.0, 0.0, DIRECT),
                ERollPID(&ERoll_I, &ERoll_O, &ERoll_S, 1.0, 0.0, 0.0, DIRECT),
                EYawPID(&EYaw_I, &EYaw_O, &EYaw_S, 1.0, 0.0, 0.0, DIRECT),
                
                BPitchRatePID(&BPitchRate_I, &BPitchRate_O, &BPitchRate_S, 1.0, 0.0, 0.0, DIRECT),
                BPitchRateTune(&BPitchRate_I, &BPitchRate_O),
                
                BRollRatePID(&BRollRate_I, &BRollRate_O, &BRollRate_S, 1.0, 0.0, 0.0, DIRECT),
                BRollRateTune(&BRollRate_I , &BRollRate_O),
                
                BYawRatePID(&BYawRate_I, &BYawRate_O, &BYawRate_S, 1.0, 0.0, 0.0, DIRECT),
                BYawRateTune(&BYawRate_I  , &BYawRate_O)
{}

// Maps motors to pins
void Quad::SetupMotors()
{    
    QDEBUG_PRINTLN(F("Mapping motors to pins..."));
    for(int i = 0; i < 4; i++)    
        pinMode(motors[i], OUTPUT);    
}

// Start systems.
void Quad::Init(bool fromSleep) 
{
    if(!fromSleep)
        Quad::SetupMotors();
    
    //--MOTORS--
    //Start motors for a sec to show activity.
    baseThrust = 60;
    Quad::SetMotors(true);
    delay(1000);
    baseThrust = 0;
    Quad::SetMotors(true);
    
    baseThrust_S = 0;
    
    //--IMU--
    Wire.begin(); //Start I2C Master mode. Later on FreeIMU switches to 400kHz :)
 
    QDEBUG_PRINTLN(F("Starting FreeIMU..."));
    my3IMU.init(true);
    my3IMU.setTempCalib(1);   
    starting_altitude = 0;   
    
    delay(1000);
    my3IMU.RESET_Q();
    
    //--PID--
    QDEBUG_PRINTLN(F("Configuring PIDs and Tuners..."));
    //Prepare the PID controllers:
    EPitchPID.SetSampleTime(sampleTime);
    ERollPID.SetSampleTime(sampleTime);
    EYawPID.SetSampleTime(sampleTime);
    BPitchRatePID.SetSampleTime((int)(sampleTime / 2));
    BRollRatePID.SetSampleTime((int)(sampleTime / 2));
    BYawRatePID.SetSampleTime((int)(sampleTime / 2));
    
    //Prepare the PID controller tuners:
    BPitchRateTune.SetControlType(PID_ATune::ZIEGLER_NICHOLS_PID);
    BRollRateTune.SetControlType(PID_ATune::ZIEGLER_NICHOLS_PID);
    BYawRateTune.SetControlType(PID_ATune::ZIEGLER_NICHOLS_PID);
    tuning = false;
    
    QDEBUG_PRINTLN(F("Init finished."));
}

void Quad::Fly()
{
    if(tuning) Quad::CancelTune();
    
    // Update Inputs
    Quad::UpdateIMU();
    
    // Enable PIDs:
    if(EPitchPID.GetMode() != 1)
        EPitchPID.SetMode(AUTOMATIC);
    if(ERollPID.GetMode() != 1)
        ERollPID.SetMode(AUTOMATIC);
    if(EYawPID.GetMode() != 1)
        EYawPID.SetMode(AUTOMATIC);
    if(BPitchRatePID.GetMode() != 1)
        BPitchRatePID.SetMode(AUTOMATIC);
    if(BRollRatePID.GetMode() != 1)
        BRollRatePID.SetMode(AUTOMATIC);
    if(BYawRatePID.GetMode() != 1)
        BYawRatePID.SetMode(AUTOMATIC);
    
    //Compute new output values
    ERollPID.Compute();
    EPitchPID.Compute();
    EYawPID.Compute();
    
/*     float rates_earth[3];
    float rates_body[3];
    
    rates_earth[0] += ERoll_O;
    rates_earth[1] += EPitch_O;
    rates_earth[2] += EYaw_O;
    
    frame_conversion_ef_to_bf(rates_earth, rates_body);
    
    BRollRate_S = rates_body[0];
    BPitchRate_S = rates_body[1];
    BYawRate_S = rates_body[2]; */
    
    BRollRate_S += ERoll_O;
    BPitchRate_S += EPitch_O;
    BYawRate_S += EYaw_O;
    
    BRollRatePID.Compute();
    BPitchRatePID.Compute();
    BYawRatePID.Compute();
    
    Quad::SetMotors();    
}

void Quad::SetMotors(bool forceThrust)
{
    //Smoothly get to the baseThrust set point.
    if(forceThrust)
        baseThrust = baseThrust_S;
    else
        baseThrust += (baseThrust_S - baseThrust > 0) - (baseThrust_S - baseThrust < 0);
    
    Yaw = BYawRate_O;
    Pitch = BPitchRate_O;
    Roll = BRollRate_O;
    
    //Calculate value for each motor.
    #ifdef X_CONFIG
        thrust[0] = (baseThrust != 0) ? constrain(baseThrust - Pitch + Roll + Yaw, 0, 255) : 0;
        thrust[1] = (baseThrust != 0) ? constrain(baseThrust - Pitch - Roll - Yaw, 0, 255) : 0;
        thrust[2] = (baseThrust != 0) ? constrain(baseThrust + Pitch - Roll + Yaw, 0, 255) : 0;
        thrust[3] = (baseThrust != 0) ? constrain(baseThrust + Pitch + Roll - Yaw, 0, 255) : 0;
    #else
        thrust[0] = (baseThrust != 0) ? constrain(baseThrust + Pitch + Yaw, 0, 255) : 0;
        thrust[1] = (baseThrust != 0) ? constrain(baseThrust - Roll  - Yaw, 0, 255) : 0;
        thrust[2] = (baseThrust != 0) ? constrain(baseThrust - Pitch + Yaw, 0, 255) : 0;
        thrust[3] = (baseThrust != 0) ? constrain(baseThrust + Roll  - Yaw, 0, 255) : 0;
    #endif
    
    //Assign to motors.
    analogWrite(motors[0], thrust[0]);
    analogWrite(motors[1], thrust[1]);
    analogWrite(motors[2], thrust[2]);
    analogWrite(motors[3], thrust[3]);    
}

bool Quad::Stop()
{
    if(tuning) Quad::CancelTune();
    
    //Update Inputs.
    Quad::UpdateIMU();
    
    
    //Land.
    Quad::SetBaseThrust(0);
    Quad::SetPitchS(0);
    Quad::SetRollS(0);
    Quad::SetYawS(0);
    
    Quad::SetMotors();

    if(baseThrust == 0)
    {
        //Turn off PID controllers.
        BPitchRatePID.SetMode(0);
        BRollRatePID.SetMode(0);
        BYawRatePID.SetMode(0);   
        EPitchPID.SetMode(0);
        ERollPID.SetMode(0);
        EYawPID.SetMode(0);   
        
        return true; //Finished.
    }
    
    return false;
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
            return &BPitchRatePID;
        case ROLL:
            return &BRollRatePID;
        default:
        case YAW:
            return &BYawRatePID;
    }
}

PID_ATune* Quad::ChooseTuner(int axis)
{
    switch(axis)
    {
        case PITCH:
            return &BPitchRateTune;
        case ROLL:
            return &BRollRateTune;
        default:
        case YAW:
            return &BYawRateTune;
    }
}

/**
* Turns the AutoTuner on/off.
* @param axis   : The PID controller.
* @param tuner : The AutoTuner to use.
* @return Returns whether the AutoTuner finished.
*/
bool Quad::TunePID(int axis)
{    
    PID* pid = Quad::ChoosePID(axis);
    PID_ATune* tuner = Quad::ChooseTuner(axis);
    tuning = true;
    
    //Update Inputs.
    Quad::UpdateIMU();
    baseThrust = 70; //TODO: verify good value!
    
    byte val = ((*tuner).Runtime());
    Quad::SetMotors(true);
    
    if (val != 0) //Finished tuning
    {
        tuning = false;

        // Set the tuning parameters
        (*pid).SetTunings((*tuner).GetKp(), (*tuner).GetKi(), (*tuner).GetKd());
        Quad::AutoTuneHelper(axis, false);
        
        QDEBUG_PRINTLN(F("Stopped."));

        return true;
    }
    return false;
}

/**
* Turns the AutoTuner on/off.
* @param axis   : The PID controller.
* @param tuner : The AutoTuner to turn on/off.
*/
void Quad::changeAutoTune(int axis)
{
    //PID* pid = Quad::ChoosePID(axis);
    PID_ATune* tuner = Quad::ChooseTuner(axis);
    
    if (!tuning)
    {
        // Set the outputs to their default values.
        BPitchRate_O = 0;
        BRollRate_O = 0;
        BYawRate_O = 0;
        
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
        Quad::AutoTuneHelper(axis, false);
        tuning = false;
    }
}

/**
* Restores/Saves the PID controller mode
* @param axis   : The PID controller.
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

void Quad::UpdateIMU()
{
    my3IMU.getQ(q, val);
    
    // Get orientation from FreeIMU:
        // Earth-ref:
    // my3IMU.getYawPitchRollRadAHRS(ypr, q); // @TODO: check if need to replace with euler angles.
    my3IMU.getEulerRad(ypr, q);
    // Body-ref rates:
    brpy[0] = val[3]; // Roll rate.
    brpy[1] = val[4]; // Pitch rate.
    brpy[2] = val[5]; // Yaw rate.
    
    // Convert to degrees.
    arr3_rad_to_deg(ypr);
    
    //Get temp.
    temperature = my3IMU.getBaroTemperature();
    
    //Get altitude.
    altitude = val[10];
    if (baseThrust_S == 0)
        starting_altitude = altitude;
    
    //Get heading.
    heading = val[9];

    EYaw_I       = ypr[0];    
    // Fix Yaw:
    if (EYaw_I > 180.)
        EYaw_I -= 360.;
    else if (EYaw_I < -180.)
        EYaw_I += 360.;
    
    EPitch_I     = ypr[1];
    ERoll_I      = ypr[2];    
    BRollRate_I  = brpy[0];
    BPitchRate_I = brpy[1];
    BYawRate_I   = brpy[2];
}

char Quad::serial_busy_wait() {
    while(!SERIAL_AVAILABLE()); // do nothing until ready
    return SERIAL_READ();
}

void Quad::writeArr(void * varr, uint8_t arr_length, uint8_t type_bytes) {
  byte * arr = (byte*) varr;
  for(uint8_t i=0; i<arr_length; i++) {
    writeVar(&arr[i * type_bytes], type_bytes);
  }
}


// thanks to Francesco Ferrara and the Simplo project for the following code!
void Quad::writeVar(void * val, uint8_t type_bytes) {
  byte * addr=(byte *)(val);
  for(uint8_t i=0; i<type_bytes; i++) { 
    SERIAL_WRITE(addr[i]);
  }
}

void Quad::Calibrate(int cmd)
{
    //QDEBUG_PRINT(F("CALIB CMD:"));
    //QDEBUG_PRINTLN((char)cmd);
    
    switch(cmd)
    {
        case 1: //Send data for calibration
        {
            SERIAL_OUT("\n");
            Quad::serial_busy_wait();
            
            QDEBUG_PRINTLN(F("Got ack"));

            while(!SERIAL_AVAILABLE()) {
                #if HAS_ITG3200()
                    my3IMU.acc.readAccel(&raw_values[0], &raw_values[1], &raw_values[2]);
                    my3IMU.gyro.readGyroRaw(&raw_values[3], &raw_values[4], &raw_values[5]);
                    Quad::writeArr(raw_values, 6, sizeof(int)); // writes accelerometer, gyro values & mag if 9150
                #elif HAS_MPU9150()  || HAS_MPU9250()
                    my3IMU.getRawValues(raw_values);
                    Quad::writeArr(raw_values, 9, sizeof(int)); // writes accelerometer, gyro values & mag if 9150
                #elif HAS_MPU6050() || HAS_MPU6000()   // MPU6050
                    //my3IMU.accgyro.getMotion6(&raw_values[0], &raw_values[1], &raw_values[2], &raw_values[3], &raw_values[4], &raw_values[5]);
                    my3IMU.getRawValues(raw_values);
                    Quad::writeArr(raw_values, 6, sizeof(int)); // writes accelerometer, gyro values & mag if 9150
                #elif HAS_ALTIMU10()
                    my3IMU.getRawValues(raw_values);
                    Quad::writeArr(raw_values, 9, sizeof(int)); // writes accelerometer, gyro values & mag of Altimu 10        
                #endif
                    //Quad::writeArr(raw_values, 6, sizeof(int)); // writes accelerometer, gyro values & mag if 9150

                #if IS_9DOM() && (!HAS_MPU9150()  && !HAS_MPU9250() && !HAS_ALTIMU10())
                    my3IMU.magn.getValues(&raw_values[0], &raw_values[1], &raw_values[2]);
                    Quad::writeArr(raw_values, 3, sizeof(int));
                #endif
                SERIAL_OUT("\n");
            }
            
            QDEBUG_PRINTLN(F("Finished sending"));
            char r = SERIAL_READ();
            QDEBUG_PRINTLN(r);
            break;
        }
        case 2: // Save new calibration to EEPROM and load it
        {
            const uint8_t eepromsize = sizeof(float) * 6 + sizeof(int) * 6;
            while(SERIAL_AVAILABLE() < eepromsize) ; // wait until all calibration data are received
            
            // Write the received calibration data to EEPROM.
            EEPROM.write(FREEIMU_EEPROM_BASE, FREEIMU_EEPROM_SIGNATURE);
            for(uint8_t i = 1; i<(eepromsize + 1); i++)
                EEPROM.write(FREEIMU_EEPROM_BASE + i, (char) SERIAL_READ());
            
            my3IMU.calLoad(); // reload calibration
            // toggle LED after calibration store.
            digitalWrite(13, HIGH);
            delay(1000);
            digitalWrite(13, LOW);
            
            break;
        }
        case 3: // Clear saved calibration
        {
            EEPROM.write(FREEIMU_EEPROM_BASE, 0); // reset signature
            my3IMU.calLoad(); // reload calibration
            
            break;
        }
        case 4: // Check calibration data
        {
            SERIAL_OUT("acc offset: ");
            SERIAL_OUT(my3IMU.acc_off_x);
            SERIAL_OUT(",");
            SERIAL_OUT(my3IMU.acc_off_y);
            SERIAL_OUT(",");
            SERIAL_OUT(my3IMU.acc_off_z);
            SERIAL_OUT("\n");

            SERIAL_OUT("magn offset: ");
            SERIAL_OUT(my3IMU.magn_off_x);
            SERIAL_OUT(",");
            SERIAL_OUT(my3IMU.magn_off_y);
            SERIAL_OUT(",");
            SERIAL_OUT(my3IMU.magn_off_z);
            SERIAL_OUT("\n");

            SERIAL_OUT("acc scale: ");
            SERIAL_OUT(my3IMU.acc_scale_x);
            SERIAL_OUT(",");
            SERIAL_OUT(my3IMU.acc_scale_y);
            SERIAL_OUT(",");
            SERIAL_OUT(my3IMU.acc_scale_z);
            SERIAL_OUT("\n");

            SERIAL_OUT("magn scale: ");
            SERIAL_OUT(my3IMU.magn_scale_x);
            SERIAL_OUT(",");
            SERIAL_OUT(my3IMU.magn_scale_y);
            SERIAL_OUT(",");
            SERIAL_OUT(my3IMU.magn_scale_z);
            SERIAL_OUT("\n");
        }
    }
}

void Quad::Test(int32_t* PARAMS)
{
    switch(PARAMS[0])
    {
        case Q_TEST_PID: //PID
            Quad::SetBaseThrust(constrain(PARAMS[1], 0, 255));
            Quad::Fly();
            SERIAL_PRINTLN(Q_TEST, 4, Q_TEST_PID,
                                      (int32_t)EYaw_O,
                                      (int32_t)EPitch_O,
                                      (int32_t)ERoll_O
            );
            break;
            
        case Q_TEST_MOTORS: //Motors_individual
            thrust[0] = constrain(PARAMS[1], 0, 255);
            thrust[1] = constrain(PARAMS[1], 0, 255);
            thrust[2] = constrain(PARAMS[1], 0, 255);
            thrust[3] = constrain(PARAMS[1], 0, 255);
            
            analogWrite(motors[0], constrain(PARAMS[1], 0, 255));
            analogWrite(motors[1], constrain(PARAMS[2], 0, 255));
            analogWrite(motors[2], constrain(PARAMS[3], 0, 255));
            analogWrite(motors[3], constrain(PARAMS[4], 0, 255));
            break;
        
        default: 
        case Q_TEST_IMU: //FreeIMU
            my3IMU.getRawValues(raw_values);
            my3IMU.getQ(q, val);
            SERIAL_PRINTLN(Q_TEST, 12, Q_TEST_IMU,
                                       (int32_t)raw_values[0],
                                       (int32_t)raw_values[1],
                                       (int32_t)raw_values[2],
                                       (int32_t)raw_values[3],
                                       (int32_t)raw_values[4],
                                       (int32_t)raw_values[5],
                                       (int32_t)raw_values[6],
                                       (int32_t)raw_values[7],
                                       (int32_t)raw_values[8],
                                       (int32_t)raw_values[9],
                                       (int32_t)val[11]
            );
            
            break;        
    }
}

/**
* Convert millivolts to battery level in %. (Calibrated for LiPo 3.7v 300mah)
* @param mV : The voltage in millivolts.
* @return Returns battery level in %.
*/
int Quad::mVtoL(double mV)
{
    return static_cast<int>(map(constrain(mV, 3000, 3600), 3000, 3600, 0, 100));
}

/**
* Read the battery level from an analog pin
* @param pin : The number of the analog pin to read from.
* @return Returns the battery level measured on the pin.
*/
int Quad::UpdateBatLevel(int pin)
{
    int32_t x = analogRead(pin);
    return mVtoL((178*x*x + 2688757565 - 1184375 * x) / 372346);
}

/*-----------------
    Getters / Setters
  -----------------*/
void    Quad::SetBaseThrust(int t){ baseThrust_S = map(constrain(t, 0, 100), 0, 100, 0, 255); }
int     Quad::GetBaseThrust()     { return baseThrust; }

float*  Quad::GetQ()              { return q; }

void    Quad::SetPitchS(double p) { EPitch_S = p; }
double  Quad::GetPitchI()         { return EPitch_I; }
double  Quad::GetPitchO()         { return EPitch_O; }

void    Quad::SetRollS(double p)  { ERoll_S = p; }
double  Quad::GetRollI()          { return ERoll_I; }
double  Quad::GetRollO()          { return ERoll_O; }

void    Quad::SetYawS(double p)   { EYaw_S = p; }
double  Quad::GetYawI()           { return EYaw_I; }
double  Quad::GetYawO()           { return EYaw_O; } 

int     Quad::GetTemp()           { return (int)temperature; }
float   Quad::GetAltitude()       { return 100 * (altitude - starting_altitude); } // Relative altitude in cm.
float   Quad::GetHeading()        { return heading; }

int*    Quad::GetMotors()         { return thrust; }

/*-----------------
    Help functions
  -----------------*/
/**
 * Converts a 3 elements array arr of angles expressed in radians into degrees
*/
void Quad::arr3_rad_to_deg(float * arr) {
  arr[0] *= 180/M_PI;
  arr[1] *= 180/M_PI;
  arr[2] *= 180/M_PI;
}

void Quad::frame_conversion_ef_to_bf(float * ef, float * bf)
{
    //TODO
}