// Quad.cpp
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

#if ARDUINO < 100
    #include "WProgram.h"
#else
    #include "Arduino.h"
#endif
#include "Quad.h"

//Constructor
Quad::Quad(const double aStep, const double aNoise, const int aLookBack, const int ST) :
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
                motors({ FRONT_LEFT, FRONT_RIGHT, BACK_RIGHT, BACK_LEFT }),
                thrust({ 0 })
{}

// Maps motors to pins
void Quad::SetupMotors()
{    
    QDEBUG_PRINTLN(F("DEBUG: Mapping motors to pins..."));
    for(int i = 0; i < 4; i++)    
        pinMode(motors[i], OUTPUT);    
}

// Start systems.
void Quad::Init(bool fromSleep) 
{
    time_since_start = millis();
    last_write = time_since_start;

    if(fromSleep == false)
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
 
    QDEBUG_PRINTLN(F("DEBUG: Starting FreeIMU..."));
    my3IMU.init(true);
    my3IMU.setTempCalib(1);   

    char str[128];
    sprintf(str, "DEBUG: FreeIMU library by %s, FREQ:%s, LIB_VERSION: %s, IMU: %s", FREEIMU_DEVELOPER, FREEIMU_FREQ, FREEIMU_LIB_VERSION, FREEIMU_ID);
    QDEBUG_PRINTLN(F(str));    
    //--PID--
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
    
    QDEBUG_PRINTLN(F("DEBUG: Init finished."));
}

bool Quad::Fly()
{
    if(tuning) Quad::CancelTune();
    
    //Update Inputs.
    Quad::UpdateIMU();
    
    if(PitchPID.GetMode() != 1)
        PitchPID.SetMode(AUTOMATIC);
    if(RollPID.GetMode() != 1)
        RollPID.SetMode(AUTOMATIC);
    if(YawPID.GetMode() != 1)
        YawPID.SetMode(AUTOMATIC);
        
    
    time_since_start = millis();
    if(time_since_start - last_write >= SAMPLE_TIME) // Only every SAMPLE_TIME msecs.
    {
        //Compute new output values    
        PitchPID.Compute();
        RollPID.Compute();
        YawPID.Compute();

        Quad::SetMotors();        
    }
}

void Quad::SetMotors(bool forceThrust)
{

    last_write = time_since_start;
    //Smoothly get to the baseThrust set point.
    if(forceThrust)
        baseThrust = baseThrust_S;
    else
        baseThrust += (baseThrust_S - baseThrust > 0) - (baseThrust_S - baseThrust < 0);

    //Calculate value for each motor.
    #ifdef X_CONFIG
        thrust[0] = (baseThrust != 0) ? constrain(baseThrust - Pitch_O + Roll_O + Yaw_O, 0, 255) : 0;
        thrust[1] = (baseThrust != 0) ? constrain(baseThrust - Pitch_O - Roll_O - Yaw_O, 0, 255) : 0;
        thrust[2] = (baseThrust != 0) ? constrain(baseThrust + Pitch_O - Roll_O + Yaw_O, 0, 255) : 0;
        thrust[3] = (baseThrust != 0) ? constrain(baseThrust + Pitch_O + Roll_O - Yaw_O, 0, 255) : 0;
    #else
        thrust[0] = (baseThrust != 0) ? constrain(baseThrust + Pitch_O + Yaw_O, 0, 255) : 0;
        thrust[1] = (baseThrust != 0) ? constrain(baseThrust - Roll_O  - Yaw_O, 0, 255) : 0;
        thrust[2] = (baseThrust != 0) ? constrain(baseThrust - Pitch_O + Yaw_O, 0, 255) : 0;
        thrust[3] = (baseThrust != 0) ? constrain(baseThrust + Roll_O  - Yaw_O, 0, 255) : 0;
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
        PitchPID.SetMode(0);
        RollPID.SetMode(0);
        YawPID.SetMode(0);   
        
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
        
        QDEBUG_PRINTLN(F("QDEBUG: Stopped."));

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
/*
    //Get Orientation.
    my3IMU.getYawPitchRoll(ypr);
    */
    #ifdef QDEBUG
    
        //Get temp.
        temperature = my3IMU.getBaroTemperature();
        QDEBUG_PRINT(F("DEBUG: Temperature is "));
        QDEBUG_PRINT(temperature);
        QDEBUG_PRINTLN(F("C degrees."));
            
        //Get altitude.
        altitude = val[10];
        QDEBUG_PRINT(F("DEBUG: Altitude is "));
        QDEBUG_PRINT(altitude);
        QDEBUG_PRINTLN(F("m."));    
        
        //Get heading.
        heading = val[9];
        QDEBUG_PRINT(F("DEBUG: Heading is "));
        QDEBUG_PRINT(heading);
        QDEBUG_PRINTLN(F(" degrees."));
        
    #endif
    
    Yaw_I   = ypr[0];
    Pitch_I = ypr[1];
    Roll_I  = ypr[2];
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
            SERIAL_PRINTLN();
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
                SERIAL_PRINTLN();
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
            SERIAL_PRINT("acc offset: ");
            SERIAL_PRINT(my3IMU.acc_off_x);
            SERIAL_PRINT(",");
            SERIAL_PRINT(my3IMU.acc_off_y);
            SERIAL_PRINT(",");
            SERIAL_PRINT(my3IMU.acc_off_z);
            SERIAL_PRINT("\n");

            SERIAL_PRINT("magn offset: ");
            SERIAL_PRINT(my3IMU.magn_off_x);
            SERIAL_PRINT(",");
            SERIAL_PRINT(my3IMU.magn_off_y);
            SERIAL_PRINT(",");
            SERIAL_PRINT(my3IMU.magn_off_z);
            SERIAL_PRINT("\n");

            SERIAL_PRINT("acc scale: ");
            SERIAL_PRINT(my3IMU.acc_scale_x);
            SERIAL_PRINT(",");
            SERIAL_PRINT(my3IMU.acc_scale_y);
            SERIAL_PRINT(",");
            SERIAL_PRINT(my3IMU.acc_scale_z);
            SERIAL_PRINT("\n");

            SERIAL_PRINT("magn scale: ");
            SERIAL_PRINT(my3IMU.magn_scale_x);
            SERIAL_PRINT(",");
            SERIAL_PRINT(my3IMU.magn_scale_y);
            SERIAL_PRINT(",");
            SERIAL_PRINT(my3IMU.magn_scale_z);
            SERIAL_PRINT("\n");
        }
    }
}

void Quad::Test(int* PARAMS)
{
    switch(PARAMS[0])
    {
        case 1: //Motors
            Quad::SetBaseThrust(PARAMS[1]);
            Quad::Fly();
            sprintf(str, "DEBUG: Yaw_O: %d, Pitch_O: %d, Roll_O: %d", Yaw_O, Pitch_O, Roll_O);
            QDEBUG_PRINTLN(F(str));
            break;
            
        case 2: //Motors_individual
            analogWrite(motors[0], constrain(PARAMS[1], 0, 255));
            analogWrite(motors[1], constrain(PARAMS[2], 0, 255));
            analogWrite(motors[2], constrain(PARAMS[3], 0, 255));
            analogWrite(motors[3], constrain(PARAMS[4], 0, 255));
            break;
        
        default: 
        case 0: //FreeIMU
            my3IMU.getRawValues(raw_values);
            sprintf(str, "DEBUG: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", raw_values[0], raw_values[1], raw_values[2], raw_values[3], raw_values[4], raw_values[5], raw_values[6], raw_values[7], raw_values[8], raw_values[9], raw_values[10]);
            QDEBUG_PRINTLN(F(str));
            my3IMU.getQ(q, val);
            sprintf(str, "DEBUG: Q: %f,%f,%f,%f", q[0], q[1], q[2], q[3]);
            QDEBUG_PRINTLN(F(str));
            my3IMU.getYawPitchRoll(ypr);
            sprintf(str, "DEBUG: Yaw: %d, Pitch: %d, Roll: %d", ypr[0], ypr[1], ypr[2]);
            QDEBUG_PRINTLN(F(str));
            break;        
    }
}

/*-----------------
    Getters / Setters
  -----------------*/
int*    Quad::GetThrust()         { return thrust; }

void    Quad::SetBaseThrust(int t){ baseThrust_S = map(constrain(t, 0, 100), 0, 100, 0, 255); }
int     Quad::GetBaseThrust()     { return baseThrust; }

float*  Quad::GetQ()              { return q; }

void    Quad::SetPitchS(double p) { Pitch_S = p; }
double  Quad::GetPitchI()         { return Pitch_I; }
double  Quad::GetPitchO()         { return Pitch_O; }

void    Quad::SetRollS(double p)  { Roll_S = p; }
double  Quad::GetRollI()          { return Roll_I; }
double  Quad::GetRollO()          { return Roll_O; }

void    Quad::SetYawS(double p)   { Yaw_S = p; }
double  Quad::GetYawI()           { return Yaw_I; }
double  Quad::GetYawO()           { return Yaw_O; } 

int     Quad::GetTemp()           { return (int)temperature; }
float   Quad::GetAltitude()       { return altitude; }
float   Quad::GetHeading()        { return heading; }

int*    Quad::GetMotors()         { return thrust; }