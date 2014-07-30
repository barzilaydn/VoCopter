/**********************************************************************************************
* Arduino PID Library - Version 2.0
* v1.0 by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
* v2.0 by Dan Barzilay <barzilaydn@gmail.com>
*
* This Library is licensed under a GPLv3 License
**********************************************************************************************/
#include <PID_v2.h>

/*Constructor (...)*********************************************************
*    The parameters specified here are those for for which we can't set up 
*    reliable defaults, so we need to have the user set them.
***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
double Kp, double Ki, double Kd, int ControllerDirection)
{
    
    myOutput = Output;
    myInput = Input;
    mySetpoint = Setpoint;
    inAuto = false;
    
    PID::SetOutputLimits(0, 255);				//default output limit corresponds to 
    //the arduino pwm limits

    sampleTime = 100;							//default Controller Sample Time is 0.1 seconds

    PID::SetControllerDirection(ControllerDirection);
    PID::SetTunings(Kp, Ki, Kd);

    lastTime = millis()-sampleTime;				
}


/* Compute() **********************************************************************
*     This, as they say, is where the magic happens.  this function should be called
*   every time "void loop()" executes.  the function will decide for itself whether a new
*   pid Output needs to be computed.  returns true when the output is computed,
*   false when nothing has been done.
**********************************************************************************/ 
bool PID::Compute() volatile
{
    if(!inAuto) return false;
    if(!masterAttached)
    {
        unsigned long now = millis();
        unsigned long timeChange = (now - lastTime);
    }
    if(timeChange>=sampleTime || masterAttached)
    {
        /*Compute all the working error variables*/
        double input = *myInput;
        double error = *mySetpoint - input;
        ITerm+= (ki * error);

        double dInput = (input - lastInput);

        /*Compute PID Output*/
        double output = kp * error + ITerm- kd * dInput;		
        
        /* Anti integral wind-up and output limiting*/
        if(output > outMax)
        {
            ITerm -= output – outMax;
            output = outMax;
        }
        else if(output < outMin)
        {
            ITerm += outMin – output;
            output = outMin;
        }

        *myOutput = output;

        /*Remember some variables for next time*/
        lastInput = input;
        lastTime = now;
        return true;
    }
    else return false;
}

/* int AttachToMaster() **********************************************************************
*  Activates interrupt mode instead of polling by connecting this instance to a PIDMaster.
*  Returns the current sampleTime for the master to use.
**********************************************************************************/ 
int PID::AttachToMaster()
{
    masterAttached = true; //Mark as attached for master.
    return sampleTime; //
}

/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted. 
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/ 
void PID::SetTunings(double Kp, double Ki, double Kd)
{
    if (Kp<0 || Ki<0 || Kd<0) return;

    dispKp = Kp; dispKi = Ki; dispKd = Kd;

    double sampleTimeInSec = ((double)sampleTime)/1000;  
    kp = Kp;
    ki = Ki * sampleTimeInSec;
    kd = Kd / sampleTimeInSec;

    if(controllerDirection ==REVERSE)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }
}

/* SetsampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed	
******************************************************************************/
void PID::SetsampleTime(int NewsampleTime)
{
    if (NewsampleTime > 0)
    {
        double ratio  = (double)NewsampleTime
        / (double)sampleTime;
        ki *= ratio;
        kd /= ratio;
        sampleTime = (unsigned long)NewsampleTime;
    }
}

/* SetOutputLimits(...)****************************************************
*     This function will be used far more often than SetInputLimits.  while
*  the input to the controller will generally be in the 0-1023 range (which is
*  the default already,)  the output will be a little different.  maybe they'll
*  be doing a time window and will need 0-8000 or something.  or maybe they'll
*  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
*  here.
**************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
    if(Min >= Max) return;

    outMin = Min;
    outMax = Max;

    if(inAuto)
    {
        /* Anti integral wind-up and output limiting*/
        if(*myOutput > outMax)
        {
            ITerm -= *myOutput – outMax;
            *myOutput = outMax;
        }
        else if(*myOutput < outMin)
        {
            ITerm += outMin – *myOutput;
            *myOutput = outMin;
        }
    }
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/ 
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/ 
void PID::Initialize()
{
    ITerm = *myOutput;
    lastInput = *myInput;
    if(ITerm > outMax) ITerm = outMax;
    else if(ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads 
* to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing.  This is called from the constructor.
******************************************************************************/
void PID::SetControllerDirection(int Direction)
{
    if(inAuto && Direction !=controllerDirection)
    {
        kp = (0 - kp);
        ki = (0 - ki);
        kd = (0 - kd);
    }   
    controllerDirection = Direction;
}

/* Status Funcions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened.  these
* functions query the internal state of the PID.  they're here for display 
* purposes.  this are the functions the PID Front-end uses for example
******************************************************************************/
double PID::GetKp(){ return  dispKp; }
double PID::GetKi(){ return  dispKi;}
double PID::GetKd(){ return  dispKd;}
int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}
int PID::GetDirection(){ return controllerDirection;}

