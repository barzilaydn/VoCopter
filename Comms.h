#ifndef Comms_h
#define Comms_h
/*-----------------
    Includes
  -----------------*/
#if ARDUINO < 100
    #include "WProgram.h"
#else
    #include "Arduino.h"
#endif

#include "Qconfig.h"

/// --- VARS --- ///
unsigned char cntrl_str[COMMAND_SIZE-1];
bool data_received = false;
bool clientStarted = false;
unsigned long last_recv;

/// --- RECEIVE --- ///
void receiveData()
{    
    // Clear params
    for (int c = 0; c < MAX_CNTRL_PARAMS; c++)    
        PARAMS[c] = 0;
    
    if(SERIAL_AVAILABLE() >= COMMAND_SIZE)
    {
        unsigned char recv_head = SERIAL_READ(); // Read first byte.
        
        // Wait till HEAD received
        while(recv_head != SERIAL_HEAD)
        {
            QDEBUG_PRINT(F("Head: "));
            QDEBUG_PRINTLN(recv_head);
            QDEBUG_PRINT(F("Expected: "));
            QDEBUG_PRINTLN(SERIAL_HEAD);
            
            if(SERIAL_AVAILABLE() >= COMMAND_SIZE)            
                recv_head = SERIAL_READ();            
            else            
                return;            
        }
                
        // Read up command:
        SERIAL_READ_BYTES(cntrl_str, COMMAND_SIZE - 1);
        
        // If matching "signature": (means that it is really a command)
        if(cntrl_str[COMMAND_SIZE-2] == SERIAL_TAIL)
        {
            for(int i = 0; i < (MAX_CNTRL_PARAMS + 1) * 4; i=i+4)
            {
                // Little-endian byte array to signed integer conversion
                int32_t st = (cntrl_str[i+3] << 24) | (cntrl_str[i+2] << 16) | (cntrl_str[i+1] << 8) | (cntrl_str[i]);
                
                if(i == 0)
                {
                    if(st >= 0 && st <= NUM_STATES-1)
                    {
                        STATE = st; 
                        QDEBUG_PRINT(F("Changing STATE to: "));
                        QDEBUG_PRINTLN(STATE);
                    }
                    else if (st == QRECV)
                    {
                        last_recv = uptime;
                        data_received = true;
                    }
                    else if (st == QSYN)
                    {
                        QDEBUG_PRINTLN(F("Client connected, starting transmitting status."));
                        clientStarted = true;
                        last_recv = uptime;
                        SERIAL_PRINTLN(QACK, 0, 0);
                    }
                    else
                    {
                        QDEBUG_PRINT(F("ST: "));
                        QDEBUG_PRINTLN(st);
                    }
                }
                else
                {
                    PARAMS[i/4 - 1] = st;
                }
            }
        }
        else
        {
            QDEBUG_PRINT(F("Tail: "));
            QDEBUG_PRINTLN(cntrl_str[COMMAND_SIZE-2]);
            QDEBUG_PRINT(F("Expected Tail: "));
            QDEBUG_PRINTLN(SERIAL_TAIL);
        }
        
        // Update time since inactive:
        zero_time = uptime;
    }
    else if(SERIAL_AVAILABLE() > 0)
    {
      /*QDEBUG_PRINT(F("Has: "));
        QDEBUG_PRINT(SERIAL_AVAILABLE());
        QDEBUG_PRINT(F(" ; Client started: "));
        QDEBUG_PRINTLN(clientStarted); */
    }
}

///  --- SEND ---  ///
void sendStatus()
{
    if(!clientStarted)
        last_recv = uptime;
    
    if((uptime - last_recv) / 1000 > 5) // Reset data_received every 5s.    
        data_received = true;
        
    if(STATE != CALIBRATE && data_received && clientStarted)
    {
        int* thrusts = VoCopter.GetMotors();
        // FORMAT: ' UP_TIME ; STATE ; YAW,PITCH,ROLL,TEMP,HEADING,ALTITUDE ; FRONT_LEFT,FRONT_RIGHT,BACK_LEFT,BACK_RIGHT ; BAT_LVL '
        SERIAL_PRINTLN(Q_STATUS, 13, 
                                    static_cast<int32_t>((uptime / 1000)),
                                    static_cast<int32_t>(STATE),
                                    static_cast<int32_t>(VoCopter.GetYawI()),
                                    static_cast<int32_t>(VoCopter.GetPitchI()),
                                    static_cast<int32_t>(VoCopter.GetRollI()),
                                    static_cast<int32_t>(VoCopter.GetTemp()),
                                    static_cast<int32_t>(VoCopter.GetHeading()),
                                    static_cast<int32_t>(VoCopter.GetAltitude()),
                                    static_cast<int32_t>(thrusts[0]),
                                    static_cast<int32_t>(thrusts[1]),
                                    static_cast<int32_t>(thrusts[2]),
                                    static_cast<int32_t>(thrusts[3]),
                                    static_cast<int32_t>(BatLvl)
        );
        data_received = false;
    }
}

/*------------------------------
    Serial command builder
  ------------------------------*/

void insertToCmdBuffer (unsigned char buffer[], int dataLen,  const unsigned char data[], int startingFrom)
{
    for(int i = 0; i < dataLen; i++)    
        if(startingFrom + i < COMMAND_SIZE) // Not overflowing
            buffer[startingFrom + i] = data[i];   
}

void SERIAL_PRINTLN(unsigned long cmd, int len, ...)
{
    //Declare a va_list macro and initialize it with va_start
    va_list argList;
    va_start(argList, len);
    
    if(clientStarted)
    {
        unsigned char buffer[COMMAND_SIZE];
        int count;
        int32_t param;
        
        //Send head bytes
        buffer[0] = SERIAL_HEAD;
        
        //Send CMD in Little-endian
        param = static_cast<int32_t>(cmd);
        insertToCmdBuffer(buffer, 4, reinterpret_cast<const unsigned char*>(&param), 1);
        
        //Send arguments in Little-endian
        for(count = 0; count < len; count++)
        {
            if(count < MAX_CNTRL_PARAMS)
            {
                param = va_arg(argList, int32_t);
                insertToCmdBuffer(buffer, 4, reinterpret_cast<const unsigned char*>(&param), 5 + 4 * count);
            }
        }
        
        //Fill up buffer
        for(; count < MAX_CNTRL_PARAMS; count++)
            insertToCmdBuffer(buffer, 4, SERIAL_EMPTY, 5 + 4 * count);
        
        //Send end bytes
        buffer[COMMAND_SIZE-1] = SERIAL_TAIL;
        
        // Make sure all previous data was sent:
        SERIAL_FLUSH();
        
        // Send the data:
        SERIAL_WRITE(buffer, COMMAND_SIZE);
    }
    else    
        QDEBUG_PRINTLN(F("Client not started, cannot send command."));
    
    va_end(argList);
}

#endif