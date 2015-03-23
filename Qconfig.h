#ifndef QCONFIG
#define QCONFIG

/*-----------------
    Serial Port
  -----------------*/

#define SERIAL_PORT Serial3
#define DEBUG_SERIAL_PORT Serial
#define RX_PIN 7

/*-------------------------
    Quad Configuration 
  -------------------------*/

#define SLEEP_BAT 10 // in Percent
  
#define X_CONFIG
//#define PLUS_CONFIG

// Motor pins:
#define FRONT_LEFT  5
#define FRONT_RIGHT 6
#define BACK_RIGHT  9
#define BACK_LEFT   10

#define GO_TO_SLEEP_TIME 180 // Seconds


/*------------------------------
    PID controllers settings
  ------------------------------*/

#define OUTPUT_STEP 20
#define NOISE 0.5
#define LOOK_BACK_SEC 1
#define SAMPLE_TIME 100

/*--------------
    Debugging
  --------------*/

// Enable debugging:
#define QDEBUG

/**********!!!!! DON'T EDIT BELOW THIS LINE !!!!!**********/

#define MAX_CNTRL_PARAMS 14
#define COMMAND_SIZE 8+MAX_CNTRL_PARAMS*4 // Calculated: 2 (HEADER BYTES = 0xDA0B) + 4 (CMD | int_32) + [PARAM count] * 4 (int_32 size) + 2 (END BYTES = 0xB0AD)

/*-----------------
    Quad States
  -----------------*/
#define NUM_STATES 6

#define SLEEP     0
#define FLY       1
#define CALIBRATE 2
#define TUNE      3
#define MOVE      4
#define TEST      5

/*-----------------
    FEEDBACK COMMANDS
  -----------------*/
  
#define Q_STATUS    1
#define Q_ALERT     2
#define Q_SLEEPING  3
#define Q_TEST      4

/*-----------------
    ALERT reasons
  -----------------*/
  
#define Q_LOW_BAT   1
#define Q_TIME_OUT  2
#define Q_WOKE_UP   3

/*-----------------
    Serial
  -----------------*/
#define SERIAL_WRITE            SERIAL_PORT.write
#define SERIAL_PRINT            SERIAL_PORT.print
#define SERIAL_AVAILABLE        SERIAL_PORT.available
#define SERIAL_READ             SERIAL_PORT.read
#define SERIAL_READ_BYTES       SERIAL_PORT.readBytes
#define SERIAL_READ_BYTES_UNTIL SERIAL_PORT.readStringUntil
#define SERIAL_FLUSH            SERIAL_PORT.flush
#define SERIAL_BEGIN            SERIAL_PORT.begin

const unsigned char SERIAL_HEAD[] = {0xDA, 0x0B};
const unsigned char SERIAL_TAIL[] = {0xB0, 0xAD};
const unsigned char SERIAL_EMPTY[] = {0x00, 0x00, 0x00, 0x00};
void SERIAL_PRINTLN(uint32_t, int, ...);

/*-----------------
    Debug
  -----------------*/
#ifdef QDEBUG
    #define QDEBUG_BEGIN(x) DEBUG_SERIAL_PORT.begin(x)
    #define QDEBUG_PRINT(x) DEBUG_SERIAL_PORT.print(x)
    #define QDEBUG_PRINTF(x, y) DEBUG_SERIAL_PORT.printf(x, y)
    #define QDEBUG_PRINTLN(x) DEBUG_SERIAL_PORT.println(x)
    #define QDEBUG_PRINTLNF(x, y) DEBUG_SERIAL_PORT.printlnf(x, y)
#else
    #define QDEBUG_PRINT(x)
    #define QDEBUG_PRINTF(x, y)
    #define QDEBUG_PRINTLN(x)
    #define QDEBUG_PRINTLNF(x, y)
#endif

#endif