/*-----------------
    Serial Port
  -----------------*/

#define SERIAL_PORT Serial3
#define DEBUG_SERIAL_PORT Serial
#define RX_PIN 7

/*-------------------------
    Quad Configuration 
  -------------------------*/

#define SLEEP_BAT 10 // Percent
  
#define X_CONFIG
//#define PLUS_CONFIG

// Motor pins:
#define FRONT_LEFT  5
#define FRONT_RIGHT 6
#define BACK_RIGHT  9
#define BACK_LEFT   10

#define GO_TO_SLEEP_TIME 180 // Seconds

#define MAX_CNTRL_PARAMS 10
#define COMMAND_STAT_SIZE 128 // Calculated: 19 ("VCSTAT;;;,,,,,,,,,;") + 7 (Reasonable INT digits) * 14 (PARAM count) ceiled to 2^X

#define COMMAND_CNTRL_SIZE COMMAND_STAT_SIZE

/*------------------------------
    PID controllers settings
  ------------------------------*/

#define OUTPUT_STEP 20
#define NOISE 0.5
#define LOOK_BACK_SEC 1
#define SAMPLE_TIME 20

/*--------------
    Debugging
  --------------*/

// Enable debugging:
#define QDEBUG

/**********!!!!! DON'T EDIT BELOW THIS LINE !!!!!**********/



/*-----------------
    Serial
  -----------------*/
#define SERIAL_WRITE            SERIAL_PORT.write
#define SERIAL_PRINT            SERIAL_PORT.print
#define SERIAL_PRINTLN          SERIAL_PORT.println
#define SERIAL_AVAILABLE        SERIAL_PORT.available
#define SERIAL_READ             SERIAL_PORT.read
#define SERIAL_READ_BYTES       SERIAL_PORT.readBytes
#define SERIAL_READ_BYTES_UNTIL SERIAL_PORT.readStringUntil
#define SERIAL_FLUSH            SERIAL_PORT.flush
#define SERIAL_BEGIN            SERIAL_PORT.begin

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