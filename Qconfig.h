/*-----------------
    Serial Port
  -----------------*/

#define SERIAL_PORT Serial3
#define RX_PIN 11
#define PIN_RX_PIN PIN_11

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

#define GO_TO_SLEEP_TIME 30 // Seconds

#define MAX_CNTRL_PARAMS 10
#define COMMAND_STAT_SIZE 84

#define COMMAND_CNTRL_SIZE (11 * MAX_CNTRL_PARAMS + 7)

/*------------------------------
    PID controllers settings
  ------------------------------*/

#define OUTPUT_STEP 20
#define NOISE 0.5
#define LOOK_BACK_SEC 1
#define SAMPLE_TIME 10

/*--------------
    Debugging
  --------------*/

// Enable debugging:
//#define QDEBUG

/**********!!!!! DON'T EDIT BELOW THIS LINE !!!!!**********/



/*-----------------
    Serial
  -----------------*/
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
    #define QDEBUG_PRINT(x) SERIAL_PRINT(x)
    #define QDEBUG_PRINTF(x, y) SERIAL_PRINT(x, y)
    #define QDEBUG_PRINTLN(x) SERIAL_PRINTLN(x)
    #define QDEBUG_PRINTLNF(x, y) SERIAL_PRINTLN(x, y)
#else
    #define QDEBUG_PRINT(x)
    #define QDEBUG_PRINTF(x, y)
    #define QDEBUG_PRINTLN(x)
    #define QDEBUG_PRINTLNF(x, y)
#endif