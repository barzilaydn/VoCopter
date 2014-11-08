//Serial port
#define SERIAL_PORT Serial
#define RX_PIN 11
#define PIN_RX_PIN PIN_11

//Quad configuration
#define X_CONFIG
//#define PLUS_CONFIG

//PID controllers settings
#define OUTPUT_STEP 20
#define NOISE 0.5
#define LOOK_BACK_SEC 1
#define SAMPLE_TIME 10

//Enable debugging
//#define QDEBUG

/**********!!!!! DON'T EDIT BELOW THIS LINE !!!!!**********/



/*-----------------
    Serial
  -----------------*/
#define SERIAL_PRINT      SERIAL_PORT.print
#define SERIAL_PRINTLN    SERIAL_PORT.println
#define SERIAL_AVAILABLE  SERIAL_PORT.available
#define SERIAL_READ       SERIAL_PORT.read
#define SERIAL_FLUSH      SERIAL_PORT.flush
#define SERIAL_BEGIN      SERIAL_PORT.begin

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