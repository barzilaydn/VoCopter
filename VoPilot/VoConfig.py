# Define copter constrains:
MAX_YAW          = 10
MAX_PITCH        = 90
MAX_ROLL         = 90

# Define the communication parameters:
DEFAULT_IP       = "10.10.100.254"
PORT             = 8899
MAX_CNTRL_PARAMS = 14

CMD_HEAD         = '\xda\x0b' # 55819 # {0xDA, 0x0B}
CMD_TAIL         = '\xb0\xad' # 45229 # {0xB0, 0xAD}
COMMAND_SIZE     = 8 + MAX_CNTRL_PARAMS*4

# Joystick:
JOYSTICK_DEADBAND= 0.4

# Define FEEDBACK COMMANDS:
Q_STATUS         = 1
Q_ALERT          = 2
Q_SLEEPING       = 3
Q_TEST           = 4

# Define ALERT TYPES:
Q_LOW_BAT        = 1
Q_TIME_OUT       = 2
Q_WOKE_UP        = 3

# Define CONTROL COMMANDS:
NUM_OF_STATES    = 7
SLEEP            = 0
FLY              = 1
CALIBRATE        = 2
TUNE             = 3
MOVE             = 4
TEST             = 5
SETTINGS         = 6

# Define Tests:
Q_TEST_IMU       = 0
Q_TEST_PID       = 1
Q_TEST_MOTORS    = 2

# Define UI:
QOffset          = [0, 0, 0, 0]