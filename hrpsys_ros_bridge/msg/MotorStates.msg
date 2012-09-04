# This is a message that holds data to describe the state of a set of motors. 
#
# The state of each motor is defined by:
#  * the temperatur of the motor (Celsius),
#
# Each motor is uniquely identified by its name
# The header specifies the time at which the motor states were recorded. All the motor states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the motor state. 
# The goal is to make each of the fields optional. When e.g. your motors have no
# effort associated with them, you can leave the effort array empty. 
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the motor name with the correct states.

# hrpsys/idl/RobotHardwareService.idl
#
#   interface RobotHardwareService
#   {
#     const unsigned long CALIB_STATE_MASK = 0x00000001;
#     const unsigned long CALIB_STATE_SHIFT = 0;
#     const unsigned long SERVO_STATE_MASK = 0x00000002;
#     const unsigned long SERVO_STATE_SHIFT = 1;
#     const unsigned long POWER_STATE_MASK = 0x00000004;
#     const unsigned long POWER_STATE_SHIFT = 2;
#     const unsigned long SERVO_ALARM_MASK = 0x0007fff8;
#     const unsigned long SERVO_ALARM_SHIFT = 3;
#     const unsigned long DRIVER_TEMP_MASK = 0xff000000;
#     const unsigned long DRIVER_TEMP_SHIFT = 24;

Header header

string[] name
bool[] calib_state
bool[] servo_state
bool[] power_state
int32[] servo_alarm
int16[] driver_temp
float64[] temperature
std_msgs/Float64MultiArray extra_data

