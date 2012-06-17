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

Header header

string[] name
float64[] temperature
