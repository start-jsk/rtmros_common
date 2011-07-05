#!/bin/sh

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`rospack find RS003`/lib
cd `rospack find RS003`/launch/
`rospack find openhrp3`/bin/openhrp-controller-bridge \
--server-name MotorControlFactory \
--module `rospack find RS003`/lib/MotorControl.so \
--out-port WheelAngle:MAIN_WHEEL1,MAIN_WHEEL2:JOINT_VALUE \
--out-port ceiling_camera:0:COLOR_IMAGE:0.1 \
--out-port top_urg:TOP_URG:RANGE_SENSOR \
--in-port WheelTorque:MAIN_WHEEL1,MAIN_WHEEL2:JOINT_TORQUE \
--out-port position:BODY:ABS_TRANSFORM \
--connection WheelAngle:MotorControl0:InCurrentWheelAngle \
--connection WheelTorque:MotorControl0:Torque \
--connection position:MotorControl0:InSimulatedPosition \
--connection ceiling_camera:SimViewCam0:in \
--connection top_urg:SimRangeUrg0:in \
--robot-name RefHw \



