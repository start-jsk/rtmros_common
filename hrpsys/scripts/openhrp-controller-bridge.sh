#!/bin/bash

if [ $# -lt 2 ] ; then
    echo "Usage: $0 [ROBOT_NAME] [MODEL_FILE]";
    exit 1;
fi
echo "Stating $1 with /tmp/$1$$.conf"
cat <<EOF > /tmp/$1$$.conf
model: file://$2
dt: 0.005
EOF

`rospack find openhrp3`/bin/openhrp-controller-bridge --server-name $1 \
    --periodic-rate $1\(Robot\)0:1.0 \
    --periodic-rate HGcontroller0:1.0 \
    --in-port qRef:JOINT_VALUE \
    --in-port dqRef:JOINT_VELOCITY \
    --in-port ddqRef:JOINT_ACCELERATION \
    --out-port q:JOINT_VALUE \
    --out-port pr:WAIST:ABS_TRANSFORM \
    --out-port left-eye:0:COLOR_IMAGE:0.1 \
    --out-port right-eye:1:COLOR_IMAGE:0.1 \
    --out-port image:0:COLOR_IMAGE:0.1 \
    --connection qRef:HGcontroller0:qOut \
    --connection dqRef:HGcontroller0:dqOut \
    --connection ddqRef:HGcontroller0:ddqOut \
    -o "corba.nameservers:localhost:2809" \
    -o "naming.formats:%n.rtc" \
    -o "logger.enable:YES" \
    -o "logger.log_level:NORMAL" \
    -o "logger.file_name:/tmp/$1rtc%p.log" \
    -o "exec_cxt.periodic.rate:1000" \
    -o "manager.shutdown_onrtcs:NO" \
    -o "manager.is_master:YES" \
    -o "manager.modules.load_path:`rospack find hrpsys`/lib" \
    -o "manager.modules.preload:HGcontroller.so" \
    -o "manager.components.precreate:HGcontroller" \
    -o "exec_cxt.periodic.type:SynchExtTriggerEC" \
    -o "example.SequencePlayer.config_file:/tmp/$1$$.conf"

