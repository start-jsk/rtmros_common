#!/bin/bash

if [ $# -lt 1 ] ; then
    echo "Usage: $0 [MODEL_FILE]";
    exit 1;
fi
echo $1
OPENHRP3_PATH=`rospack find openhrp3`
ROBOTNAME=`$OPENHRP3_PATH/bin/extract-robotname $1`
if [ $? != 0 ]; then exit 1; fi
MODELFILE=`$OPENHRP3_PATH/bin/extract-modelpath $1`
if [ $? != 0 ]; then exit 1; fi
ROOTLINK=`$OPENHRP3_PATH/bin/extract-rootlink $1`
if [ $? != 0 ]; then exit 1; fi

echo "Stating $ROBOTNAME from $MODELFILE with /tmp/$ROBOTNAME$$.conf"

cat <<EOF > /tmp/$ROBOTNAME$$.conf
model: file://$MODELFILE
dt: 0.005
EOF

gdb -ex run --args `rospack find openhrp3`/bin/openhrp-controller-bridge --server-name $ROBOTNAME \
    --periodic-rate $ROBOTNAME\(Robot\)0:1.0 \
    --periodic-rate HGcontroller0:1.0 \
    --in-port qRef:JOINT_VALUE \
    --in-port dqRef:JOINT_VELOCITY \
    --in-port ddqRef:JOINT_ACCELERATION \
    --out-port q:JOINT_VALUE \
    --out-port pr:$ROOTLINK:ABS_TRANSFORM \
    --out-port left-eye:0:COLOR_IMAGE:0.1 \
    --connection qRef:HGcontroller0:qOut \
    --connection dqRef:HGcontroller0:dqOut \
    --connection ddqRef:HGcontroller0:ddqOut \
    -o "corba.nameservers:localhost:2809" \
    -o "naming.formats:%n.rtc" \
    -o "logger.enable:YES" \
    -o "logger.log_level:NORMAL" \
    -o "logger.file_name:/tmp/$ROBOTNAME$$.log" \
    -o "exec_cxt.periodic.rate:1000" \
    -o "manager.shutdown_onrtcs:NO" \
    -o "manager.is_master:YES" \
    -o "manager.modules.load_path:`rospack find hrpsys`/lib" \
    -o "manager.modules.preload:HGcontroller.so" \
    -o "manager.components.precreate:HGcontroller" \
    -o "exec_cxt.periodic.type:SynchExtTriggerEC" \
    -o "example.SequencePlayer.config_file:/tmp/$ROBOTNAME$$.conf"

