#!/bin/sh

export ROBOT=HIRONX
export JSK_DIR=/opt/jsk
export LOG_DIR=$JSK_DIR/var/log
export PATH=$JSK_DIR/bin:$PATH
export LD_LIBRARY_PATH=$JSK_DIR/lib:/opt/hiro/lib:$LD_LIBRARY_PATH
export RTCD_KILLNAME=rtcd

OS=`uname`
if [ "$OS" = "Linux" ]; then
  KILLALL="/usr/bin/killall -q"
elif [ "$OS" = "QNX" ]; then
  export LD_LIBRARY_PATH=/usr/pkg/lib:$LD_LIBRARY_PATH
  KILLALL=/bin/slay
fi

#
# loop for rtcd
#
mkdir -p $LOG_DIR
$KILLALL -9 $RTCD_KILLNAME

while :; do
    ./unlock_iob


    if [ -f $LOG_DIR/rtcd.log ]; then
        rm $LOG_DIR/rtcd.log
    fi

    DATESTRING=`date +%Y%m%d%H%M%S`
    ln -s $LOG_DIR/rtcd-$DATESTRING.log $LOG_DIR/rtcd.log
    rtcd -f $JSK_DIR/etc/$ROBOT/hrprtc/rtcdRobotMode.conf > $LOG_DIR/rtcd-$DATESTRING.log 2>&1

    /usr/bin/env sleep 1
    $KILLALL -9 $RTCD_KILLNAME
    /usr/bin/env sleep 1
    $KILLALL -9 $RTCD_KILLNAME
done
