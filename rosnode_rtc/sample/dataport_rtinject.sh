#!/bin/bash -i

port=$1
data=$2

waitsec=10

echo "sleep ${waitsec} seconds, then rtinject 10 times"
sleep ${waitsec}
rosrun openrtm rtinject -p /tmp -m RTMROSDataBridge -n 10 -c "${data}" /localhost/RTMROSDataBridge0.rtc:${port}

# this script may not work in OpenRTM-Python-1.1.0
# ModuleMgr.find_class in rtshell does not parse dataport.data_type
# ex.  std_msgs_String -> IDL:RTMROSDataBridge/std_msgs_String:1.0
