#!/bin/bash -i

port=$1
data=$2

echo "waiting..."
while [ 0 -eq `rtcat ${port} | grep "+DataInPort" | wc -l` ]
do
    sleep 1
done
echo "start injecting 10 times to ${port}"

rosrun openrtm rtinject -p /tmp -m RTMROSDataBridge -n 10 -c "${data}" ${port}

# this script may not work in OpenRTM-Python-1.1.0
# ModuleMgr.find_class in rtshell does not parse dataport.data_type
# ex.  std_msgs_String -> IDL:RTMROSDataBridge/std_msgs_String:1.0
