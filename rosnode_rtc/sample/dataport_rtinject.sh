#!/bin/bash -i

port=$1
data=$2
count=10

echo "waiting..."
while [ 0 -eq `rtcat ${port} | grep "+DataInPort" | wc -l` ]
do
    sleep 1
done
echo "start injecting ${count} times to ${port}"

rosrun rtshell rtinject -p /tmp -m RTMROSDataBridge -n ${count} -c "${data}" ${port}

# this script may not work in OpenRTM-Python-1.1.0
# ModuleMgr.find_class in rtshell does not parse dataport.data_type
# ex.  std_msgs_String -> IDL:RTMROSDataBridge/std_msgs_String:1.0
