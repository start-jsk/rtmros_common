#!/bin/bash -i

echo "echo \"i 0x0014 e\" > /dev/pcan32"
echo "i 0x0014 e" > /dev/pcan32
echo "sudo ifconfig can0 down"
sudo ifconfig can0 down
echo "sudo ifconfig can0 up"
sudo ifconfig can0 up
echo "rosrun openrtm rtm-naming-restart"
rosrun openrtm rtm-naming-restart





