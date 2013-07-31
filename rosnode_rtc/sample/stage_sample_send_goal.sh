#!/bin/bash -i

set -x

pos_x=21.0
pos_y=25    # 15,20,25
port=move_base_simple_goal
data="RTMROSDataBridge.geometry_msgs_PoseStamped(header=RTMROSDataBridge.std_msgs_Header(seq=0, stamp=RTC.Time(sec=0, nsec=0), frame_id='/map'), pose=RTMROSDataBridge.geometry_msgs_Pose(position=RTMROSDataBridge.geometry_msgs_Point(x=${pos_x}, y=${pos_y}, z=0.0), orientation=RTMROSDataBridge.geometry_msgs_Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)))"

rosrun rtshell rtinject -p /tmp -m RTMROSDataBridge -c "${data}" /localhost/move_base_node0.rtc:${port}
