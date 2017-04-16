#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Tokyo Opensource Robotics Kyokai Association (TORK)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Tokyo Opensource Robotics Kyokai Association. nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Isaac IY Saito

from hrpsys import rtm
import argparse


class RtmCommandline(object):
    '''
    This class is intended to be called during the initialization step in RTM
    client script, e.g. /nextage_ros_bridge/nextage.py, to replace the tedious
    and erroneous arg parse tasks. 

    Example:

        if __name__ == '__main__':
            commandline = rtm_commandline.RtmCommandline('HiroOpen')  # Robot name can be anything.
            robot = nxc = nextage_client.NextageClient()
            robot.init(robotname=commandline.get_args().robot,
                       url=commandline.get_args().modelfile)
    '''

    def __init__(self, robot_name,
                 robothw_name='RobotHardware0',
                 robothw_name_default='HiroNX(Robot)0'):
        '''
        @type robot_name: str
        '''
        self._robot_name = robot_name
        self._robothw_name = robothw_name
        self._robothw_name_default = robothw_name_default
        self._args = self.init_commandline(robot_name, robothw_name, robothw_name_default)

    def get_args(self):
        return self._args

    def init_commandline(self, robot_name, robothw_name, robothw_name_default):
        '''
        @type robot_name: str
        @type robothw_name: str
        @type robothw_name_default: str
        @rtype: argparse.Namespace
        '''
        parser = argparse.ArgumentParser(
                 description='{} command line interpreters'.format(robot_name))
        parser.add_argument('--host', help='corba name server hostname')
        parser.add_argument('--port', help='corba name server port number')
        parser.add_argument('--modelfile', help='robot model file nmae')
        parser.add_argument('--robot', help='robot modlule name (RobotHardware0 for real robot, Robot()')
        args, unknown = parser.parse_known_args()

        if args.host:
            rtm.nshost = args.host
        if args.port:
            rtm.nsport = args.port
        if not args.robot:
            args.robot = robothw_name if args.host else robothw_name_default
        if not args.modelfile:
            args.modelfile = ""

        # support old style format
        if len(unknown) >= 2:
            args.robot = unknown[0]
            args.modelfile = unknown[1]

        return args
