#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest("hrpsys")
import sys
import os
import socket
import time
import math
import numpy

import rtm
import OpenHRP

import sys; sys.path.insert (0, roslib.packages.get_pkg_dir('hrpsys')+'/scripts'); ## add python
from hrpsys import HrpsysConfigurator

from waitInput import waitInputConfirm, waitInputSelect

SWITCH_ON  = OpenHRP.RobotHardwareService.SWITCH_ON
SWITCH_OFF = OpenHRP.RobotHardwareService.SWITCH_OFF

class HIRONX(HrpsysConfigurator):
    OffPose     = [[0], [0,0],
                   [ 25,-139,-157,45,0,0],
                   [-25,-139,-157,-45,0,0],
                   [0,0,0,0],
                   [0,0,0,0]]
    InitialPose = [[0], [0,0],
                   [-0.6,0,-100,15.2,9.4,3.2],
                   [0.6,0,-100,-15.2,9.4,-3.2],
                   [0,0,0,0],
                   [0,0,0,0]]

    Groups = [['torso', ['CHEST_JOINT0']],
              ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']],
              ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']],
              ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']]]
    HandGroups = {'rhand': [2, 3, 4, 5], 'lhand': [6, 7, 8, 9]}

    RtcList = []

    # servo controller (grasper)
    sc = None
    sc_svc = None

    #
    # hiro specific methods
    #
    def getRTCList(self):
        return [self.rh, self.seq, self.sh, self.fk, self.sc, self.log]

    def init(self, robotname="HiroNX(Robot)0", url=""):
        HrpsysConfigurator.init(self, robotname=robotname, url=url)
        self.setSelfGroups()

    def goOffPose(self, tm=7):
        for i in range(len(self.Groups)):
            self.setJointAnglesOfGroup(self.Groups[i][0], self.OffPose[i], tm, wait=False)
        for i in range(len(self.Groups)):
            self.seq_svc.waitInterpolationOfGroup(self.Groups[i][0])
        self.servoOff(wait=False)


    def goInitial(self, tm=7, wait=True):
        ret = True
        for i in range(len(self.Groups)):
            #radangles = [x/180.0*math.pi for x in self.InitialPose[i]]
            print self.configurator_name, 'self.setJointAnglesOfGroup(',self.Groups[i][0],',', self.InitialPose[i],', ',tm,',wait=False)'
            ret &= self.setJointAnglesOfGroup(self.Groups[i][0], self.InitialPose[i], tm, wait=False)
        if wait:
            for i in range(len(self.Groups)):
                self.seq_svc.waitInterpolationOfGroup(self.Groups[i][0])
        return ret

    def createComps(self):
        self.seq = self.createComp("SequencePlayer", "seq")
        if self.seq :
            self.seq_svc = rtm.narrow(self.seq.service("service0"), "SequencePlayerService")

        self.sh = self.createComp("StateHolder", "sh")
        if self.sh :
            self.sh_svc = rtm.narrow(self.sh.service("service0"), "StateHolderService")

        self.fk = self.createComp("ForwardKinematics", "fk")
        if self.fk :
            self.fk_svc = rtm.narrow(self.fk.service("service0"), "ForwardKinematicsService")

        #self.co = self.createComp("CollisionDetector", "co")
        #if self.co :
        #    self.co_svc = rtm.narrow(self.co.service("service0"), "CollisionDetectorService")

        # servo controller (grasper)
        self.sc = self.createComp("ServoController", "sc")
        if self.sc :
            self.sc_svc = rtm.narrow(self.sc.service("service0"), "ServoControllerService")

        self.log = self.createComp("DataLogger", "log")
        if self.log :
            self.log_svc = rtm.narrow(self.log.service("service0"), "DataLoggerService");

    def connectComps(self):
        if self.sh and self.rh:
            print self.configurator_name, 'connectComps: self.rh =', self.rh
            print self.configurator_name, 'connectComps self.sh =', self.sh
            rtm.connectPorts(self.rh.port("q"), [self.sh.port("currentQIn"), self.fk.port("q")])

        #
        if self.seq and self.sh:
            rtm.connectPorts(self.seq.port('qRef'),      self.sh.port('qIn'))
            rtm.connectPorts(self.seq.port('basePos'),   self.sh.port('basePosIn'))
            rtm.connectPorts(self.seq.port('baseRpy'),   self.sh.port('baseRpyIn'))

            #
            rtm.connectPorts(self.sh.port('qOut'),       [self.fk.port("qRef"),self.seq.port('qInit')])
            rtm.connectPorts(self.sh.port('basePosOut'), [self.seq.port('basePosInit'), self.fk.port("basePosRef")])
            rtm.connectPorts(self.sh.port('baseRpyOut'), [self.seq.port('baseRpyInit'), self.fk.port("baseRpyRef")])
            #
            rtm.connectPorts(self.sh.port('qOut'),       self.rh.port('qRef'))

    #
    # hand interface
    # effort: 1~100[%]
    # hiro.HandOpen("rhand", effort)
    # hiro.HandOpen()        # for both hand
    # hiro.HandClose("rhand", effort)
    # hiro.HandClose()       # for both hand
    #
    def HandOpen(self, hand=None, effort=None):
        self.setHandWidth(hand, 100, effort=effort)
    def HandClose(self, hand=None, effort=None):
        self.setHandWidth(hand, 0, effort=effort)
    def setHandJointAngles(self, hand, angles, tm=1):
        self.sc_svc.setJointAnglesOfGroup(hand, angles, tm)
    def setHandEffort(self, effort=100):
        for i in [ v for vs in HandGroups.values() for v in vs]: # flatten
            self.sc_svc.setMaxTorque(i, effort)
    def setHandWidth(self, hand, width, tm=1, effort=None):
        if effort:
            self.setHandEffort(effort)
        if hand:
            self.setHandJointAngles(hand, self.hand_width2angles(width), tm)
        else:
            for h in self.HandGroups.keys():
                self.setHandJointAngles(h, self.hand_width2angles(width), tm)
    def hand_width2angles(self, width):
        safetyMargin = 3
        l1, l2 = (41.9, 19)

        if width < 0.0 or width > (l1+l2 - safetyMargin)*2:
            return None

        xPos   = width/2.0 + safetyMargin
        a2Pos  = xPos - l2
        a1radH = math.acos(a2Pos/l1)
        a1rad  = math.pi/2.0 - a1radH

        return a1rad, -a1rad, -a1rad, a1rad
    #
    #
    #
    def setSelfGroups(self):
        for item in self.Groups:
            self.seq_svc.addJointGroup(item[0], item[1])
        for k, v in self.HandGroups.iteritems():
            self.sc_svc.addJointGroup(k, v)

    #
    def getActualState(self):
        return self.rh_svc.getStatus()

    # Check whether joints have been calibrated
    def isCalibDone(self):
        if self.simulation_mode:
            return True
        else:
            rstt = self.rh_svc.getStatus()
            for item in rstt.servoState:
                if not item[0]&1:
                    return False
        #
        return True

    # Check whether servo control has been switched on
    def isServoOn(self, jname='any'):
        if self.simulation_mode:
            return True
        else:
            s_s = self.getActualState().servoState
            if jname.lower() == 'any' or jname.lower() == 'all':
                for s in s_s:
                    #print self.configurator_name, 's = ', s
                    if (s[0]&2) == 0:
                        return False
            else:
                jid = eval('self.'+jname)
                print self.configurator_name, s_s[jid]
                if s_s[jid][0]&1 == 0:
                    return False
            return True
        return False


    def liftRobotUp(self):
        return True

    def stOff(self):
        return False

    def flat2Groups(self, flatList):
        retList = []
        index = 0
        for group in self.Groups:
            joint_num = len(group[1])
            retList.append(flatList[index : index+joint_num])
            index += joint_num
        return retList

    # switch servos on/off
    # destroy argument is not used
    def servoOn(self, jname='all', destroy=1, tm=3):
        # check joints are calibrated
        if not self.isCalibDone():
            waitInputConfirm('!! Calibrate Encoders with checkEncoders first !!\n\n')
            return -1

        # check servo state
        if self.isServoOn():
            return 1

        # check jname is acceptable
        if jname == '':
            jname = 'all'

        self.liftRobotUp()
        self.rh_svc.power('all', SWITCH_ON)

        try:
            waitInputConfirm(\
                '!! Robot Motion Warning (SERVO_ON) !!\n\n'
                'Confirm RELAY switched ON\n'
                'Push [OK] to switch servo ON(%s).'%(jname))
        except: # ths needs to change
            self.rh_svc.power('all', SWITCH_OFF)
            raise

        try:
            self.stOff()
            self.goActual()
            time.sleep(0.1)
            self.rh_svc.servo(jname, SWITCH_ON)
            time.sleep(2)
            #time.sleep(7)
            if not self.isServoOn(jname):
                print self.configurator_name, 'servo on failed.'
                raise
        except:
            print self.configurator_name, 'exception occured'

        try:
            angles = self.flat2Groups(map(numpy.rad2deg, self.getActualState().angle))
            print 'Move to Actual State, Just a minute.'
            for i in range(len(self.Groups)):
                self.setJointAnglesOfGroup(self.Groups[i][0], angles[i], tm, wait=False)
            print 'Movement finished'
            for i in range(len(self.Groups)):
                self.seq_svc.waitInterpolationOfGroup(self.Groups[i][0])
        except:
            print self.configurator_name, 'post servo on motion trouble'

        # turn on hand motors
        self.sc_svc.servoOn()

        return 1

    #
    def servoOff(self, jname = 'all', wait=True):
        # do nothing for simulation
        if self.simulation_mode:
            print self.configurator_name, 'omit servo off'
            return 0

        # if the servos aren't on switch power off
        if not self.isServoOn(jname):
            if jname.lower() == 'all':
                self.rh_svc.power('all', SWITCH_OFF)
            return 1

        # if jname is not set properly set to all -> is this safe?
        if jname == '':
            jname = 'all'

        self.liftRobotUp()

        if wait:
            waitInputConfirm(
                '!! Robot Motion Warning (Servo OFF)!!\n\n'
                'Push [OK] to servo OFF (%s).'%(jname)) #:

        try:
            self.rh_svc.servo('all', SWITCH_OFF)
            time.sleep(0.2)
            if jname == 'all':
                self.rh_svc.power('all', SWITCH_OFF)

            # turn off hand motors
            self.sc_svc.servoOff()

            return 2
        except:
            print self.configurator_name, 'servo off: communication error'
            return -1
    #
    def checkEncoders(self, jname='all', option=''):
        if self.isServoOn():
            waitInputConfirm('Servo must be off for calibration')
            return
        # do not check encoders twice
        elif self.isCalibDone():
            waitInputConfirm('System has been calibrated')
            return

        self.rh_svc.power('all', SWITCH_ON)
        msg = '!! Robot Motion Warning !!\n'\
              'Turn Relay ON.\n'\
              'Then Push [OK] to '
        if option == '-overwrite':
            msg = msg + 'calibrate(OVERWRITE MODE) '
        else:
            msg = msg + 'check '

        if jname == 'all':
            msg = msg + 'the Encoders of all.'
        else:
            msg = msg + 'the Encoder of the Joint "'+jname+'".'

        try:
            waitInputConfirm(msg)
        except:
            self.rh_svc.power('all', SWITCH_OFF)
            return 0

        print self.configurator_name, 'calib-joint ' + jname + ' ' + option
        self.rh_svc.initializeJointAngle(jname, option)
        print self.configurator_name, 'done'
        self.rh_svc.power('all', SWITCH_OFF)
        self.goActual()
        time.sleep(0.1)
        self.rh_svc.servo(jname, SWITCH_ON)

        # turn on hand motors
        self.sc_svc.servoOn()

if __name__ == '__main__':
    hiro = HIRONX()
    if len(sys.argv) > 2:
        hiro.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1:
        hiro.init(sys.argv[1])
    else:
        hiro.init()

# for simulated robot
# $ ./hironx.py
#
# for real robot
# ./hironx.py RobotHardware0 -ORBInitRef NameService=corbaloc:iiop:hiro014:2809/NameService
# ipython -i ./hironx.py RobotHardware0 -- -ORBInitRef NameService=corbaloc:iiop:hiro014:2809/NameService

