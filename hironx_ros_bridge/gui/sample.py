#!/opt/grx/bin/hrpsyspy
import os
import sys
import socket
import math
import time
import rtm
import waitInput
import bodyinfo
import java.lang.System
import org.omg.CORBA.DoubleHolder
import OpenHRP
from OpenHRP.RobotHardwareServicePackage import SwitchStatus

def init(robotHost=None):
    if robotHost != None:
      print 'robot host = '+robotHost
      java.lang.System.setProperty('NS_OPT',
          '-ORBInitRef NameService=corbaloc:iiop:'+robotHost+':2809/NameService')
      rtm.initCORBA()

    print "creating components"
    rtcList = createComps(robotHost)

    print "connecting components"
    connectComps()

    print "activating components"
    activateComps(rtcList)

    print "initialized successfully"

def activateComps(rtcList):
    rtm.serializeComponents(rtcList)
    for r in rtcList:
        r.start()

def initRTC(module, name):
    ms.load(module)
    return ms.create(module, name)

def setJointAnglesDeg(pose, tm, ttm = org.omg.CORBA.DoubleHolder(), wait=True):
    if seq_svc:
        #ret = seq_svc.setJointAngles(bodyinfo.deg2radPose(pose), tm, ttm)
        ret = seq_svc.setJointAngles(bodyinfo.deg2radPose_ROS(pose), tm)
        mask = 31
        if wait:
            seq_svc.isEmpty()
            #seq_svc.isEmpty(mask, True)
        return ret
    else:
        print 'seq_svc is Null'
        return None

def goInitial(tm=bodyinfo.timeToInitialPose):
    setJointAnglesDeg(bodyinfo.initialPose, tm)

def goOffPose(tm=bodyinfo.timeToOffPose):
    setJointAnglesDeg(bodyinfo.offPose, tm)
    servoOff()

def servoOn(part = 'all'):
    waitInputConfirm("!! Robot Motion Warning !! \n Push [OK] to Servo ON "+part)
    if rh_svc != None:
        rh_svc.servo(part, SwitchStatus.SWITCH_ON)
        if part == 'all':
            rh_svc.servo('RHAND', SwitchStatus.SWITCH_ON)
            rh_svc.servo('LHAND', SwitchStatus.SWITCH_ON)

def servoOff(part = 'all'):
    waitInputConfirm("!! Robot Motion Warning !! \n Push [OK] to Servo OFF "+part)
    if rh_svc != None:
        rh_svc.servo(part, SwitchStatus.SWITCH_OFF)
        if part == 'all':
            rh_svc.servo('RHAND', SwitchStatus.SWITCH_OFF)
            rh_svc.servo('LHAND', SwitchStatus.SWITCH_OFF)

def loadPattern(basename, tm=3.0):
    seq_svc.loadPattern(basename, tm)

def testPattern():
    waitInputConfirm("!! Robot Motion Warning !! \n Push [OK] to execute "+\
                     bodyinfo.testPatternName)
    dblHolder = org.omg.CORBA.DoubleHolder()
    for p in bodyinfo.testPattern:
        print setJointAnglesDeg(p[0], p[1], dblHolder)
        print dblHolder.value
    waitInputConfirm("finished")

def badPattern():
    waitInputConfirm("!! Robot Motion Warning !! \n Push [OK] to execute "+\
                     bodyinfo.badPatternName)
    dblHolder = org.omg.CORBA.DoubleHolder()
    for p in bodyinfo.badPattern:
        print setJointAnglesDeg(p[0], p[1], dblHolder)
        print dblHolder.value
    waitInputConfirm("finished")

def createComps(hostname=socket.gethostname()):
    global ms, adm_svc, rh, rh_svc, seq, seq_svc, armR, armR_svc,\
           armL, armL_svc, grsp, grsp_svc, sa, tk_svc, log, force
    ms = rtm.findRTCmanager(hostname)

#    adm = rtm.findRTC("SystemAdmin0")
    adm_svc = None
#    if adm != None:
#        adm.start()
#        adm_svc = OpenHRP.SystemAdminServiceHelper.narrow(adm.service("service0"))

    rh = rtm.findRTC("RobotHardware0")
    rh_svc = OpenHRP.RobotHardwareServiceHelper.narrow(rh.service("service0"))

    seq = initRTC("SequencePlayer", "seq")
    seq_svc = OpenHRP.SequencePlayerServiceHelper.narrow(seq.service("service0"))
    #seq_svc = None

    armR = initRTC("ArmControl", "armR")
#    armR_svc = OpenHRP.ArmControlServiceHelper.narrow(armR.service("service0"))

    armL = initRTC("ArmControl", "armL")
#    armL_svc = OpenHRP.ArmControlServiceHelper.narrow(armL.service("service0"))

    grsp = initRTC("Grasper", "grsp")
#    grsp_svc = OpenHRP.GrasperServiceHelper.narrow(grsp.service("service0"))

    sa = initRTC("StateArbitrator", "sa")
#    tk_svc = OpenHRP.TimeKeeperServiceHelper.narrow(sa.service("service1"))

    log = initRTC("DataLogger", "log")

    # added force sensor RTC load - 2011/10/24
    force = initRTC("ForceSensor", "force")
    print "force = ", force

    return [rh, seq, armR, armL, sa, grsp, log, force]

def connectComps():
    rtm.connectPorts(rh.port("jointDatOut"),   seq.port("jointDatIn"))
    rtm.connectPorts(rh.port("jointDatOut"),  armR.port("jointDatIn"))
    rtm.connectPorts(rh.port("jointDatOut"),  armL.port("jointDatIn"))

    rtm.connectPorts(  seq.port("jointDatOut"),    sa.port("jointDatIn0"))
    rtm.connectPorts( armR.port("jointDatOut"),    sa.port("jointDatIn1"))
    rtm.connectPorts( armL.port("jointDatOut"),    sa.port("jointDatIn2"))

    rtm.connectPorts(   sa.port("jointDatOut"), grsp.port("jointDatIn"))
    rtm.connectPorts( grsp.port("jointDatOut"),    rh.port("jointDatIn"))

def setupLogger():
    global log_svc
    log_svc = OpenHRP.DataLoggerServiceHelper.narrow(log.service("service0"))
    log_svc.add("TimedJointData", "jointDatServo")
    log_svc.add("TimedJointData", "jointDatSeq")
    rtm.connectPorts( rh.port("jointDatOut"),  log.port("jointDatServo"))
    rtm.connectPorts( seq.port("jointDatOut"), log.port("jointDatSeq"))

def saveLog():
    if log_svc == None:
      waitInputConfirm("Setup DataLogger RTC first.")
      return
    log_svc.save('/tmp/sample')
    print 'saved'

def calibrateJoint():
    print('calibrateing joints ...'),
    if rh_svc.calibrateJoint('all') == 0:
      print('finised.')
    else:
      print('failed. execute servoOff() and try again.')

def servoOnHands():
    servoOn('RHAND')
    servoOn('LHAND')

def servoOffHands():
    servoOff('RHAND')
    servoOff('LHAND')
    
def EngageProtectiveStop():
    rh_svc.engageProtectiveStop()
  
def DisengageProtectiveStop():
    rh_svc.disengageProtectiveStop()

def reboot():
    if adm_svc != None:
        waitInputConfirm("Reboot the robot host ?")
        adm_svc.reboot("")

def shutdown():
    if adm_svc != None:
        waitInputConfirm("Shutdown the robot host ?")
        adm_svc.shutdown("")

def getVersion():
    if adm_svc != None:
        currentVersion = adm_svc.getVersion("")
        waitInputConfirm("Robot system version is: %s"%(currentVersion))

#
# move arms using Inverse Kinematics
#
def moveRelativeR(dx=0, dy=0, dz=0, dr=0, dp=0, dw=0, rate=30, wait=True):
  return moveRelative(armR_svc, dx, dy, dz, dr, dp, dw, rate, wait)

def moveRelativeL(dx=0, dy=0, dz=0, dr=0, dp=0, dw=0, rate=30, wait=True):
  return moveRelative(armL_svc, dx, dy, dz, dr, dp, dw, rate, wait)

def moveRelative(armsvc, dx, dy, dz, dr, dp, dw, rate, wait):
  x,y,z,r,p,w = getCurrentConfiguration(armsvc)
  return move(armsvc, x+dx, y+dy, z+dz, r+dr, p+dp, w+dw, rate, wait) 

def moveR(x, y, z, r, p, w, rate=30, wait=True):
  return move(armR_svc, x, y, z, r, p, w, rate, wait)

def moveL(x, y, z, r, p, w, rate=30, wait=True):
  return move(armL_svc, x, y, z, r, p, w, rate, wait)

def move(armsvc, x, y, z, r, p, w, rate, wait):
  ret = armsvc.setTargetAngular(x, y, z, r, p, w, rate)
  while wait and armsvc.checkStatus() == OpenHRP.ArmState.ArmBusyState:
    tk_svc.sleep(0.2)
  return ret

def getCurrentConfiguration(armsvc):
  x = org.omg.CORBA.DoubleHolder()
  y = org.omg.CORBA.DoubleHolder()
  z = org.omg.CORBA.DoubleHolder()
  r = org.omg.CORBA.DoubleHolder()
  p = org.omg.CORBA.DoubleHolder()
  w = org.omg.CORBA.DoubleHolder()
  armsvc.getCurrentConfiguration(x, y, z, r, p, w)
  return [x.value, y.value, z.value, r.value, p.value, w.value]

def ikTest():
  import random
  goInitial(8)
  x0, y0, z0, r0, p0, w0 = getCurrentConfiguration(armR_svc)
  for i in range(10):
    x1 =  x0
    y1 =  y0 + 0.1*random.random()
    z1 =  z0 + 0.1*random.random()
    r1 =  r0
    p1 =  p0
    w1 =  w0
    ret = moveR(x1, y1, z1, r1, p1, w1, 10)
    x2, y2, z2, r2, p2, w2 = getCurrentConfiguration(armR_svc)
    print '\ntarget pos  (x[m], y[m], z[m], r[rad], p[rad], y[rad]) = %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f' %(x1, y1, z1, r1, p1, w1)
    print 'current pos (x[m], y[m], z[m], r[rad], p[rad], y[rad]) = %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f' %(x2, y2, z2, r2, p2, w2)
    if ret < 0:
      print 'ik failed.'
    else:
      print 'differece   (x[m], y[m], z[m], r[rad], p[rad], y[rad]) = %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f' %(x2-x1, y2-y1, z2-z1, r2-r1, p2-p1, w2-w1)
  goInitial(4)

def ikTest_interrupt():
  import random
  ttm = org.omg.CORBA.DoubleHolder()
  goInitial(8)
  x0R, y0R, z0R, r0R, p0R, w0R = getCurrentConfiguration(armR_svc)
  x0L, y0L, z0L, r0L, p0L, w0L = getCurrentConfiguration(armL_svc)
  for i in range(100):
    tk_svc.sleep(0.2)
    
    # right arm
    if random.random() > 0.8:
      y1R = y0R - 0.15*random.random() + 0.05
      z1R = z0R + 0.15*random.random() + 0.05
      ret = moveR(x0R, y1R, z1R, r0R, p0R, w0R, 20, False)
      if ret < 0:
        print 'ik failed (right).'

    # left arm
    if random.random() > 0.7:
      y1L = y0L + 0.15*random.random() - 0.05
      z1L = z0L + 0.15*random.random() + 0.05
      ret = moveL(x0L, y1L, z1L, r0L, p0L, w0L, 20, False)
      if ret < 0:
        print 'ik failed (left).'

    # chest and neck
    if random.random() > 0.8:
      chest = random.random()*40 - 20
      pan   = random.random()*100 - 50
      tilt  = random.random()*45 - 15
      seq_svc.setJointAngles(bodyinfo.deg2radPose([[chest,pan,tilt],[],[],[],[]]), 2, ttm)

    # right hand
    if random.random() > 0.9:
      rhandOpen20()
    else:
      rhandClose()

    # left hand
    if random.random() > 0.9:
      lhandClose()
    else:
      lhandOpen20()

  goInitial(4)

def ikTest_spline():
  import random
  goInitial(8)
  x0, y0, z0, r0, p0, w0 = getCurrentConfiguration(armR_svc)
  nodeseq = []
  for i in range(10):
    x1 =  x0
    y1 =  y0 + 0.1*random.random()
    z1 =  z0 + 0.1*random.random()
    r1 =  r0
    p1 =  p0
    w1 =  w0
    sn = OpenHRP.SplineNode()
    sn.time = 2
    sn.node = [x1, y1, z1, r1, p1, w1]
    nodeseq.append(sn)
  armR_svc.setTargetsSpline(nodeseq)
  #goInitial(4)

#
# grasper
#
def rhandOpen():
  ttm = org.omg.CORBA.DoubleHolder()
  for i in range(12):
    grsp_svc.setJointAngles('RHAND', bodyinfo.anglesFromDistance(i*10), 1.0, ttm)
    tk_svc.sleep(0.1)

def rhandOpen20():
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('RHAND', bodyinfo.anglesFromDistance(20), 1.0, ttm)

def rhandClose():
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('RHAND', bodyinfo.anglesFromDistance(0), 3.0, ttm)

def lhandOpen():
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('LHAND', bodyinfo.anglesFromDistance(100), 1.0, ttm)

def lhandOpen20():
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('LHAND', bodyinfo.anglesFromDistance(20), 1.0, ttm)

def lhandClose():
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('LHAND', bodyinfo.anglesFromDistance(0), 1.0, ttm)

def setRHandAnglesDeg(angles):
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('RHAND', [v*math.pi/180.0 for v in angles], 1.0, ttm)

def setLHandAnglesDeg(angles):
  ttm = org.omg.CORBA.DoubleHolder()
  grsp_svc.setJointAngles('LHAND', [v*math.pi/180.0 for v in angles], 1.0, ttm)


#
# test execution of this script
# e.g: ./sample.py hiro015
#
if __name__ == '__main__' or __name__ == 'main':
  if len(sys.argv) > 1:
    robotHost = sys.argv[1]
  else:
    robotHost = None
  init(robotHost)
  ikTest()
