# bodyinfo ros supported JointAngles
import math

def deg2radPose(pose):
    ret = []
    for p in pose:
        ret += [jv*math.pi/180.0 for jv in p]
    return ret

initialPose = [
   [ 0,    0,    0],
   [-0.6,  0, -100,  15.2,  9.4,  3.2],
   [ 0.6,  0, -100, -15.2,  9.4, -3.2],
   [],   # [ 0,    0,    0,   0],
   [],   # [ 0,    0,    0,   0],
]
print deg2radPose(initialPose)
