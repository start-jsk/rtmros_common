#!/usr/bin/env python

PKG = 'hrpsys'
import roslib; roslib.load_manifest(PKG)
import unittest

import rtm

from rtm import *
from OpenHRP import *

def findModelLoader():
    try:
        return rtm.findObject("ModelLoader")
    except:
        return None

import numpy
from numpy import array,mat,sin,cos,dot,eye,identity
from numpy.linalg import norm
numpy.set_printoptions(precision=6)

def rodrigues(w, dt):
    def S(n):
        Sn = array([[0,-n[2],n[1]],[n[2],0,-n[0]],[-n[1],n[0],0]])
        return Sn
    th = norm(w)*dt
    wn = w/norm(w)
    w_wedge = S(wn)
    return eye(3) +  w_wedge*sin(th) +  dot(w_wedge,w_wedge)*(1-cos(th))

class testModelLoader(unittest.TestCase):
    class Link:
        p = numpy.array([0,0,0])
        R = identity(3)

        mass = 0

        parent = -1;
        children = []

        def __init__(self, l, parentRs) :
            self.name = l.name
            self.b = dot(parentRs, array(l.translation))
            self.Rs = dot(parentRs, rodrigues(l.rotation[0:3],l.rotation[3]))
            self.q = 0
            self.a = dot(self.Rs,array(l.jointAxis))
            self.mass = l.mass
            self.parent = l.parentIndex
            self.children = l.childIndices

    links = []

    def loadModel(self, fname) :
        self.links = []
        self.ml = findModelLoader()
        self.binfo = self.ml.getBodyInfo(fname)
        self.createLink(self.binfo._get_links()[0], identity(3)) # set root link
        self.forwardKinematics()

    def createLink(self, l, parentRs):
        lk = self.Link(l, parentRs)
        self.links.append(lk)
        print "{0}\n    translation:{1}\n       rotation:{2}\n           mass:{3}\n   centorOfMass:{4}".format(lk.name, lk.p, dot(lk.R, lk.Rs).flatten(), lk.mass, l.centerOfMass)
        for i in l.childIndices:
            self.createLink(self.binfo._get_links()[i], lk.Rs)

    #
    def forwardKinematics(self, j=0):
        if self.links[j].parent == -1 :
            self.links[j].p = self.links[j].b
        else:
            mon = self.links[j].parent
            self.links[j].p = dot(self.links[mon].R, self.links[j].b) + self.links[mon].p
            self.links[j].R = dot(self.links[mon].R, rodrigues(self.links[j].a, self.links[j].q))

        for jj in self.links[j].children:
            self.forwardKinematics(jj)

    #
    def check_link(self, name, p, R) :
        l = filter(lambda l : l.name == name, self.links)[0]
        print "check_link ", l.name
        print "           p = ", array(p), "r = ", array(R)
        print "           p = ", l.p, "r = ", dot(l.R, l.Rs).flatten()
        numpy.testing.assert_array_almost_equal(l.p, p, decimal=4)
        numpy.testing.assert_array_almost_equal(dot(l.R, l.Rs).flatten(), R, decimal=4)

    def angle_vector(self, av):
        self.assertEqual(len(av), len(self.links[1:]))
        i = 0
        for j in self.links[1:]:
            j.q = av[i]
            i = i+1
        print "angle-vector : ", [j.name for j in self.links[1:] ]
        print "               ", [a * 180 / math.pi for a in av]
        self.forwardKinematics()

