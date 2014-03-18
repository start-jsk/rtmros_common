#!/usr/bin/env python

PKG = 'openrtm_tools'
NAME = 'test-rtmlaunch'

import imp  ## for rosbuild
try:
    imp.find_module(PKG)
except:
    import roslib; roslib.load_manifest(PKG)

import unittest, os, sys, time
import subprocess
from subprocess import call, check_output, Popen, PIPE, STDOUT

import rtctree.tree

class TestRtmLaunch(unittest.TestCase):

    # check if MyServiceProviderComp and MyServiceConsumerComp is activated and connected
    def test_provider_activated(self):
        provider = None
        count = 0
        while count < 20 :
            try:
                tree = rtctree.tree.RTCTree(servers='localhost:2809')
                provider = tree.get_node(['/', 'localhost:2809','MyServiceProvider0.rtc'])
                print >>sys.stderr, "Provier : ", provider, provider.get_state_string()
                if provider.state==rtctree.component.Component.ACTIVE:
                    break
            except:
                pass
            time.sleep(1)
            count += 1
        self.assertTrue(provider.state==rtctree.component.Component.ACTIVE, "State of Provider is %s"%(provider.state_string))

    def test_consumer_activated(self):
        count = 0
        while count < 20 :
            try:
                tree = rtctree.tree.RTCTree(servers='localhost:2809')
                consumer = tree.get_node(['/', 'localhost:2809','MyServiceConsumer0.rtc'])
                print >>sys.stderr, "Consumer : ", consumer, consumer.get_state_string()
                if consumer.state==rtctree.component.Component.ACTIVE:
                    break
            except:
                pass
            time.sleep(1)
            count += 1
        self.assertTrue(consumer.state==rtctree.component.Component.ACTIVE, "State of Consumer is %s"%(consumer.state_string))

    def test_provider_and_consumer_connected(self):
        count = 0
        while count < 20 :
            try:
                tree = rtctree.tree.RTCTree(servers='localhost:2809')
                provider = tree.get_node(['/', 'localhost:2809','MyServiceProvider0.rtc'])
                consumer = tree.get_node(['/', 'localhost:2809','MyServiceConsumer0.rtc'])
                provider_port = provider.get_port_by_name("MyService")
                consumer_port = consumer.get_port_by_name("MyService")
                connection = provider_port.get_connection_by_dest(consumer_port)
                print >>sys.stderr, "Connection : ", connection.properties
                break
            except:
                pass
            time.sleep(1)
            count += 1
        self.assertTrue(connection.properties['port.port_type']=='CorbaPort')
        self.assertTrue(connection.properties['dataport.subscription_type']=='flush')

    # check if SequenceInComponent0.rtc and SequenceOutComponent0.rtc ans activated and connected
    def test_seqin_activated(self):
        count = 0
        while count < 20 :
            try:
                tree = rtctree.tree.RTCTree(servers='localhost:2809')
                seqin = tree.get_node(['/', 'localhost:2809','SequenceInComponent0.rtc'])
                print >>sys.stderr, "SeqIn  : ", seqin, seqin.get_state_string()
                if seqin.state==rtctree.component.Component.ACTIVE:
                    break
            except:
                pass
            time.sleep(1)
            count += 1
        self.assertTrue(seqin.state==rtctree.component.Component.ACTIVE, "State of SeqIn is %s"%(seqin.state_string))

    def test_seqout_activated(self):
        count = 0
        while count < 20 :
            try:
                tree = rtctree.tree.RTCTree(servers='localhost:2809')
                seqout = tree.get_node(['/', 'localhost:2809','SequenceOutComponent0.rtc'])
                print >>sys.stderr, "SeqOut : ", seqout, seqout.get_state_string()
                if seqout.state==rtctree.component.Component.ACTIVE:
                    break
            except:
                pass
            time.sleep(1)
            count += 1
        self.assertTrue(seqout.state==rtctree.component.Component.ACTIVE, "State of SeqOut is %s"%(seqout.state_string))

    def test_seqin_and_seqout_connected(self):
        count = 0
        while count < 20 :
            try:
                tree = rtctree.tree.RTCTree(servers='localhost:2809')
                seqin  = tree.get_node(['/', 'localhost:2809','SequenceInComponent0.rtc'])
                seqout = tree.get_node(['/', 'localhost:2809','SequenceInComponent0.rtc'])
                seqin_port1  = seqin.get_port_by_name("Float")
                seqin_port2  = seqin.get_port_by_name("FloatSeq")
                seqout_port1 = seqout.get_port_by_name("Float")
                seqout_port2 = seqout.get_port_by_name("FloatSeq")
                connection1 = seqin_port1.get_connection_by_dest(seqout_port1)
                connection2 = seqin_port2.get_connection_by_dest(seqout_port2)
                print >>sys.stderr, "Connection : ", connection1.properties
                print >>sys.stderr, "Connection : ", connection2.properties
                break
            except:
                pass
            time.sleep(1)
            count += 1

        self.assertTrue(connection1.properties['dataport.data_type']=='IDL:RTC/TimedFloat:1.0')
        self.assertTrue(connection1.properties['dataport.subscription_type']=='new') # Float
        self.assertTrue(connection1.properties['dataport.publisher.push_policy']=='all')

        self.assertTrue(connection2.properties['dataport.data_type']=='IDL:RTC/TimedFloatSeq:1.0')
        self.assertTrue(connection2.properties['dataport.subscription_type']=='flush') # FloatSeq
        self.assertTrue(connection2.properties.get('dataport.publisher.push_policy')==None)


#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestRtmLaunch, sys.argv)
