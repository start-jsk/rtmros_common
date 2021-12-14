#!/usr/bin/env python

PKG = 'openrtm_tools'
NAME = 'test_rthsell_setup'

import argparse
import unittest, os, sys
import subprocess
from subprocess import call, check_output, Popen, PIPE, STDOUT

class TestOpenrtmTools(unittest.TestCase):

    def setUp(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('--corba_port', type=int, default=2809, help='corba port')
        args, unknown = parser.parse_known_args()
        self.corba_port = args.corba_port

    def run_rt_command(self, command):
        pkg_path = check_output("rospack find openrtm_tools", shell=True).rstrip()
        script = os.path.join(pkg_path, "scripts/rtshell-setup.sh")
        self.assertTrue(os.path.exists(script))
        try:
            check_output("bash -c 'source %s; RTCTREE_NAMESERVERS=localhost:%d %s'"%(script, self.corba_port, command), shell=True, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError, (e):
            self.assertTrue(False, 'subprocess.CalledProcessError: cmd:%s returncode:%s output:%s' % (e.cmd, e.returncode, e.output))
        self.assertTrue(True) # ok rtls works

    def test_rtls(self):
        self.run_rt_command('rtls')

    def test_rtprint(self):
        self.run_rt_command('rtprint -n1 localhost:{}/SequenceOutComponent0.rtc:Float'.format(self.corba_port))
        self.run_rt_command('rtprint -n1 localhost:{}/SequenceOutComponent0.rtc:FloatSeq'.format(self.corba_port))

    def test_rtcat(self):
        self.run_rt_command('rtcat localhost:{}/SequenceOutComponent0.rtc'.format(self.corba_port))

#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestOpenrtmTools, sys.argv)
