#!/usr/bin/env python

PKG = 'openrtm_tools'
NAME = 'test-rthsell-setup'

import unittest, os, sys
import subprocess
from subprocess import call, check_output, Popen, PIPE, STDOUT

class TestOpenrtmTools(unittest.TestCase):

    def run_rt_command(self, command):
        pkg_path = check_output("rospack find openrtm_tools", shell=True).rstrip()
        script = os.path.join(pkg_path, "scripts/rtshell-setup.sh")
        self.assertTrue(os.path.exists(script))
        try:
            check_output("bash -c 'source %s; RTCTREE_NAMESERVERS=localhost:2809 %s'"%(script, command), shell=True, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError, (e):
            self.assertTrue(False, 'subprocess.CalledProcessError: cmd:%s returncode:%s output:%s' % (e.cmd, e.returncode, e.output))
        self.assertTrue(True) # ok rtls works

    def test_rtls(self):
        self.run_rt_command('rtls')

    def test_rtprint(self):
        self.run_rt_command('rtprint -n1 localhost:2809/SequenceOutComponent0.rtc:Float')
        self.run_rt_command('rtprint -n1 localhost:2809/SequenceOutComponent0.rtc:FloatSeq')

    def test_rtcat(self):
        self.run_rt_command('rtcat localhost:2809/SequenceOutComponent0.rtc')

#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestOpenrtmTools, sys.argv)
