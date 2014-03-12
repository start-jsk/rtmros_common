#!/usr/bin/env python

PKG = 'openrtm_tools'
NAME = 'test-rthsell-setup'

import unittest, os, sys
from subprocess import call, check_output, Popen, PIPE, STDOUT

class TestOpenrtmTools(unittest.TestCase):

    def test_rtshell_setup(self):
        pkg_path = check_output("rospack find openrtm_tools", shell=True).rstrip()
        script = os.path.join(pkg_path, "scripts/rtshell-setup.sh")
        self.assertTrue(os.path.exists(script))
        print check_output("bash -c 'source %s; RTCTREE_NAMESERVERS=localhost:2809 rtls'"%(script), shell=True)
        self.assertTrue(True) # ok rtls works

#unittest.main()
if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, TestOpenrtmTools, sys.argv)
