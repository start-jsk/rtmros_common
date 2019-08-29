#!/usr/bin/env python

import rostest, unittest, sys, os

import time
#time.sleep(10) # wait for launch

vars(__builtins__).update({'__IPYTHON__': "fake python as ipython to not to drop in shell"})
class TestInteractive(unittest.TestCase):
    def test(self):
        local_dict = locals() # https://stackoverflow.com/questions/34780235/pythons-execfile-variable-scope-issue
        execfile(os.path.join(os.path.dirname(os.path.abspath(__file__)), "../scripts", "hrpsys_tools_config.py"), globals(), local_dict)
        hcf = local_dict['hcf']
        hcf.setJointAngles([10,10,10,10,10,10,10,10,10], 1)
        print hcf.getJointAngles()
        self.assertTrue("OK")

if __name__ == '__main__':
    rostest.rosrun("hrpsys_tools", "test_interactive", TestInteractive, sys.argv)
