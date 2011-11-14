#!/usr/bin/env python

PKG='openhrp3'
import roslib; roslib.load_manifest(PKG)

import os,sys,subprocess,time
import unittest
from optparse import OptionParser
##

class TestGrxUIProject(unittest.TestCase):
    proc=None
    project_filename=""

    def setUp(self):
        parser = OptionParser(description='grxui project testing')
        parser.add_option('--project',action="store",type='string',dest='project_filename',default=None,
                          help='project iflename')
        (options, args) = parser.parse_args()
        self.project_filename = options.project_filename

    def wait_for_window(self,name):
        while subprocess.call("xdotool search --name \""+name+"\"", shell=True) == 1:
            print "wait for \""+name+"\" ..."
            time.sleep(1)
        print "wait for \""+name+"\" ... done"
    def unmap_window(self,name):
        print "unmap \""+name+"\""
        subprocess.call("xdotool search --name \""+name+"\" windowunmap --sync", shell=True)
    def map_window(self,name):
        print "map \""+name+"\""
        subprocess.call("xdotool search --name \""+name+"\" windowmap --sync", shell=True)

    def return_window(self,name):
        print "send return to \""+name+"\""
        subprocess.call("xdotool search --name \""+name+"\" windowactivate --sync key --clearmodifiers Return", shell=True)

    def start_simulation(self):
        print "start simulation"
        subprocess.call("xdotool set_desktop 2", shell=True)
        subprocess.call("xdotool search --name \"Eclipse SDK \" set_desktop_for_window 2", shell=True)
        subprocess.call("xdotool search --name \"Eclipse SDK \" windowmove --sync 0 0", shell=True)
        subprocess.call("xdotool search --name \"Eclipse SDK \" windowactivate --sync", shell=True)
        # start simulator
        subprocess.call("\
        xdotool search --name \"Eclipse SDK\" windowactivate --sync \
	key --clearmodifiers alt+g \
	key --clearmodifiers Down \
	key --clearmodifiers Down \
	key --clearmodifiers Down \
	key --clearmodifiers Down \
	key --clearmodifiers Down \
	key --clearmodifiers Return", shell=True)

    def wait_times_is_up(self):
        #self.wait_for_window("Time is up")
        i = 0
        while subprocess.call("xdotool search --name \"Time is up\"", shell=True) == 1:
            print "wait for \"Time is up\" ..."
            print self.project_filename
            print os.path.basename(self.project_filename)
            print os.path.splitext(os.path.basename(self.project_filename))
            filename="%s-%d.png"%(os.path.splitext(os.path.basename(self.project_filename))[0], i)
            subprocess.call("import -screen -window Eclipse\ SDK\  "+filename, shell=True)
            time.sleep(1)
            i+=1
        print "wait for \"Time is up\" ... done"

        self.unmap_window("Time is up")

    def capture_eclipse(self):
        filename=os.path.splitext(os.path.basename(self.project_filename))[0]+".png"
        subprocess.call("import -screen -window Eclipse\ SDK\  "+filename, shell=True)

    def exit_eclipse(self):
        self.map_window("Time is up")
        self.return_window("Time is up")
        self.wait_for_window("Simulation Finished")
        self.return_window("Simulation Finished")
        subprocess.call("xdotool search --name \"Eclipse SDK\" windowactivate --sync key --clearmodifiers alt+f key --clearmodifiers x", shell=True)
        if self.proc:
            self.proc.wait()

    def test_0_launch_grxui(self):
        subprocess.call("rosrun openhrp3 openhrp-shutdown-servers", shell=True)
        subprocess.call("rosrun openrtm rtm-naming-restart", shell=True)
        self.proc=subprocess.Popen(['rosrun', 'openhrp3', 'grxui.sh', self.project_filename])
    def test_1_wait_for_grxui(self):
        import check_online_viewer
        check_online_viewer.waitOnlineViewer()
    def test_2_capture(self):
        self.start_simulation()
        self.wait_times_is_up()
        self.capture_eclipse()
        self.exit_eclipse()

    def __del__(self):
        if self.proc and self.proc.poll() == None:
            self.proc.terminate()
            self.proc.kill()


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, "test_sample_projects", TestGrxUIProject)

