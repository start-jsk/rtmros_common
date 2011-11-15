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
    capture_filename=""
    max_time=100

    def setUp(self):
        parser = OptionParser(description='grxui project testing')
        parser.add_option('--project',action="store",type='string',dest='project_filename',default=None,
                          help='project xml filename')
        parser.add_option('--capture',action="store",type='string',dest='capture_filename',default=None,
                          help='do not launch grxui, just wait finish and capture files')
        parser.add_option('--max-time',action="store",type='int',dest='max_time',default=self.max_time,
                          help='wait sec until exit from grxui')
        parser.add_option('--gtest_output'); # dummy
        parser.add_option('--text'); # dummy
        (options, args) = parser.parse_args()
        self.project_filename = options.project_filename
        self.max_time = options.max_time
        if options.capture_filename:
            self.capture_filename = options.capture_filename
        else:
            self.capture_filename = os.path.splitext(os.path.basename(self.project_filename))[0]+".png"

    def xdotool(self,name,action):
        ret = 1
        while 1 :
            ret = subprocess.call("xdotool search --name \"%s\" %s"%(name,action), shell=True)
            if ret != 1 : break
            time.sleep(1)
        return ret

    def check_window(self,name,visible=""):
        if visible : visible = "--onlyvisible"
        print "check window", name
        return subprocess.call("xdotool search "+visible+" --name \""+name+"\"", shell=True) != 1

    def wait_for_window(self,name):
        while subprocess.call("xdotool search --name \""+name+"\"", shell=True) == 1:
            print "wait for \""+name+"\" ..."
            time.sleep(1)
        print "wait for \""+name+"\" ... done"
    def unmap_window(self,name):
        if self.check_window(name):
            print "unmap \""+name+"\""
            self.xdotool(name, "windowunmap --sync")
    def map_window(self,name):
        if self.check_window(name):
            print "map \""+name+"\""
            self.xdotool(name, "windowmap --sync")
    def move_window(self,name,x,y):
        if self.check_window(name):
            print "move %s %d %d"%(name,x,y)
            self.xdotool(name, "windowmove --sync %d %d"%(x,y))

    def return_window(self,name):
        if self.check_window(name):
            print "send return to \""+name+"\""
            self.xdotool(name, "windowactivate --sync key --clearmodifiers Return")

    def start_simulation(self):
        print "start simulation"
        self.xdotool("Eclipse SDK ", "mousemove --sync 400 50")
        #subprocess.call("xdotool set_desktop 2", shell=True)
        #subprocess.call("xdotool search --name \"Eclipse SDK \" set_desktop_for_window 2", shell=True)
        self.xdotool("Eclipse SDK ", "windowmove --sync 0 0")
        self.xdotool("Eclipse SDK ", "windowactivate --sync")
        self.xdotool("Eclipse SDK ", "\
        windowactivate --sync \
	key --clearmodifiers alt+g \
	key --clearmodifiers s")

    def wait_times_is_up(self):
        i = 0
        while (not self.check_window("Time is up")) and (i < self.max_time):
            print "wait for \"Time is up\" (%d/%d) ..."%(i, self.max_time)
            for camera_window in ["VISION_SENSOR1"]:
                if self.check_window(camera_window, visible=True):
                    self.move_window(camera_window,679,509)
            filename="%s-%03d.png"%(os.path.splitext(os.path.basename(self.capture_filename))[0], i)
            if os.path.dirname(self.capture_filename):
                filename=os.path.dirname(self.capture_filename)+"/"+filename
            print "write to ",filename
            subprocess.call("import -frame -screen -window Eclipse\ SDK\  "+filename, shell=True)
            time.sleep(1)
            i+=1
        print "wait for \"Time is up\" ... done"
        self.unmap_window("Time is up")

    def capture_eclipse(self):
        subprocess.call("import -screen -window Eclipse\ SDK\  "+self.capture_filename, shell=True)

    def exit_eclipse(self):
        self.map_window("Time is up")
        if self.check_window("Time is up"):
            self.return_window("Time is up")
            self.wait_for_window("Simulation Finished")
            self.return_window("Simulation Finished")
        subprocess.call("xdotool search --name \"Eclipse SDK\" windowactivate --sync key --clearmodifiers alt+f key --clearmodifiers x", shell=True)
        subprocess.call("xdotool key --clearmodifiers return")
        # wait 10 seconds?
        i = 0
        while self.proc and self.proc.poll() == None and i < 10:
            timer.sleep(1)
            i += 1

    def test_grxui_simulation(self):
        import check_online_viewer
        if self.project_filename: # if no project_filename is given, we assume grxui is already launched
            subprocess.call("rosrun openhrp3 openhrp-shutdown-servers", shell=True)
            subprocess.call("rosrun openrtm rtm-naming-restart", shell=True)
            self.proc=subprocess.Popen(['rosrun', 'openhrp3', 'grxui.sh', self.project_filename])
        # wait online viewer
        check_online_viewer.waitOnlineViewer()
        # start simualation
        self.start_simulation()
        # wait for a time
        self.wait_times_is_up()
        # capture result
        self.capture_eclipse()
        # exit
        self.exit_eclipse()

    def __del__(self):
        if self.proc and self.proc.poll() == None:
            self.proc.terminate()
            self.proc.kill()


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, "test_grxui_projects", TestGrxUIProject)

