#!/usr/bin/env python

PKG='openhrp3'
import roslib; roslib.load_manifest(PKG)

import os,sys,shlex,subprocess,time
import unittest
import re
from optparse import OptionParser
##

class TestGrxUIProject(unittest.TestCase):
    proc = None;
    name = ""

    def setUp(self):
        parser = OptionParser(description='grxui project testing')
        parser.add_option('--capture-window',action="store",type='string',dest='capture_window',default="Eclipse\ SDK\ ",
                          help='do not launch grxui, just wait finish and capture files')
        parser.add_option('--max-time',action="store",type='int',dest='max_time',default=100,
                          help='wait sec until exit from grxui')
        parser.add_option('--target-directory',action="store",type='string',dest='target_directory',default='../build/images/',
                          help='directory to write results')
        parser.add_option('--script',action="store",type='string',dest='script',default=None,
                          help='sample script to execute during test')
        parser.add_option('--no-start-simulation',action="store_false",dest='simulation_start', default=False);
        parser.add_option('--start-simulation',action="store_true",dest='simulation_start');
        parser.add_option('--gtest_output'); # dummy
        parser.add_option('--text'); # dummy
        for arg in sys.argv:
            match=re.match('__name:=(.*)',arg)
            if match :
                self.name=match.group(1)
        (options, args) = parser.parse_args()
        self.simulation_start = options.simulation_start
        self.target_directory = options.target_directory
        self.max_time = options.max_time
        self.script = options.script
        self.capture_window = options.capture_window

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
        if not os.path.isdir(self.target_directory) :
            os.mkdir(self.target_directory)
        subprocess.call('rm -f %s/%s*.png'%(self.target_directory,self.name),shell=True)
        self.xdotool(self.capture_window, "windowactivate --sync")
        if self.script:
            args = shlex.split(self.script)
            print "execute script",self.script
            self.proc = subprocess.Popen(args)
        while (not self.check_window("Time is up")) and (i < self.max_time) :
            print "wait for \"Time is up\" (%d/%d) ..."%(i, self.max_time)
            for camera_window in ["VISION_SENSOR1"]:
                if self.check_window(camera_window, visible=True):
                    if self.simulation_start :
                        self.move_window(camera_window,679,509)
                    else:
                        self.unmap_window(camera_window)
            filename="%s-%03d.png"%(self.name, i)
            print "write to ",filename
            subprocess.call('import -frame -screen -window %s %s/%s'%(self.capture_window, self.target_directory, filename), shell=True)
            if self.proc and self.proc.poll() != None:
                print repr(self.proc.communicate()[0])
                print "execute script",self.script
                self.proc = subprocess.Popen(args)
            i+=1
        print "wait for \"Time is up\" ... done"
        self.unmap_window("Time is up")

    def capture_eclipse(self):
        subprocess.call('import -frame -screen -window %s %s/%s.png'%(self.capture_window, self.target_directory, self.name), shell=True)
        subprocess.call('convert -delay 10 -loop 0 %s/%s-*.png %s/%s.gif'%(self.target_directory, self.name, self.target_directory, self.name), shell=True)

    def exit_eclipse(self):
        self.map_window("Time is up")
        if self.check_window("Time is up"):
            self.return_window("Time is up")
            self.wait_for_window("Simulation Finished")
            self.return_window("Simulation Finished")
        subprocess.call("xdotool search --name \"Eclipse SDK\" windowactivate --sync key --clearmodifiers alt+f key --clearmodifiers x", shell=True)
        subprocess.call("xdotool key --clearmodifiers Return", shell=True)
        # wait 10 seconds?
        i = 0
        subprocess.call("pkill omniNames", shell=True)
        # wait scripts
        if self.proc: self.proc.terminate()
        while self.proc and self.proc.poll() == None and i < 10:
            time.sleep(1)
            i += 1

    def test_grxui_simulation(self):
        import check_online_viewer
        # wait online viewer
        check_online_viewer.waitOnlineViewer()
        # start simualation
        if self.simulation_start :
            self.start_simulation()
        # wait for a time
        self.wait_times_is_up()
        # capture result
        self.capture_eclipse()
        # exit
        self.exit_eclipse()

    def __del__(self):
        subprocess.call("pkill omniNames", shell=True)
        # wait scripts
        if self.proc and self.proc.poll() == None:
            self.proc.terminate()
            self.proc.kill()


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, "test_grxui_projects", TestGrxUIProject)

