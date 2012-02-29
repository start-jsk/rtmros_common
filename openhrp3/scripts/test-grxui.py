#!/usr/bin/env python

PKG='openhrp3'
import roslib; roslib.load_manifest(PKG)
import rospy,tf
import os,sys,shlex,subprocess,time,signal,re
import unittest
from optparse import OptionParser
##

class TestGrxUIProject(unittest.TestCase):
    success = False
    script_proc = None;
    name = ""

    def setUp(self):
        parser = OptionParser(description='grxui project testing')
        parser.add_option('--capture-window',action="store",type='string',dest='capture_window',default="Eclipse ",
                          help='do not launch grxui, just wait finish and capture files')
        parser.add_option('--viewer-name',action="store",type='string',dest='viewer_name',default="OnlineViewer",
                          help='do not launch grxui, just wait finish and capture files')
        parser.add_option('--max-time',action="store",type='int',dest='max_time',default=100,
                          help='wait sec until exit from grxui')
        parser.add_option('--target-directory',action="store",type='string',dest='target_directory',default='/tmp/',
                          help='directory to write results')
        parser.add_option('--init-script',action="store",type='string',dest='init_script',default=None,
                          help='init script to execute before test')
        parser.add_option('--script',action="store",type='string',dest='scripts',default=None,
                          help='sample script to execute during test')
        parser.add_option('--check-tf',action="store",type='string',dest='check_tf',default=None,
                          help='tf pair to test')
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
        self.init_script = options.init_script
        self.max_time = options.max_time
        self.scripts = options.scripts
        self.check_tf = options.check_tf
        self.capture_window = options.capture_window
        self.viewer_name = options.viewer_name

    def xdotool(self,name,action,visible=""):
        ret = 1
        if visible : visible = "--onlyvisible"
        while 1 :
            ret = subprocess.call("xdotool search %s --maxdepth 3 --name \"%s\" %s"%(visible,name,action), shell=True)
            if ret != 1 : break
            time.sleep(1)
        return ret

    def check_window(self,name,visible=""):
        if visible : visible = "--onlyvisible"
        ret = subprocess.call("xdotool search %s --maxdepth 3 --name \"%s\""%(visible,name), shell=True) != 1
        print "[%s] check window %s -> %s"%(self.id(),name, ret)
        return ret

    def wait_for_window(self,name):
        while subprocess.call("xdotool search --maxdepth 3 --name \""+name+"\"", shell=True) == 1:
            print "[%s] wait for %s ..."%(self.id(),name)
            time.sleep(1)
        print "[%s] wait for %s ... done"%(self.id(),name)
    def unmap_window(self,name):
        if self.check_window(name):
            print "[%s] unmap window %s"%(self.id(),name)
            self.xdotool(name, "windowunmap --sync")
    def map_window(self,name):
        if self.check_window(name):
            print "[%s] map window %s"%(self.id(),name)
            self.xdotool(name, "windowmap --sync")
    def move_window(self,name,x,y):
        if self.check_window(name):
            print "[%s] move  %s %d %d"%(self.id(),name, x, y)
            self.xdotool(name, "windowmove --sync %d %d"%(x,y))

    def return_window(self,name):
        if self.check_window(name):
            print "[%s] activate for return %s"%(self.id(),name)
            self.xdotool(name, "windowactivate --sync")
        if self.check_window(name):
            print "[%s] send return %s"%(self.id(),name)
            self.xdotool(name, "key Return",visible=True)
        if self.check_window(name):
            print "[%s] send return %s"%(self.id(),name)
            self.xdotool(name, "key --clearmodifiers Return",visible=True)

    def start_simulation(self):
        print "[%s] start simulation"%(self.id())
        self.xdotool("Eclipse ", "mousemove --sync 400 50")
        self.xdotool("Eclipse ", "windowactivate --sync")
        self.xdotool("Eclipse ", "\
        windowactivate --sync \
	key --clearmodifiers alt+g \
	key --clearmodifiers s")

    def execute_scripts(self):
        print "[%s] start scripts %s"%(self.id(),self.scripts)
        self.script_proc = subprocess.Popen(self.scripts, stdin=subprocess.PIPE, shell=True)

    def terminate_scripts(self):
        p = self.script_proc
        print "[%s] __del__, process %s"%(self.id(), p)
        print subprocess.check_output("ps -o pid,ppid,user,command -ax", shell=True)
        ## get process that has p.pid as ppid
        if p :
            for sig in [signal.SIGINT, signal.SIGKILL, signal.SIGTERM]:
                i = 0
                while p and (p.poll() == None) and (i < 5):
                    print "[%s] __del__, send %d to process %s, poll %s, pid %d (%d)"%(self.id(), sig, p, p.poll(), p.pid, i)
                    p.send_signal(sig)
                    subprocess.call('for child in $(ps -o pid,ppid -ax | awk "{ if ( \$2 == %d ){ print \$1}}"); do  echo "[%s] __del__, kill -%d $child"; kill -%d $child; done'%(p.pid, self.id(), sig, sig), shell=True)
                    i += 1
                    time.sleep(1)
            self.script_proc = None

    def exit_eclipse(self):
        self.map_window("Time is up")
        if self.check_window("Time is up"):
            self.return_window("Time is up")
            print "[%s] send return to \"Time is up\""%(self.id())
            self.wait_for_window("Simulation Finished")
            self.return_window("Simulation Finished")
            print "[%s] send return to \"Simulation finished\""%(self.id())
        print "[%s] send Alt+f -> x -> Return"%(self.id())
        subprocess.call("xdotool search --name \"Eclipse \" windowactivate --sync key --clearmodifiers alt+f key --clearmodifiers x", shell=True)
        subprocess.call("xdotool key --clearmodifiers Return", shell=True)
        subprocess.call("pkill omniNames", shell=True)

    def test_grxui_simulation(self):
        import check_online_viewer,psutil
        # wait online viewer
        if self.viewer_name != '' :
            check_online_viewer.waitForObject(self.viewer_name)
        else:
            time.sleep(10)
        # check window id
        winid = None
        while winid == None:
            try:
                winid = subprocess.check_output("xdotool search \"%s\""%(self.capture_window), shell=True).rstrip()
                print "[%s] eclipse winid %s"%(self.id(), winid)
            except:
                print "[%s] eclipse winid %s .. retry"%(self.id(), winid)
        # for pa10
        if self.check_window("Restoring Problems") :
            print "[%s] Restoring Problems"%(self.id())
            self.xdotool("Restoring Problems", "key Tab",visible=True)
            self.xdotool("Restoring Problems", "key Return",visible=True)

        # start simualation
        if self.simulation_start :
            self.start_simulation()

        # wait tf
        if self.check_tf :
            print "[%s] check tf %s"%(self.id(),self.check_tf)
            self.check_tf = shlex.split(self.check_tf)
            self.tf = tf.TransformListener()
            ret = None
            while (ret == None) and (not rospy.is_shutdown()):
                try:
                    ret = self.tf.lookupTransform(self.check_tf[0], self.check_tf[1], rospy.Time(0))
                    print ret
                except:
                    print "%s wait for %s"%(self.id(), self.check_tf)
                time.sleep(1)
            # send shift-F8 to rviz to start glc capture
            self.xdotool("RViz", "windowactivate --sync",visible=True)
            time.sleep(1);
            subprocess.call("xdotool key --clearmodifiers \"shift+F8\"", shell=True)
            time.sleep(1);
            self.xdotool(self.capture_window, "windowactivate --sync",visible=True)

        # start scripts
        if self.scripts:
            self.execute_scripts()

        # start capture
        if not os.path.exists(self.target_directory) : os.makedirs(self.target_directory)
        ret = subprocess.Popen("recordmydesktop --no-wm-check --no-sound --no-cursor --windowid=%s --on-the-fly-encoding --v_quality=40 --overwrite --output=%s/%s"%(winid,self.target_directory,self.name), shell=True)

        # wait for max time
        start_time = time.time()
        camera_window_names = ['VISION_SENSOR1','VISION_SENSOR2','camera1']
        while 1:
            elapsed_time = time.time() - start_time
            print "[%s] wait for finish... %0.3f/%s"%(self.id(), elapsed_time, self.max_time)
            if  elapsed_time > self.max_time :
                break
            if self.check_window("Time is up") :
                break
            if self.scripts:
                if self.script_proc.poll() == 0 :
                    break;
            for camera_window in camera_window_names :
                if self.check_window(camera_window, visible=True):
                    # self.move_window(camera_window,679,509) # for lucid
                    self.move_window(camera_window,749,509) # for oneirick
                    camera_window_names.remove(camera_window)
            #
            if self.check_window("Setup Controller", visible=True):
                self.xdotool("Setup Controller", "key Tab",visible=True)
                self.xdotool("Setup Controller", "key Return",visible=True)

        # stop terminal
        print "[%s] killing script? .."%(self.id())
        #if self.script_proc :
        #    self.script_proc.communicate("\u0003\n")

        # stop record my desktop
        print "[%s] killing recordmydesktop .."%(self.id())
        subprocess.call("pkill recordmydesktop", shell=True)

        # wait for record my desktop
        #if self.simulation_start :
        #    self.exit_eclipse()
        print "[%s] wait for recordmydesktop encoding..."%(self.id())
        wait_for_recordmydesktop = True
        while wait_for_recordmydesktop :
            print "[%s] wait for recordmydesktop encoding..."%(self.id())
            if all(not p.name == "recordmydesktop" for p in psutil.process_iter()) :
                wait_for_recordmydesktop = False
            time.sleep(1)
        print "[%s] recordmydesktop terminated"%(self.id())

        self.success = True

    def __del__(self):
        # check sucess
        self.assertTrue(self.success)
        #
        print "[%s] __del__, pkill omniNames"%(self.id())
        subprocess.call("pkill omniNames", shell=True)
        subprocess.call("pkill recordmydesktop", shell=True)
        subprocess.call("pkill recordmydesktop", shell=True)
        # wait scripts
        #self.terminate_scripts()


if __name__ == '__main__':
    import rosunit
    rospy.init_node('test_grxui_simulation')
    rosunit.unitrun(PKG, "test_grxui_projects", TestGrxUIProject)

