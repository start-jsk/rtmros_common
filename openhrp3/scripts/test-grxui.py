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
    script_procs = [];
    init_proc = None
    name = ""

    def setUp(self):
        parser = OptionParser(description='grxui project testing')
        parser.add_option('--capture-window',action="store",type='string',dest='capture_window',default="Eclipse\ SDK\ ",
                          help='do not launch grxui, just wait finish and capture files')
        parser.add_option('--max-time',action="store",type='int',dest='max_time',default=100,
                          help='wait sec until exit from grxui')
        parser.add_option('--target-directory',action="store",type='string',dest='target_directory',default='../build/images/',
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

    def xdotool(self,name,action,visible=""):
        ret = 1
        if visible : visible = "--onlyvisible"
        while 1 :
            ret = subprocess.call("xdotool search %s --name \"%s\" %s"%(visible,name,action), shell=True)
            if ret != 1 : break
            time.sleep(1)
        return ret

    def check_window(self,name,visible=""):
        if visible : visible = "--onlyvisible"
        print "[%s] check window %s"%(self.id(),name)
        return subprocess.call("xdotool search "+visible+" --name \""+name+"\"", shell=True) != 1

    def wait_for_window(self,name):
        while subprocess.call("xdotool search --name \""+name+"\"", shell=True) == 1:
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
            print "[%s] send return %s"%(self.id(),name)
            self.xdotool(name, "windowactivate --sync key --clearmodifiers Return")

    def start_simulation(self):
        print "[%s] start simulation"%(self.id())
        self.xdotool("Eclipse SDK ", "mousemove --sync 400 50")
        #subprocess.call("xdotool set_desktop 2", shell=True)
        #subprocess.call("xdotool search --name \"Eclipse SDK \" set_desktop_for_window 2", shell=True)
        self.xdotool("Eclipse SDK ", "windowmove --sync 0 0")
        self.xdotool("Eclipse SDK ", "windowactivate --sync")
        self.xdotool("Eclipse SDK ", "\
        windowactivate --sync \
	key --clearmodifiers alt+g \
	key --clearmodifiers s")

    def execute_scripts(self):
        scripts = shlex.shlex(self.scripts)
        whitespace = scripts.whitespace
        scripts.whitespace = ';'
        scripts.whitespace_split = True
        self.script_procs = []
        for script in scripts:
            args = shlex.split(script)
            self.script_procs.append(subprocess.Popen(args))

    def terminate_scripts(self):
        if self.init_proc and self.init_proc.poll() == None:
            self.init_proc.send_signal(2) ## send SIGINT
            self.init_proc.terminate()
            self.init_proc.kill()
        self.init_proc = None
        for p in self.script_procs:
            print "[%s] __del__, process %s"%(self.id(), p)
            ## get process that has p.pid as ppid
            p.pid
            for sig in [signal.SIGINT, signal.SIGKILL, signal.SIGTERM]:
                i = 0
                while p and (p.poll() == None) and (i < 5):
                    print "[%s] __del__, send %d to process %s, poll %s, pid %d (%d)"%(self.id(), sig, p, p.poll(), p.pid, i)
                    p.send_signal(sig)
                    subprocess.call('for child in $(ps -o pid,ppid -ax | awk "{ if ( \$2 == %d ){ print \$1}}"); do  echo $child; kill %d $child; done'%(p.pid, sig), shell=True)
                    i += 1
                    time.sleep(1)
        self.script_procs = []

    def wait_times_is_up(self):
        i = 0
        if not os.path.isdir(self.target_directory) :
            os.mkdir(self.target_directory)
        subprocess.call('rm -f %s/%s*.png'%(self.target_directory,self.name),shell=True)
        self.xdotool(self.capture_window, "windowactivate --sync")
        if self.scripts: self.execute_scripts()
        while (not self.check_window("Time is up")) and (i < self.max_time) :
            print "[%s] wait for \"Time is up\" (%d/%d) ..."%(self.id(), i, self.max_time)
            print self.check_tf
            for camera_window in ["VISION_SENSOR1","VISION_SENSOR2"]:
                if self.check_window(camera_window, visible=True):
                    if self.simulation_start :
                        self.move_window(camera_window,679,509)
                    else:
                        self.unmap_window(camera_window)
            filename="%s-%03d.png"%(self.name, i)
            print "[%s] write to %s"%(self.id(), filename)
            self.xdotool(self.capture_window, "windowactivate --sync", visible=True)
            #ret = subprocess.call('import -frame -screen -window %s %s/%s'%(self.capture_window, self.target_directory, filename), shell=True)
            ret = subprocess.call('import -frame -screen -window root %s/%s'%(self.target_directory, filename), shell=True)
            print "[%s] import returns %s"%(self.id(), ret)
            self.assertEqual(ret, 0) #
            if self.script_procs and all(map(lambda x: x.poll()!=None, self.script_procs)) :
                for p in self.script_procs:
                    print "[%s] %s"%(self.id(),repr(p.communicate()[0]))
                self.execute_scripts()
            i+=1
        print "[%s] wait for \"Time is up\" ... done"%(self.id())
        self.unmap_window("Time is up")

    def capture_eclipse(self):
        subprocess.call('import -frame -screen -window %s %s/%s.png'%(self.capture_window, self.target_directory, self.name), shell=True)

    def exit_eclipse(self):
        self.map_window("Time is up")
        if self.check_window("Time is up"):
            self.return_window("Time is up")
            self.wait_for_window("Simulation Finished")
            self.return_window("Simulation Finished")
        subprocess.call("xdotool search --name \"Eclipse SDK\" windowactivate --sync key --clearmodifiers alt+f key --clearmodifiers x", shell=True)
        subprocess.call("xdotool key --clearmodifiers Return", shell=True)
        subprocess.call("pkill omniNames", shell=True)
        # wait scripts
        self.terminate_scripts()
        # create animation gif
        print "[%s] convert png to gif"%(self.id())
        subprocess.call('convert -delay 10 -loop 0 %s/%s-*.png %s/%s.gif'%(self.target_directory, self.name, self.target_directory, self.name), shell=True)
        self.success = True

    def test_grxui_simulation(self):
        if self.init_script :
            print shlex.split(self.init_script)
            self.init_proc = subprocess.Popen(shlex.split(self.init_script))
        import check_online_viewer
        # wait online viewer
        check_online_viewer.waitOnlineViewer()
        # start simualation
        if self.simulation_start :
            self.start_simulation()
        # wait for tf
        if self.check_tf :
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
        # wait for a time
        self.wait_times_is_up()
        # capture result
        self.capture_eclipse()
        # exit
        self.exit_eclipse()

    def __del__(self):
        # check sucess
        self.assertTrue(self.success)
        #
        print "[%s] __del__, pkill omniNames"%(self.id())
        subprocess.call("pkill omniNames", shell=True)
        # wait scripts
        self.terminate_scripts()


if __name__ == '__main__':
    import rosunit
    rospy.init_node('test_grxui_simulation')
    rosunit.unitrun(PKG, "test_grxui_projects", TestGrxUIProject)

