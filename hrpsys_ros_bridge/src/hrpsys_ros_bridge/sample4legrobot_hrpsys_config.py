#!/usr/bin/env python

pkg = 'hrpsys'

from hrpsys.hrpsys_config import *
import OpenHRP

class Sample4LegRobotHrpsysConfigurator(HrpsysConfigurator):
    def getRTCList (self):
        return self.getRTCListUnstable()
    def init (self, robotname="Sample4LegRobot", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        print "initialize rtc parameters"
        self.setStAbcParameters()

    def defJointGroups (self):
        rleg_6dof_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
        lleg_6dof_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
        rarm_6dof_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']]
        larm_6dof_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']]
        self.Groups = [rleg_6dof_group, lleg_6dof_group, rarm_6dof_group, larm_6dof_group]

    def setStAbcParameters (self):
        # TL parameters
        tlp=self.tl_svc.getParameter()[1]
        tlp.debug_print_freq = int(10 / 0.002)
        tlp.alarmRatio = 1.0
        self.tl_svc.setParameter(tlp)
        abcp=hcf.abc_svc.getAutoBalancerParam()[1]
        abcp.leg_names = ['rleg', 'lleg', 'rarm', 'larm']
        hcf.abc_svc.setAutoBalancerParam(abcp)
        ggp = hcf.abc_svc.getGaitGeneratorParam()[1]
        ggp.zmp_weight_map = [1.0]*4
        ggp.default_step_height = 0.009 # see https://github.com/fkanehiro/hrpsys-base/issues/801
        hcf.abc_svc.setGaitGeneratorParam(ggp)
        stp_org = hcf.st_svc.getParameter()
        # for tpcc
        stp_org.k_tpcc_p=[0.2, 0.2]
        stp_org.k_tpcc_x=[4.0, 4.0]
        stp_org.k_brot_p=[0.0, 0.0]
        # for eefm
        stp_org.eefm_leg_inside_margin=71.12*1e-3
        stp_org.eefm_leg_outside_margin=71.12*1e-3
        stp_org.eefm_leg_front_margin=182.0*1e-3
        stp_org.eefm_leg_rear_margin=72.0*1e-3
        stp_org.st_algorithm=OpenHRP.StabilizerService.EEFMQPCOP
        rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp_org.eefm_leg_front_margin, stp_org.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp_org.eefm_leg_front_margin, -1*stp_org.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp_org.eefm_leg_rear_margin, -1*stp_org.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp_org.eefm_leg_rear_margin, stp_org.eefm_leg_inside_margin])]
        lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp_org.eefm_leg_front_margin, stp_org.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp_org.eefm_leg_front_margin, -1*stp_org.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp_org.eefm_leg_rear_margin, -1*stp_org.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp_org.eefm_leg_rear_margin, stp_org.eefm_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp_org.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [lleg_vertices, rleg_vertices, larm_vertices, rarm_vertices])
        stp_org.eefm_k1=[-1.39899,-1.39899]
        stp_org.eefm_k2=[-0.386111,-0.386111]
        stp_org.eefm_k3=[-0.175068,-0.175068]
        stp_org.eefm_rot_damping_gain = [[20*1.6*1.5, 20*1.6*1.5, 1e5]]*4
        stp_org.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.0*1.5]]*4
        stp_org.is_ik_enable = [True]*4
        stp_org.is_feedback_control_enable = [True]*4
        stp_org.is_zmp_calc_enable = [True]*4
        stp_org.eefm_use_force_difference_control=False
        stp_org.st_algorithm=OpenHRP.StabilizerService.EEFMQPCOP
        hcf.st_svc.setParameter(stp_org)

    def __init__(self, robotname=""):
        HrpsysConfigurator.__init__(self)
        self.defJointGroups()

if __name__ == '__main__':
    hcf = Sample4LegRobotHrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
