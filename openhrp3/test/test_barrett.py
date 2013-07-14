#!/usr/bin/env python

from  test_modelloader import testModelLoader
import rospkg, math

class TestBarrettWam(testModelLoader):

    def setUp(self):
        self.loadModel(rospkg.RosPack().get_path("openhrp3")+"/test/barrett-wam.zae")

    def test_av0(self):
        self.check_link("Shoulder_Yaw", [0.0, 0.0, 0.0], [1.0, 0.0, 0.0,  0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.check_link("Shoulder_Pitch", [0.0, 0.0, 0.0], [1.0, 0.0, 0.0,  0.0, 3.267949e-07, 1.0, 0.0, -1.0, 3.267949e-07])
        self.check_link("Shoulder_Roll", [0.0, 0.0, 0.0], [1.0, 0.0, 0.0,  0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.check_link("Elbow", [0.045, 0.0, 0.55], [1.0, 0.0, 0.0,  0.0, 3.267949e-07, 1.0, 0.0, -1.0, 3.267949e-07])
        self.check_link("Wrist_Yaw", [0.0, 0.0, 0.55], [1.0, 0.0, 0.0,  0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.check_link("Wrist_Pitch", [0.0, 0.0, 0.85], [1.0, 0.0, 0.0,  0.0, 3.267949e-07, 1.0, 0.0, -1.0, 3.267949e-07])
        self.check_link("Wrist_Roll", [0.0, 0.0, 0.85], [1.0, 0.0, 0.0,  0.0, 1.0, 0.0, 0.0, 0.0, 1.0])

    def test_av30(self):
        self.angle_vector([math.pi * 30 / 180 for i in range(7)])

        self.check_link("Shoulder_Yaw", [0.0, 0.0, 0.0], [0.866025, -0.5, 0.0,  0.5, 0.866025, 0.0, 0.0, 0.0, 1.0])
        self.check_link("Shoulder_Pitch", [0.0, 0.0, 0.0], [0.75, -0.433013, -0.5,  0.433013, -0.25, 0.866025, -0.5, -0.866025, 3.267949e-07])
        self.check_link("Shoulder_Roll", [0.0, 0.0, 0.0], [0.399519, -0.808013, 0.433013,  0.808013, 0.533494, 0.25, -0.433013, 0.25, 0.866025])
        self.check_link("Elbow", [0.256135, 0.173861, 0.456828], [0.129487, -0.57476, -0.808013,  0.57476, -0.620513, 0.533494, -0.808013, -0.533494, 0.25])
        self.check_link("Wrist_Yaw", [0.250308, 0.147996, 0.493189], [-0.291867, -0.764503, 0.574759,  0.764503, 0.174639, 0.620513, -0.574759, 0.620513, 0.533494])
        self.check_link("Wrist_Pitch", [0.422736, 0.33415, 0.653237], [-0.540144, -0.351823, -0.764503,  0.351823, -0.919631, 0.174639, -0.764503, -0.174639, 0.620513])
        self.check_link("Wrist_Roll", [0.422736, 0.33415, 0.653237], [-0.85003, -0.392007, 0.351823,  0.392007, -0.02467, 0.919631, -0.351823, 0.919631, 0.174639])


if __name__ == '__main__':
    import rostest
    rostest.rosrun("openhrp3", 'test_barrett', TestBarrettWam)
