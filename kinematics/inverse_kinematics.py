'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''
import numpy
import math

from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity


def trans(value, d):
    T = identity(4)
    x = 0
    y = 0
    z = 0
    if d == 'x':
        x = value
    if d == 'y':
        y = value
    if d == 'z':
        z = value
    T = numpy.matrix([[1, 0, 0, x],
         [0, 1, 0, y],
         [0, 0, 1, z],
         [0, 0, 0, 1]])
    return T


def rot(angle, d):
    T = identity(4)
    if d == 'x':
        T = numpy.matrix([[1, 0, 0, 0],
                [0, math.cos(angle), -math.sin(angle), 0],
                [0, math.sin(angle), math.cos(angle), 0],
                [0, 0, 0, 1]])
    if d == 'y':
        T = numpy.matrix([[math.cos(angle), 0, math.sin(angle), 0],
                    [0, 1, 0, 0],
                    [-math.sin(angle), 0, math.cos(angle), 0],
                    [0, 0, 0, 1]])

    if d == 'z':
        T = numpy.matrix([[math.cos(angle), -math.sin(angle), 0, 0],
                 [math.sin(angle), math.cos(angle), 0, 0],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])
    return T

class InverseKinematicsAgent(ForwardKinematicsAgent):

    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        if effector_name == "LLeg":
            hipOffset = -0.050
        else:
            hipOffset = 0.050
        lUpperLeg = 0.100
        lLowerLeg = 0.1029
        joint_angles = []

        if effector_name == "LLeg":
            T1 = numpy.matrix([[1, 0, 0, 0],
                         [0, 1, 0, 0.050],
                         [0, 0, 1, -0.085],
                         [0, 0, 0, 1]])
            foot2Hip = numpy.linalg.inv(T1) * numpy.linalg.inv(transform)
        else:
            T1 = numpy.matrix([[1, 0, 0, 0],
                         [0, 1, 0, -0.050],
                         [0, 0, 1, -0.085],
                         [0, 0, 0, 1]])
            foot2Hip = numpy.linalg.inv(T1) * numpy.linalg.inv(transform)


        foot2HipOrth = rot(math.pi/4, 'x') * foot2Hip

        x = foot2HipOrth[0,3]
        y = foot2HipOrth[1,3]
        z = foot2HipOrth[2,3]

        hipOrth2Foot = numpy.linalg.inv(foot2HipOrth)
        ltarns = math.sqrt(hipOrth2Foot[0, 3]**2 + hipOrth2Foot[1, 3]**2 + hipOrth2Foot[2, 3]**2)
        kp = math.acos((lUpperLeg**2 + lLowerLeg**2 - ltarns**2) / (2 * lUpperLeg * lLowerLeg))
        knee_pitch = math.pi - kp
        ankle_pitch_1 = math.acos((lLowerLeg**2 + ltarns**2 - lUpperLeg**2) / (2 * lLowerLeg * ltarns))
        ankle_pitch_2 = math.atan2(x, math.sqrt(y**2 + z**2))
        ankle_pitch = ankle_pitch_1 + ankle_pitch_2
        ankle_roll = math.atan2(y,z)

        thigh2foot = rot(ankle_roll,'x') * rot(ankle_pitch,'y') * trans(lLowerLeg,'z') * rot(knee_pitch,'y') * trans(lUpperLeg,'z')
        hipOrth2Thigh = numpy.linalg.inv(thigh2foot) * hipOrth2Foot

        hip_roll = math.asin(hipOrth2Thigh[2,1]) - math.pi/4
        hip_yaw = math.atan2(-hipOrth2Thigh[0,1], hipOrth2Thigh[1,1])
        hip_pitch = math.atan2(-hipOrth2Thigh[2,0], hipOrth2Thigh[2,2])

        joint_angles.append(hip_yaw)
        joint_angles.append(hip_roll)
        joint_angles.append(hip_pitch)
        joint_angles.append(knee_pitch)
        joint_angles.append(ankle_pitch)
        joint_angles.append(ankle_roll)

        return joint_angles

    def set_keyframe (self, effector_name, joint_angles):
        names = list()
        times = list()
        keys = list()

        if effector_name == "LLeg":
            names.append("LHipYawPitch")
            names.append("LHipRoll")
            names.append("LHipPitch")
            names.append("LKneePitch")
            names.append("LAnklePitch")
            names.append("LAnkleRoll")
        else:
            names.append("RHipYawPitch")
            names.append("RHipRoll")
            names.append("RHipPitch")
            names.append("RKneePitch")
            names.append("RAnklePitch")
            names.append("RAnkleRoll")

        times.append([0.8, 1.2])
        times.append([0.8, 1.2])
        times.append([0.8, 1.2])
        times.append([0.8, 1.2])
        times.append([0.8, 1.2])
        times.append([0.8, 1.2])

        for angle in joint_angles:
            keys.append([[0.0, [3, -0.26667, 0.00000], [3, 0.25333, 0.00000]],[angle, [3, -0.26667, 0.00000], [3, 0.25333, 0.00000]]])

        return names, times, keys

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        joint_angles = self.inverse_kinematics(effector_name, transform)
        self.keyframes = ([], [], [])  # the result joint angles have to fill in
        self.keyframes = self.set_keyframe(effector_name,joint_angles)

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[1, -1] = 0.05
    T[2, -1] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
