'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import math
import os
import sys
import math

import numpy

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):


    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       # YOUR CODE HERE
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll']
                       }

    def think(self, perception):
        target_joints = {}
        target_joints["HeadYaw"] = -0.018500490114092827
        target_joints["HeadPitch"] = 0.07670722156763077
        target_joints["LShoulderPitch"] = 1.5323816537857056
        target_joints["LShoulderRoll"] = 0.2618168294429779
        target_joints["LElbowYaw"] = 0.0
        target_joints["LElbowRoll"] = -0.1745329201221466
        target_joints["LHipYawPitch"] = -0.0
        target_joints["LHipRoll"] = -0.0
        target_joints["LHipPitch"] = -0.7175921201705933
        target_joints["LKneePitch"] = 1.3576916456222534
        target_joints["LAnklePitch"] = -0.6400995254516602
        target_joints["LAnkleRoll"] = -0.0
        target_joints["RHipYawPitch"] = -0.0
        target_joints["RHipRoll"] = 0.0
        target_joints["RHipPitch"] = -0.7175921201705933
        target_joints["RKneePitch"] = 1.3576916456222534
        target_joints["RAnklePitch"] = -0.6400995254516602
        target_joints["RAnkleRoll"] = 0.0
        target_joints["RShoulderPitch"] = 1.5323816537857056
        target_joints["RShoulderRoll"] = -0.2618168294429779
        target_joints["RElbowYaw"] = -0.0
        target_joints["RElbowRoll"] = 0.1745329201221466


        self.target_joints.update(target_joints)
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''

        x = {'HeadYaw': 0.0, 'HeadPitch': 0.0, 'LShoulderPitch': 0.0, 'LShoulderRoll': 0.0, 'LElbowYaw': 105.0, 'LElbowRoll': 0.0, 'LWristYaw': 55.0,
        'RShoulderPitch': 0.0, 'RShoulderRoll': 0.0, 'RElbowYaw': 105.0, 'RElbowRoll': 0.0, 'RWristYaw': 55.0, 'LHipYawPitch': 0.0, 'LHipRoll':0.0, 'LHipPitch':0.0, 'LKneePitch':0.0, 'LAnklePitch':0.0,'LAnkleRoll':0.0,
        'RHipYawPitch': 0.0, 'RHipRoll': 0.0, 'RHipPitch': 0.0, 'RKneePitch': 0.0, 'RAnklePitch': 0.0, 'RAnkleRoll': 0.0}

        y =  {'HeadYaw' : 0.0, 'HeadPitch' : 0.0, 'LShoulderPitch': 98.0, 'LShoulderRoll': 0.0 ,'LElbowYaw': 15.0,'LElbowRoll': 0.0, 'LWristYaw': 0.0,
        'RShoulderPitch': 0.0, 'RShoulderRoll': 0.0 ,'RElbowYaw': 15.0,'RElbowRoll': 0.0, 'RWristYaw': 0.0,'LHipYawPitch': 50.0, 'LHipRoll':0.0, 'LHipPitch':0.0, 'LKneePitch':0.0, 'LAnklePitch':0.0,'LAnkleRoll':0.0,
        'RHipYawPitch':50.0, 'RHipRoll':0.0, 'RHipPitch':0.0, 'RKneePitch': 0.0, 'RAnklePitch':0.0,'RAnkleRoll' : 0.0}

        z =  {'HeadYaw' : 0.0, 'HeadPitch' : 0.0, 'LShoulderPitch': 0.0, 'LShoulderRoll': 0.0,'LElbowYaw': 105,'LElbowRoll': 0.0,
        'RShoulderPitch': 0.0, 'RShoulderRoll': 0.0 ,'RElbowYaw': 105,'RElbowRoll': 0.0,'LHipYawPitch': 0.0, 'LHipRoll':0.0, 'LHipPitch':0.0, 'LKneePitch':0.0, 'LAnklePitch':0.0,'LAnkleRoll':0.0,
        'RHipYawPitch': 0.0, 'RHipRoll':0.0, 'RHipPitch':0.0, 'RKneePitch': 0.0, 'RAnklePitch': 0.0,'RAnkleRoll' : 0.0}


        alpha = {'HeadYaw' : 0.0, 'HeadPitch' : -math.pi/2, 'LShoulderPitch': -math.pi/2, 'LShoulderRoll': math.pi/2 ,'LElbowYaw': math.pi/2,'LElbowRoll': -math.pi/2,
        'RShoulderPitch': - math.pi/2, 'RShoulderRoll': math.pi/2,'RElbowYaw': math.pi/2,'RElbowRoll': -math.pi/2,'LHipYawPitch': -3/4 * math.pi, 'LHipRoll': -math.pi/2, 'LHipPitch':math.pi/2, 'LKneePitch': 0.0, 'LAnklePitch':0.0,'LAnkleRoll':-math.pi/2,
        'RHipYawPitch': - math.pi/4, 'RHipRoll': -math.pi/2, 'RHipPitch': math.pi/2, 'RKneePitch': 0.0, 'RAnklePitch': 0.0,'RAnkleRoll' : -math.pi/2}

        a = {'HeadYaw': 0.0, 'HeadPitch': 0.0, 'LShoulderPitch': 0.0,  'LShoulderRoll': 0.0, 'LElbowYaw': 15, 'LElbowRoll': 0.0,
                 'RShoulderPitch': 0.0, 'RShoulderRoll': 0.0, 'RElbowYaw': -15, 'RElbowRoll': 0.0, 'LHipYawPitch': 0.0, 'LHipRoll': 0.0, 'LHipPitch': 0.0, 'LKneePitch': -100.0, 'LAnklePitch': -102.90,
                 'LAnkleRoll': 0.0,'RHipYawPitch': 0.0, 'RHipRoll': 0.0, 'RHipPitch': 0.0, 'RKneePitch': -100.0, 'RAnklePitch': -102.90,'RAnkleRoll': 0.0}

        theta = {'HeadYaw': 0.0, 'HeadPitch': -math.pi/2, 'LShoulderPitch':0.0,  'LShoulderRoll': math.pi/2, 'LElbowYaw': 0.0, 'LElbowRoll': 0.0,
                 'RShoulderPitch': 0.0, 'RShoulderRoll': 0.0, 'RElbowYaw': 0.0, 'RElbowRoll': 0.0, 'LHipYawPitch': -math.pi/2, 'LHipRoll': math.pi/4, 'LHipPitch': 0.0, 'LKneePitch': 0.0, 'LAnklePitch': 0.0,
                 'LAnkleRoll': 0.0,'RHipYawPitch': -math.pi/2, 'RHipRoll': -math.pi/4, 'RHipPitch': 0.0, 'RKneePitch': 0.0, 'RAnklePitch': 0.0,'RAnkleRoll': 0.0}

        T = identity(4)
        T1 = identity(4)
        #joint_angle = joint_angle + alpha[joint_name]
        #if joint_name == "HeadPitch":
        #    joint_angle = joint_angle - math.pi/2
        # YOUR CODE HERE

        joint_angle = joint_angle + theta[joint_name]
        T = [[math.cos(joint_angle), -math.sin(joint_angle) * math.cos(alpha[joint_name]), math.sin(joint_angle) * math.sin(alpha[joint_name]), a[joint_name] * (10 ** -3) * math.cos(joint_angle)],
             [math.sin(joint_angle), math.cos(joint_angle) * math.cos(alpha[joint_name]), -math.cos(joint_angle) * math.sin(alpha[joint_name]), a[joint_name] * (10 ** -3)*math.sin(joint_angle)],
             [0, math.sin(alpha[joint_name]),  math.cos(alpha[joint_name]), z[joint_name] * (10 ** -3)],
             [0, 0, 0, 1]]
        """"
        if joint_name.find('Pitch') != -1:
            T = [[math.cos(joint_angle), 0, math.sin(joint_angle), x[joint_name]* (10**-3)],
                 [0, 1, 0, y[joint_name] * (10**-3)],
                 [-math.sin(joint_angle), 0, math.cos(joint_angle), z[joint_name]* (10**-3)],
                 [0, 0, 0, 1]]
        elif joint_name.find('Yaw') != -1:
            T = [[math.cos(joint_angle), -math.sin(joint_angle), 0, x[joint_name]*(10**-3)],
                 [math.sin(joint_angle), math.cos(joint_angle), 0, y[joint_name]*(10**-3)],
                 [0, 0, 1, z[joint_name] * (10**-3)],
                 [0, 0, 0, 1]]
        else:
            T = [[1, 0, 0, x[joint_name]*(10**-3)],
                 [0, math.cos(joint_angle), -math.sin(joint_angle), y[joint_name]*(10**-3)],
                 [0, math.sin(joint_angle), math.cos(joint_angle), z[joint_name]*(10**-3)],
                 [0, 0, 0, 1]]
        #print (T)
      """""
        if joint_name.find("Head") != -1:
           print (joint_name)
           print (T)
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                T1 = identity(4)
                if joint == "HeadYaw":
                    T1 = [[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0.1265],
                          [0, 0, 0, 1]]
                elif joint == "RShoulderPitch":
                    T1 = [[1, 0, 0, 0],
                          [0, 1, 0, -0.098],
                          [0, 0, 1, 0.100],
                          [0, 0, 0, 1]]
                elif joint == "LShoulderPitch":
                    T1 = [[1, 0, 0, 0],
                          [0, 1, 0, 0.098],
                          [0, 0, 1, 0.100],
                          [0, 0, 0, 1]]
                elif joint == "LHipYawPitch":
                    T1 = [[1, 0, 0, 0],
                          [0, 1, 0, 0.050],
                          [0, 0, 1, -0.085],
                          [0, 0, 0, 1]]
                elif joint == "RHipYawPitch":
                    T1 = [[1, 0, 0, 0],
                          [0, 1, 0, -0.050],
                          [0, 0, 1, -0.085],
                          [0, 0, 0, 1]]
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = T * Tl *T1
                self.transforms[joint] = T



        ry = [[math.cos(math.pi/2), 0,math.sin(math.pi/2), 0],
                 [0, 1, 0, 0],
                 [-math.sin(math.pi/2), 0, math.cos(math.pi/2),0],
                 [0, 0, 0, 1]]
        m =  self.transforms["RElbowRoll"]
        a = numpy.array(m)

        print "x" + repr(a[0][3])
        print "y" + repr(a[1][3])
        print "z" + repr(a[2][3])

        print "Roll" + repr(math.atan2(a[2][1], a[2][2]))
        print "pitch" + repr(math.atan2(-a[2][0], math.sqrt(a[2][1] ** 2 + a[2][2] ** 2)))
        print "yaw" + repr(math.atan2(a[1][0], a[0][0]))
        """
        print "Roll" + repr(math.atan2(a[2][1], a[1][1]))
        print "pitch" + repr(math.atan2(a[0][2],a[0][0]))
        print "yaw" + repr(math.atan2(a[1][0], a[0][0]))
    """
if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()

