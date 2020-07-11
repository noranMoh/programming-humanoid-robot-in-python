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

#from angle_interpolation import AngleInterpolationAgent

from recognize_posture import PostureRecognitionAgent

class ForwardKinematicsAgent(PostureRecognitionAgent):


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
        """"
        target_joints = {}
        target_joints["HeadYaw"] = -0.018500490114092827
        target_joints["HeadPitch"] = 0.07670722156763077
        target_joints["LShoulderPitch"] = 1.4679789543151855
        target_joints["LShoulderRoll"] = -0.08894197642803192
        target_joints["LElbowYaw"] = -0.6841865181922913
        target_joints["LElbowRoll"] = -1.2118170261383057
        target_joints["LHipYawPitch"] = -0.2637890577316284
        target_joints["LHipRoll"] = -0.07878416031599045
        target_joints["LHipPitch"] = -0.6887767314910889
        target_joints["LKneePitch"] = 2.0018577575683594
        target_joints["LAnklePitch"] = -1.1412882804870605
        target_joints["LAnkleRoll"] = 0.0783652812242508
        target_joints["RHipYawPitch"] = -0.2722539007663727
        target_joints["RHipRoll"] = 0.0711570754647255
        target_joints["RHipPitch"] = -0.6965259909629822
        target_joints["RKneePitch"] = 2.018787384033203
        target_joints["RAnklePitch"] = -1.1493691205978394
        target_joints["RAnkleRoll"] = - 0.09470156580209732
        target_joints["RShoulderPitch"] = 1.5769747495651245
        target_joints["RShoulderRoll"] = - 0.030752701684832573
        target_joints["RElbowYaw"] = 0.9004851579666138
        target_joints["RElbowRoll"] = 1.2426046133041382
        self.target_joints.update(target_joints)
        """

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
        'RShoulderPitch': 0.0, 'RShoulderRoll': 0.0, 'RElbowYaw': 105.0, 'RElbowRoll': 0.0, 'LHipYawPitch': 0.0, 'LHipRoll':0.0, 'LHipPitch':0.0, 'LKneePitch':0.0, 'LAnklePitch':0.0,'LAnkleRoll':0.0,
        'RHipYawPitch': 0.0, 'RHipRoll': 0.0, 'RHipPitch': 0.0, 'RKneePitch': 0.0, 'RAnklePitch': 0.0, 'RAnkleRoll': 0.0}

        y =  {'HeadYaw' : 0.0, 'HeadPitch' : 0.0, 'LShoulderPitch': 0.0, 'LShoulderRoll': 0.0 ,'LElbowYaw': 15.0,'LElbowRoll': 0.0, 'LWristYaw': 0.0,
        'RShoulderPitch': 0.0, 'RShoulderRoll': 0.0 ,'RElbowYaw': -15.0,'RElbowRoll': 0.0,'LHipYawPitch': 0.0, 'LHipRoll':0.0, 'LHipPitch':0.0, 'LKneePitch':0.0, 'LAnklePitch':0.0,'LAnkleRoll':0.0,
        'RHipYawPitch': 0.0, 'RHipRoll':0.0, 'RHipPitch':0.0, 'RKneePitch': 0.0, 'RAnklePitch':0.0,'RAnkleRoll' : 0.0}

        z =  {'HeadYaw' : 0.0, 'HeadPitch' : 0.0, 'LShoulderPitch': 0.0, 'LShoulderRoll': 0.0,'LElbowYaw': 0.0,'LElbowRoll': 0.0,
        'RShoulderPitch': 0.0, 'RShoulderRoll': 0.0,'RElbowYaw': 0.0,'RElbowRoll': 0.0,'LHipYawPitch': 0.0, 'LHipRoll':0.0, 'LHipPitch':0.0, 'LKneePitch':-100.00, 'LAnklePitch': -102.9,'LAnkleRoll':0.0,
        'RHipYawPitch': 0.0, 'RHipRoll':0.0, 'RHipPitch':0.0, 'RKneePitch': -100.00, 'RAnklePitch': -102.9,'RAnkleRoll' : 0.0}

        T = identity(4)

        if joint_name.find("LHipYawPitch")!= -1:
           T = matrix([[math.cos(joint_angle), -math.sin(joint_angle), 0, x[joint_name]*(10**-3)],
                 [math.sin(joint_angle), math.cos(joint_angle), 0, y[joint_name]*(10**-3)],
                 [0, 0, 1, z[joint_name] * (10**-3)],
                 [0, 0, 0, 1]])
           T =  T * matrix([[math.cos(joint_angle), 0, math.sin(joint_angle), x[joint_name]* (10**-3)],
                 [0, 1, 0, y[joint_name] * (10**-3)],
                 [-math.sin(joint_angle), 0, math.cos(joint_angle), z[joint_name]* (10**-3)],
                 [0, 0, 0, 1]])
        elif joint_name.find("RHipYawPitch") != -1:
            T = matrix([[math.cos(joint_angle), -math.sin(joint_angle), 0, x[joint_name] * (10 ** -3)],
                        [math.sin(joint_angle), math.cos(joint_angle), 0, y[joint_name] * (10 ** -3)],
                        [0, 0, 1, z[joint_name] * (10 ** -3)],
                        [0, 0, 0, 1]])
            T = T * matrix([[math.cos(joint_angle), 0, math.sin(joint_angle), x[joint_name] * (10 ** -3)],
                            [0, 1, 0, y[joint_name] * (10 ** -3)],
                            [-math.sin(joint_angle), 0, math.cos(joint_angle), z[joint_name] * (10 ** -3)],
                            [0, 0, 0, 1]])

        elif joint_name.find("Pitch") != -1: #pitch
            T = matrix([[math.cos(joint_angle), 0, math.sin(joint_angle), x[joint_name]* (10**-3)],
                 [0, 1, 0, y[joint_name] * (10**-3)],
                 [-math.sin(joint_angle), 0, math.cos(joint_angle), z[joint_name]* (10**-3)],
                 [0, 0, 0, 1]])
        elif joint_name.find("ShoulderRoll") != -1 or joint_name.find("ElbowRoll") != -1 or joint_name == "HeadYaw": #Yaw
            T = matrix([[math.cos(joint_angle), -math.sin(joint_angle), 0, x[joint_name]*(10**-3)],
                 [math.sin(joint_angle), math.cos(joint_angle), 0, y[joint_name]*(10**-3)],
                 [0, 0, 1, z[joint_name] * (10**-3)],
                 [0, 0, 0, 1]])
        else: #roll    # HipRoll ,
            T = matrix([[1, 0, 0, x[joint_name]*(10**-3)],
                 [0, math.cos(joint_angle), -math.sin(joint_angle), y[joint_name]*(10**-3)],
                 [0, math.sin(joint_angle), math.cos(joint_angle), z[joint_name]*(10**-3)],
                 [0, 0, 0, 1]])
        #print (T)

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
                    T1 = matrix([[1, 0.0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0.1265],
                          [0, 0, 0, 1]])
                elif joint == "RShoulderPitch":
                    T1 = matrix([[1, 0, 0, 0],
                          [0, 1, 0, -0.098],
                          [0, 0, 1, 0.100],
                          [0, 0, 0, 1]])
                elif joint == "LShoulderPitch":
                    T1 = numpy.matrix([[1, 0, 0, 0],
                          [0, 1, 0, 0.098],
                          [0, 0, 1, 0.100],
                          [0, 0, 0, 1]])
                elif joint == "LHipYawPitch":
                    T1 = matrix([[1, 0, 0, 0],
                          [0, 1, 0, 0.050],
                          [0, 0, 1, -0.085],
                          [0, 0, 0, 1]])
                elif joint == "RHipYawPitch":
                    T1 = matrix([[1, 0, 0, 0],
                          [0, 1, 0, -0.050],
                          [0, 0, 1, -0.085],
                          [0, 0, 0, 1]])

                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                T = T1 * T * Tl
                self.transforms[joint] = T

        """
        m = self.transforms["RAnkleRoll"]
        a = numpy.array(m)

        print "x: " + repr(a[0][3])
        print "y: " + repr(a[1][3])
        print "z: " + repr(a[2][3])

        print "Roll: " + repr(math.atan2(a[2][1], a[2][2]))
        print "pitch: " + repr(math.atan2(-a[2][0], math.sqrt(a[2][1] ** 2 + a[2][2] ** 2)))
        print "yaw: " + repr(math.atan2(a[1][0], a[0][0]))
      """
if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()

