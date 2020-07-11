'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from SimpleXMLRPCServer import SimpleXMLRPCServer
import xmlrpclib

from inverse_kinematics import InverseKinematicsAgent


class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ServerAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.server = SimpleXMLRPCServer(("localhost", 8000))

        self.server.register_function(self.get_angle, "get_angle")
        self.server.register_function(self.set_angle, "set_angle")

        self.server.register_function(self.get_posture, "get_posture")
        self.server.register_function(self.execute_keyframes, "execute_keyframes")

        self.server.register_function(self.get_transform, "get_transform")
        self.server.register_function(self.set_transform, "set_transform")

        #self.target_joints["LKneePitch"] = 0.5
        self.server.serve_forever()



    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        print joint_name
        return self.target_joints[joint_name]



    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        self.target_joints[joint_name] = angle
        return 0

   # server.register_function(set_angle, "set_angle")

    def get_posture(self):
        '''return current posture of robot'''
        print "kjk"
        perception = self.sense() #todo : fi moshkla henaa
        print "dc"
        print perception
        return self.recognize_posture(perception)


    #server.register_function(get_posture, "get_posture")

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        self.keyframes = keyframes

        #todo: wait till keyframe ends

        return 0

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        super(ServerAgent, self).set_transform(effector_name, transform)
        return 0



if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()

