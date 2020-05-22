'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import wipe_forehead
from keyframes import rightBellyToStand
from keyframes import rightBackToStand

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.key = [0] * 1000
        self.i = [0]*1000
        self.time = 0
        self.lasttime = 0
        self.init = [0] * 1000


    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        names = keyframes[0]
        times = keyframes[1]
        keys = keyframes[2]

        if self.lasttime == 0:
            self.lasttime = perception.time
        t = perception.time - self.lasttime
        self.time = self.time + t
        self.lasttime = perception.time

       # self.i = (self.time - times[self.key])/(times[self.key+1])

        for j in range(len(names)):
            if self.key[j] + 1 < len(keys[j]):
                if self.init[j]!=0:
                    self.i[j] = (self.time - times[j][self.key[j]])/(times[j][self.key[j]+1] - times[j][self.key[j]])
                    if self.i[j] > 1:
                        self.key[j] = self.key[j] + 1
                        self.i[self.key[j]] = 0
                    if self.key[j] + 1 < len(keys[j]):
                        if keys[j][self.key[j]][0] == 0:
                            p0 = 0
                        else:
                            p0 = keys[j][self.key[j]][0]
                            #p0 = keys[j][self.key[j]][1][2] / keys[j][self.key[j]][1][1]
                        if keys[j][self.key[j]][1][0] == 0:
                            p1 = 0
                        else:
                            s1 = 0
                            if (keys[j][self.key[j]][2][1] != 0):
                                s1 = 0 #keys[j][self.key[j]][2][2] / keys[j][self.key[j]][2][1]
                            p1 = keys[j][self.key[j]][0] + keys[j][self.key[j]][2][2] + s1*self.i[j]
                            #p1 = keys[j][self.key[j]][2][2] / keys[j][self.key[j]][2][1]
                        if keys[j][self.key[j]+1][2][0] == 0:
                            p2 = 0
                        else:
                            s2 = 0
                            if (keys[j][self.key[j]+1][1][1]!=0):
                                s2 = 0 # keys[j][self.key[j]+1][1][2]/keys[j][self.key[j]+1][1][1]
                            p2 = keys[j][self.key[j]+1][0] + keys[j][self.key[j]+1][1][2] - s2*self.i[j]
                            #p2 = keys[j][self.key[j]+1][1][2] / keys[j][self.key[j]+1][1][1]
                        if keys[j][self.key[j]+1][0] == 0:
                            p3 = 0
                        else:
                            p3 = keys[j][self.key[j] + 1][0]
                            #p3 = keys[j][self.key[j]+1][2][2] / keys[j][self.key[j]+1][2][1]
                        b = (1 - self.i[j]) ** 3 * p0 + 3 * (1 - self.i[j]) ** 2 * self.i[j] * p1 + 3 * (
                                    1 - self.i[j]) * self.i[j] ** 2 * p2 + self.i[j] ** 3 * p3
                        target_joints[names[j]] = b
                else:
                    self.i[j] = self.time / times[j][self.key[j]]
                    if self.i[j] > 1:
                        self.init[j] += 1
                    p0 = 0
                    p1 = 0
                    p2 =0
                    p3 = keys[j][self.key[j]][0]
                    print(names[j] + "::")
                    print (p0)
                    print (p1)
                    print (p2)
                    print (p3)
                    b = (1-self.i[j])**3 * p0 + 3 * (1-self.i[j])**2 * self.i[j] * p1 + 3*(1-self.i[j]) * self.i[j]**2*p2 + self.i[j]**3 * p3
                    target_joints[names[j]] = b

        print(target_joints)
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
