import os
from gym import utils
from gym.envs.robotics import point_env
import numpy as np
from gym.envs.robotics.assets.point.xmlCreator import main
from gym.envs.robotics.assets.point import xmlCreator #e gym.envs.robotics.assets.point.xmlCreator
from IPython import embed
import pdb


# Ensure we get the path separator correct on windows
MODEL_XML_PATH = os.path.join('point', 'output.xml')
#MODEL_INITIAL_PATH = os.path.join('fetch', 'initialxml.txt')


class PointMassEnv(point_env.PointEnv, utils.EzPickle):
    def __init__(self, reward_type='sparse', num_objs=1, num_obstacles=0):
        path = os.path.dirname(os.path.dirname(__file__))
        main(num_objs, path, 'point/point.xml')
        self.gRange = xmlCreator.gRange

        self.deterministic = False
        self.fixed_obj = False
        self.fixed_goal = False

        self.actionRange = 0.025
        self.target_range = self.gRange - 0.08


        if self.deterministic:
            initial_qpos = {
                'agent0_x': 0,
            }
        else:
            initial_qpos = {
            'agent0_x': 0,
            'agent0_y': 0,
            }

        for i in range(num_objs):
            initial_qpos['object%d:joint'%i] = [0.02, 0.04, 0.025, 1., 0., 0., 0.]
        
        for i in range(num_obstacles):
            pass

        point_env.PointEnv.__init__(
            self, MODEL_XML_PATH, n_substeps=20,
            distance_threshold=0.05,
            initial_qpos=initial_qpos, reward_type=reward_type, num_objs=num_objs)
        utils.EzPickle.__init__(self)


    def ret_obs(self):
        object_pos = np.array([self.sim.data.get_site_xpos('object%s' %n)[0:2] for n in range(self.num_objs)]).flatten()
        goal = np.array([self.goal[i, :-1] for i in range(max(1,self.num_objs))]).flatten()
        res =  np.concatenate([object_pos, goal.flatten()])
        return res

    