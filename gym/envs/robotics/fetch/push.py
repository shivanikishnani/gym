import os
from gym import utils
from gym.envs.robotics import fetch_env
from gym.envs.robotics.assets.fetch.xmlCreator import main
import numpy as np

MODEL_XML_PATH = os.path.join('fetch', 'output_push.xml')

class FetchPushEnv(fetch_env.FetchEnv, utils.EzPickle):

    def __init__(self, reward_type='sparse', num_objs=1):

        path = os.path.dirname(os.path.dirname(__file__))
        main(num_objs, path, 'fetch/push.xml')

        initial_qpos = {
            'robot0:slide0': 0.405,
            'robot0:slide1': 0.48,
            'robot0:slide2': 0.0,
        }

        for obj in range(num_objs):
            initial_qpos['object%s:joint' %obj] = [1.25, 0.53, 0.4, 1., 0., 0., 0.]

        fetch_env.FetchEnv.__init__(
            self, MODEL_XML_PATH, has_object=True, block_gripper=True, n_substeps=20,
            gripper_extra_height=0.0, target_in_the_air=False, target_offset=0.0,
            obj_range=0.2, target_range=0.2, distance_threshold=0.05,
            initial_qpos=initial_qpos, reward_type=reward_type, num_objs=num_objs)

        utils.EzPickle.__init__(self)

    def ret_obs(self):
        object_pos = self.sim.data.get_site_xpos('object0')
        gripper_pos = self.sim.data.get_site_xpos('robot0:grip')
        return np.concatenate([object_pos, gripper_pos, self.goal.copy()], axis =0)