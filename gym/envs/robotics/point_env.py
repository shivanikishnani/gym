import numpy as np
from gym.envs.robotics import rotations, utils, super_env
from mujoco_py.generated import const
import glfw
import mujoco_py
import pdb
import glfw

def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)


class PointEnv(super_env.SuperEnv):
    """Superclass for all Point environments.
    """

    def __init__(
        self, model_path, n_substeps,
        distance_threshold,initial_qpos, reward_type, num_objs=2, has_object=True, cnn=False,
    ):
        """Initializes a Point Mass environment.

        Args:
            model_path (string): path to the environments XML file
            n_substeps (int): number of substeps the simulation runs on every call to step
            distance_threshold (float): the threshold after which a goal is considered achieved
            initial_qpos (dict): a dictionary of joint names and values that define the initial configuration
            reward_type ('sparse' or 'dense'): the reward type, i.e. sparse or dense
        """
        self.distance_threshold = distance_threshold
        self.reward_type = reward_type
        self.obj = None
        self.num_completed = -1
        self.num_objs = num_objs
        self.target_offset = 0
        self.has_object = has_object
        self.cnn = cnn
        self.has_object = self.num_objs > 0
        self.frame_skip = 1

        self.rw_scale = 1

        if self.deterministic:
            self.n_actions = 1
        else:
            self.n_actions = 2

        super(PointEnv, self).__init__(
            model_path=model_path, n_substeps=n_substeps,
            initial_qpos=initial_qpos)

    # GoalEnv methods
    # ----------------------------

    def compute_reward(self, achieved_goal, goal, info, agent_pos=None):
        # Compute distance between goal and the achieved goal.
        d1 = goal_distance(achieved_goal, goal)
        if self.reward_type == 'sparse':
            return -(d1 > self.distance_threshold).astype(np.float32)
        else:
            if not self.has_object:
                return -d1
            d2 = 0
            for i in range(self.num_objs):
                d2 += goal_distance(achieved_goal[i * 3 : (i + 1) * 3], agent_pos)
            # if d2 < self.distance_threshold:
            #     d2_weight = 0
            # else:
            #     d2_weight = 1
            # if d1 < self.distance_threshold:
            #     d2_weight = 0
            d2_weight = 0.5
            return - (d1 + d2_weight * d2)

    # RobotEnv methods
    # ----------------------------

    def _step_callback(self):
        #TODO
        self.sim.forward()

    def _set_action(self, action):
        assert action.shape == (self.n_actions,)
        action = action.copy()  # ensure that we don't change the action outside of this scope
        for i in range(self.frame_skip):
            self.sim.data.set_joint_qvel("agent0_x", action[0])
            if not self.deterministic:
                self.sim.data.set_joint_qvel("agent0_y", action[1])
            self.do_simulation(np.zeros(self.sim.model.nu), 1)
        self.sim.forward()


    def _get_obs(self):
        flatten = lambda x : x.copy().flatten().ravel()
        # positions
        agent_pos = self.sim.data.body_xpos[self.agent_id].copy()
        dt = self.sim.nsubsteps * self.sim.model.opt.timestep

        #v_x = self.sim.data.get_joint_qpos("agent0_x").copy() * dt #self.sim.data.get_joint_qvel("agent0_x").copy() #v_y = self.sim.data.get_joint_qpos("agent0_y").copy() * dt ##self.sim.data.get_joint_qvel("agent0_y").copy() #agent_vel = np.array([v_x, v_y, 0])

        # ag = agent_pos.ravel()
        if self.has_object:
            object_pos = np.array([self.sim.data.get_site_xpos('object%s' %n) for n in range(self.num_objs)])
            self.ag = object_pos
            #i = self.selectClosestObject()#obj = 'object' + str(i)#object_pos = object_pos[i]# rotations
            object_rot = np.array([rotations.mat2euler(self.sim.data.get_site_xmat('object%s' %n)) for n in range(self.num_objs)])

            # velocities
            object_velp = np.array([self.sim.data.get_site_xvelp('object%s' %n) * dt for n in range(self.num_objs)])
            object_velr = np.array([self.sim.data.get_site_xvelr('object%s' %n) * dt for n in range(self.num_objs)])

            # gripper state
            object_rel_pos = object_pos - agent_pos
            #object_velp -= agent_vel
        else:
            object_pos = object_rot = object_velp = object_velr = object_rel_pos = np.zeros(0)

        if not self.has_object:
            achieved_goal = flatten(agent_pos)
        else:
            achieved_goal = flatten(object_pos)

        obs = np.concatenate([
            flatten(agent_pos), flatten(object_pos), flatten(object_rel_pos), flatten(object_rot),
            flatten(object_velp), flatten(object_velr), #agent_vel.flatten().ravel(),
        ])

        # if self.cnn:
        #     obs = np.flatten(self.render(mode='rgb_array', width=128, height=128))
        #     return {
        #     'achieved_goal': obs.copy(),
        #     'observation': obs.copy(),
        #     'desired_goal': self.goal_img.copy()
        #     }


        if self.cnn:
            obs = self.render(mode='rgb_array', width=128, height=128) #maybe flatten this?
            goal = self.goal.flatten()
            achieved_goal = ag.flatten() #desired_goal = self.goal_img.copy()


        return {
            'observation': obs.copy(),
            'achieved_goal': achieved_goal,
            'desired_goal': flatten(self.goal),
        }

    def _viewer_setup(self, mode='human'):
        # self.viewer.cam.distance = self.sim.model.stat.extent * 3.5
        # self.viewer.cam.trackbodyid = self.agent_id
        self.viewer.cam.type = const.CAMERA_FIXED
        self.viewer.cam.fixedcamid = 0


    def _render_callback(self):
        #render targets
        for i in range(max(1, self.num_objs)):
            site_id = self.sim.model.site_name2id('target%s' %i)
            self.sim.model.site_pos[site_id] = self.goal[i * 3 : (i + 1) * 3] # - sites_offset[0]
            self.sim.forward()

    def get_object_xy(self, id):
        return self.sim.data.get_joint_qpos('object%d:joint' % id)[:2]

    def _reset_sim(self):
        self.sim.set_state(self.initial_state)
        # Randomize start position of object.
        if self.has_object:
            for i in range(self.num_objs):
                obj_id = self.sim.model.geom_name2id('object%i' % i)
                object_size = self.sim.model.geom_size[obj_id][0]

                object_xpos = self.initial_agent_position[:2] / 2

                if self.deterministic:
                    n = self.np_random.uniform(self.gRange - 0.05, 0, size=1)[0]
                    while n > self.goal.flatten()[0]:
                        n = self.np_random.uniform(self.gRange - 0.05, 0, size=1)[0]

                    object_xpos = np.array([np.array([n, 0])])
                elif self.fixed_obj:
                    object_xpos = np.array([np.array([-0.1, 0])])
                else:
                    while (np.linalg.norm(object_xpos - self.initial_agent_position[:2]) < min(0.2, self.gRange / 4)):
                        object_xpos = self.np_random.uniform(-self.target_range, self.target_range, size=2)

                object_qpos = self.sim.data.get_joint_qpos('object%d:joint' % i)
                assert object_qpos.shape == (7,)
                object_qpos[:2] = object_xpos
                self.sim.data.set_joint_qpos('object%d:joint' % i, object_qpos)

        action = np.array([0, 0])
        utils.ctrl_set_action(self.sim, action)
        self.sim.forward()
        return True

    def _sample_goal(self):
        if self.deterministic:
            pos = np.array([np.array([self.np_random.uniform(0, self.gRange - 0.05, size=1)[0], 0, 0]) for _ in range(max(1, self.num_objs))])
            return pos.copy()
        if self.fixed_goal:
            pos = np.array([np.array([self.gRange - 0.05, self.gRange - 0.05, 0]) for _ in range(max(1, self.num_objs))])
            return pos.copy()
        num = max(1, self.num_objs)
        goal = list(range(num))
        for i in range(max(1, self.num_objs)):
            gl = self.initial_agent_position[:3] + self.np_random.uniform(-self.target_range, self.target_range, size=3)
            gl += self.target_offset
            gl[2] = self.height_offset
            goal[i] = gl
        goal = np.array(goal).flatten() #reshape(num, -1)

        return goal.copy()


    def _is_success(self, achieved_goal, desired_goal):
        d = np.array(goal_distance(achieved_goal, desired_goal))
        return (d < self.distance_threshold).astype(np.float32)

    def _env_setup(self, initial_qpos):
        for name, value in initial_qpos.items():
            self.sim.data.set_joint_qpos(name, value)
        self.sim.forward()
        self.agent_id = self.sim.model.body_name2id('agent0')
        self.object_ids = [self.sim.model.body_name2id('object%d' %i) for i in range(self.num_objs)]
        self.initial_agent_position = self.sim.data.body_xpos[self.agent_id].copy()

        self.height_offset = self.sim.data.get_site_xpos('target0')[2]

    def render(self, mode='human', width=128, height=128):
        return super(PointEnv, self).render(mode, width, height)

    def set_agent0_xy(self, goal_pos):
        # if self.has_object:
        #     self.sim.data.set_joint_qpos('agent0_x', 60)
        #     self.sim.data.set_joint_qpos('agent0_y', 60)
        # else:
        self.sim.data.set_joint_qpos('agent0_x', goal_pos[0])
        self.sim.data.set_joint_qpos('agent0_y', goal_pos[1])
        self.sim.forward()

    def set_object_xy(self, goal_pos):
        for i in range(self.num_objs):                
            object_qpos[i] = self.sim.data.get_joint_qpos('object%d:joint' % i)
            assert object_qpos[i].shape == (7,)
            object_qpos[i][:2] = goal_pos[i]
            self.sim.data.set_joint_qpos('object%d:joint' % i, object_qpos[i])
        self.sim.forward()




""" 
    def selectClosestObject(self):
        if self.num_objs == 1:
            self.obj = 0
            return self.obj
        target = self.goal.copy()
        norm = np.linalg.norm(self.ag - target, axis =1)
        if self.obj is None:
            self.num_completed += 1
            self.obj = np.argmin(norm)
        if norm[self.obj] < self.distance_threshold:
            self.num_completed = (self.num_completed + 1) % self.num_objs
            if self.num_completed >= self.num_objs:
                return self.obj
            self.obj = np.argpartition(norm, self.num_completed)[self.num_completed]
        return self.obj

"""

