import numpy as np
from gym.envs.robotics import rotations, robot_env, utils
from mujoco_py.generated import const
import pdb

def goal_distance(goal_a, goal_b):
    assert goal_a.shape == goal_b.shape
    return np.linalg.norm(goal_a - goal_b, axis=-1)


class FetchEnv(robot_env.RobotEnv):
    """Superclass for all Fetch environments.
    """

    def __init__(
        self, model_path, n_substeps, gripper_extra_height, block_gripper,
        has_object, target_in_the_air, target_offset, obj_range, target_range,
        distance_threshold, initial_qpos, reward_type, num_objs=0,
    ):
        """Initializes a new Fetch environment.

        Args:
            model_path (string): path to the environments XML file
            n_substeps (int): number of substeps the simulation runs on every call to step
            gripper_extra_height (float): additional height above the table when positioning the gripper
            block_gripper (boolean): whether or not the gripper is blocked (i.e. not movable) or not
            has_object (boolean): whether or not the environment has an object
            target_in_the_air (boolean): whether or not the target should be in the air above the table or on the table surface
            target_offset (float or array with 3 elements): offset of the target
            obj_range (float): range of a uniform distribution for sampling initial object positions
            target_range (float): range of a uniform distribution for sampling a target
            distance_threshold (float): the threshold after which a goal is considered achieved
            initial_qpos (dict): a dictionary of joint names and values that define the initial configuration
            reward_type ('sparse' or 'dense'): the reward type, i.e. sparse or dense
        """
        self.gripper_extra_height = gripper_extra_height
        self.block_gripper = block_gripper
        self.target_in_the_air = target_in_the_air
        self.target_offset = target_offset
        self.obj_range = obj_range
        self.target_range = target_range
        self.distance_threshold = distance_threshold
        self.reward_type = reward_type
        self.num_objs = num_objs
        self.has_object = self.num_objs != 0
        self.reward_scale = 0.5

        self.obj = None
        self.num_completed = -1

        super(FetchEnv, self).__init__(
            model_path=model_path, n_substeps=n_substeps, n_actions=4,
            initial_qpos=initial_qpos)

    # GoalEnv methods
    # ----------------------------


    def compute_reward(self, achieved_goal, goal, info):
        # Compute distance between goal and the achieved goal.
        d1 = goal_distance(achieved_goal, goal)
        if self.reward_type == 'sparse':
            return -(d1 > self.distance_threshold).astype(np.float32)
        else:
            if self.has_object:
                gripper_pos = self.sim.data.get_site_xpos('robot0:grip')
                d2 = goal_distance(gripper_pos, achieved_goal)
            else:
                d2 = 0

            return - (d1 + self.reward_scale * d2) 
        
    # RobotEnv methods
    # ----------------------------

    def _step_callback(self):
        if self.block_gripper:
            self.sim.data.set_joint_qpos('robot0:l_gripper_finger_joint', 0.)
            self.sim.data.set_joint_qpos('robot0:r_gripper_finger_joint', 0.)
            self.sim.forward()

    def _set_action(self, action):
        assert action.shape == (4,)
        action = action.copy()  # ensure that we don't change the action outside of this scope
        print(action)
        pos_ctrl, gripper_ctrl = action[:3], action[3]
        pos_ctrl *= 0.05  # limit maximum change in position
        rot_ctrl = [1., 0., 1., 0.]  # fixed rotation of the end effector, expressed as a quaternion
        gripper_ctrl = np.array([gripper_ctrl, gripper_ctrl])
        assert gripper_ctrl.shape == (2,)
        if self.block_gripper:
            gripper_ctrl = np.zeros_like(gripper_ctrl)
        action = np.concatenate([pos_ctrl, rot_ctrl, gripper_ctrl])
        # Apply action to simulation.
        utils.ctrl_set_action(self.sim, action)
        utils.mocap_set_action(self.sim, action)

    def _get_obs(self):
            # positions
            grip_pos = self.sim.data.get_site_xpos('robot0:grip')
            dt = self.sim.nsubsteps * self.sim.model.opt.timestep
            grip_velp = self.sim.data.get_site_xvelp('robot0:grip') * dt
            robot_qpos, robot_qvel = utils.robot_get_obs(self.sim)
            flatten = lambda x: x.flatten().ravel()

            if self.has_object:
                object_pos = np.array([self.sim.data.get_site_xpos('object%s' %n) for n in range(self.num_objs)])
                self.ag = object_pos

                object_rot = np.array([rotations.mat2euler(self.sim.data.get_site_xmat('object%s' %n)) for n in range(self.num_objs)])
                # velocities
                object_velp = np.array([self.sim.data.get_site_xvelp('object%s' %n) * dt for n in range(self.num_objs)])
                object_velr = np.array([self.sim.data.get_site_xvelr('object%s' %n) * dt for n in range(self.num_objs)])
                # gripper state
                object_rel_pos = object_pos - grip_pos
            else:
                object_pos = object_rot = object_velp = object_velr = object_rel_pos = np.zeros(0)
            gripper_state = robot_qpos[-2:]
            gripper_vel = robot_qvel[-2:] * dt  # change to a scalar if the gripper is made symmetric

            if not self.has_object:
                achieved_goal = np.squeeze(grip_pos.copy())
            else:
                achieved_goal = flatten(object_pos.copy())

            obs = np.concatenate([
                grip_pos, flatten(object_pos.copy()), flatten(object_rel_pos.copy()), flatten(gripper_state.copy()), flatten(object_rot.copy()),
                #object_velp.ravel(), object_velr.ravel(), grip_velp, gripper_vel,
            ])
            return {
                'observation': obs.copy(),
                'achieved_goal': achieved_goal.copy(),
                'desired_goal': flatten(self.goal.copy()),
            }


    def _viewer_setup(self, mode='human'):
        if mode != 'rgb_array_table':
            body_id = self.sim.model.body_name2id('robot0:gripper_link')
            lookat = self.sim.data.body_xpos[body_id]
            for idx, value in enumerate(lookat):
                self.viewer.cam.lookat[idx] = value
        else:
            self.viewer.cam.type = const.CAMERA_FIXED
            self.viewer.cam.fixedcamid = 3
        self.viewer.cam.distance = 1.25
        self.viewer.cam.azimuth = 132.
        self.viewer.cam.elevation = -10.

    def _render_callback(self):
        # Visualize target.
        sites_offset = (self.sim.data.site_xpos - self.sim.model.site_pos).copy()
        site_id = list(range(max(1, self.num_objs)))
        for i in range(max(1, self.num_objs)):
            site_id = self.sim.model.site_name2id('target%s' %i)
            self.sim.model.site_pos[site_id] = self.goal[i] - sites_offset[0]
            self.sim.forward()

    def _reset_sim(self):
        self.sim.set_state(self.initial_state)
        # Randomize start position of object.
        if self.has_object:
            for i in range(self.num_objs):
                object_xpos = self.initial_gripper_xpos[:2]

                while np.linalg.norm(object_xpos - self.initial_gripper_xpos[:2]) < 0.1:
                    object_xpos = self.initial_gripper_xpos[:2] + self.np_random.uniform(-self.obj_range, self.obj_range, size=2) 
                object_qpos = self.sim.data.get_joint_qpos('object%d:joint' % i)

                assert object_qpos.shape == (7,)
                object_qpos[:2] = object_xpos
                self.sim.data.set_joint_qpos('object%d:joint' % i, object_qpos)
        
        self.sim.forward()
        return True

    def _sample_goal(self):
        if self.has_object:
            goal = list(range(self.num_objs))
            for i in range(self.num_objs):
                gl = self.initial_gripper_xpos[:3] + self.np_random.uniform(-self.target_range, self.target_range, size=3)
                gl += self.target_offset
                gl[2] = self.height_offset
                if self.target_in_the_air and self.np_random.uniform() < 0.5:
                    gl[2] += self.np_random.uniform(0, 0.45)
                goal[i] = gl.copy()

            goal = np.array(goal).reshape(self.num_objs, -1)
        else:
            goal = np.array([self.initial_gripper_xpos[:3] + self.np_random.uniform(-self.target_range, self.target_range, size=3)])
        return goal.copy()

    def _is_success(self, achieved_goal, desired_goal):
        d = goal_distance(achieved_goal, desired_goal)
        return (d < self.distance_threshold).astype(np.float32)

    def _env_setup(self, initial_qpos):
        for name, value in initial_qpos.items():
            self.sim.data.set_joint_qpos(name, value)
        utils.reset_mocap_welds(self.sim)
        self.sim.forward()

        # Move end effector into position.
        gripper_target = np.array([-0.498, 0.005, -0.431 + self.gripper_extra_height]) + self.sim.data.get_site_xpos('robot0:grip')
        gripper_rotation = np.array([1., 0., 1., 0.])
        self.sim.data.set_mocap_pos('robot0:mocap', gripper_target)
        self.sim.data.set_mocap_quat('robot0:mocap', gripper_rotation)
        for _ in range(10):
            self.sim.step()

        # Extract information for sampling goals.
        self.initial_gripper_xpos = self.sim.data.get_site_xpos('robot0:grip').copy()
        if self.has_object:
            self.height_offset = self.sim.data.get_site_xpos('object0')[2]

    def render(self, mode='human', width=128, height=128):
        return super(FetchEnv, self).render(mode, width, height)

"""
    def get_img(self):
        data = self.render(mode='rgb_array')
        data = np.asarray(data, dtype=np.uint8)
        data = data.reshape([1, 32, 32, 3])
        return self.get_param(data)
        
    edits to get_obs: 
        i = self.selectClosestObject()
        obj = 'object' + str(i)
        object_pos = object_pos[i]
        rotations
        object_rot = rotations.mat2euler(self.sim.data.get_site_xmat(obj))
        velocities
        object_velp = self.sim.data.get_site_xvelp(obj) * dt
        object_velr = self.sim.data.get_site_xvelr(obj) * dt
        gripper state
        object_rel_pos = object_pos - grip_pos
        object_velp -= grip_velp

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

    def compute_reward(self, achieved_goal, goal, info):
        # Compute distance between goal and the achieved goal.
        d1 = goal_distance(achieved_goal, goal)
        if self.reward_type == 'sparse':
            return -(d1 > self.distance_threshold).astype(np.float32)
        else:
            gripper_pos = self.sim.data.get_site_xpos('robot0:grip')
            d2 = goal_distance(gripper_pos, achieved_goal)
            d2_weight = 1
            if d2 < 0.1:
                d2_weight = 0
            else: 
                d2_weight = 1
            return - (d1 + d2_weight * d2) 
"""