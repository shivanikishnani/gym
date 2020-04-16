    def get_diagnostics(self, paths, prefix=""):
        statistics = OrderedDict()
        stat_names = ['agent_pos', 'is_success']
        if self.has_object:
            stat_names = ['object%i' % i + '_distance' for i in range(self.num_objs)] + ['is_success']

        for stat_name in stat_names:
            stat = get_stat_in_paths(paths, 'env_infos', stat_name)
            statistics.update(create_stats_ordered_dict(
                '%s%s' % (prefix, stat_name),
                stat,
                always_show_all_stats=True,
            ))
            statistics.update(create_stats_ordered_dict(
                'Final %s%s' % (prefix, stat_name),
                [s[-1] for s in stat],
                always_show_all_stats=True,
            ))
        return statistics


    """
    MULTITASK FUNCTIONS
    """
    @property
    def goal_dim(self) -> int:
        return 3 * max(1, self.num_objs)

    def sample_goals(self, batch_size):
        goal_dim = 3 * max(1, self.num_objs)
        goals = np.random.uniform(
            -self.range_x,
            self.range_x,
            size=(batch_size, goal_dim),
        )
        for i in range(max(1, self.num_objs)):
            goals[:, i * 3 + 2] = self.height_offset
        return {
            'desired_goal': goals,
            'state_desired_goal': goals,
        }


    def sample_goal_for_rollout(self):
        g = self.sample_goal()
        return g

    def set_to_goal(self, goal):
        state_goal = goal['state_desired_goal']
        if self.has_object:
            self.set_object_xy(state_goal)
        self.set_agent0_xy(state_goal)



    def set_object_xy(self, goal_pos):
        for i in range(self.num_objs):                
            object_qpos[i] = self.sim.data.get_joint_qpos('object%d:joint' % i)
            assert object_qpos[i].shape == (7,)
            object_qpos[i][:2] = goal_pos[i * 3: i * 3 + 2]
            self.sim.data.set_joint_qpos('object%d:joint' % i, object_qpos[i])

    def compute_her_reward_np(self, ob, action, next_ob, goal, env_info=None):
        return self.compute_reward(ob, action, next_ob, goal, env_info=env_info)

    def get_image(self, width=None, height=None):
        return self.render('rgb_array', height = height, width=width)

    def get_env_state(self):
        return self._get_obs()

    def set_env_state(self, state):
        position = state["observation"]
        goal = state["desired_goal"]
        self._position = position
        self._target_position = goal

    def set_to_goal(self, goal_dict):
        goal = goal_dict["desired_goal"]
        self._position = goal
        self._target_position = goal

        # """Returns a black and white image"""
        # if self.drawer is None:
        #     if width != height:
        #         raise NotImplementedError()
        #     self.drawer = PygameViewer(
        #         screen_width=width,
        #         screen_height=height,
        #         x_bounds=(-self.range_x, self.range_x),
        #         y_bounds=(-self.range_x, self.range_x),
        #         render_onscreen=False,
        #     )
        # self.draw(self.drawer, False)
        # img = self.drawer.get_image()
        # if self.images_are_rgb:
        #     return img.transpose((1, 0, 2))
        # else:
        #     r, g, b = img[:, :, 0], img[:, :, 1], img[:, :, 2]
        #     img = (-r + b).transpose().flatten()
        #     return img

    def initialize_camera(self, init_fctn):
        sim = self.sim
        viewer = mujoco_py.MjRenderContextOffscreen(sim, device_id=-1)
        # viewer = mujoco_py.MjViewer(sim)
        viewer.cam.type = const.CAMERA_FIXED
        viewer.cam.fixedcamid = 0
        # iinit_fctn(viewer.cam)
        sim.add_render_context(viewer)