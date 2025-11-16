# train_ppo.py
import os, math, time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

class Tb3Env(Node):
    def __init__(self, goal=(3.0, 0.0)):
        super().__init__('tb3_env')
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.reset_cli = self.create_client(SetEntityState, '/set_entity_state')

        self.goal = np.array(goal, dtype=np.float32)
        self.scan = None
        self.pose = np.zeros(3, dtype=np.float32)   # x, y, yaw
        self.collision = False
        self.max_range = 3.5
        self.step_time = 0.15
        self.episode_steps = 0
        self.max_steps = 300

        self.actions = [
            (0.12,  0.8),
            (0.12,  0.0),
            (0.12, -0.8),
            (0.00,  0.8),
            (0.00, -0.8),
        ]

    def _on_scan(self, msg: LaserScan):
        rng = np.array(msg.ranges, dtype=np.float32)
        n = 24
        inds = np.linspace(0, len(rng) - 1, n).astype(int)
        r = np.clip(rng[inds], 0.0, self.max_range)
        self.collision = bool((r < 0.18).any())
        self.scan = r / self.max_range

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.pose[:] = (x, y, yaw)

    def reset(self):
        start = (np.random.uniform(-1.0, 1.0),
                 np.random.uniform(-1.0, 1.0),
                 np.random.uniform(-math.pi, math.pi))
        self.goal = np.array((np.random.uniform(2.0, 3.5),
                              np.random.uniform(-1.0, 1.0)), dtype=np.float32)

        if not self.reset_cli.service_is_ready():
            self.reset_cli.wait_for_service(timeout_sec=5.0)
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = 'tb3'
        req.state.pose.position.x = float(start[0])
        req.state.pose.position.y = float(start[1])
        cy, sy = math.cos(start[2] * 0.5), math.sin(start[2] * 0.5)
        req.state.pose.orientation.z = sy
        req.state.pose.orientation.w = cy
        self.reset_cli.call_async(req)

        self.episode_steps = 0
        self.collision = False

        t0 = time.time()
        while (self.scan is None) or (time.time() - t0 < 0.25):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._obs()

    def step(self, a_idx: int):
        v, w = self.actions[int(a_idx)]
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)

        t_end = self.get_clock().now().nanoseconds + int(self.step_time * 1e9)
        while self.get_clock().now().nanoseconds < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)

        self.episode_steps += 1

        obs = self._obs()
        rew = -0.01  
        done = False

        dist = np.linalg.norm(self.goal - self.pose[:2])
        rew += 0.02 * (1.0 - np.tanh(dist))

        if self.collision:
            rew -= 5.0
            done = True
        if dist < 0.25:
            rew += 10.0
            done = True
        if self.episode_steps >= self.max_steps:
            done = True

        return obs, rew, done, {}

    def _obs(self):
        scan = self.scan if self.scan is not None else np.zeros(24, dtype=np.float32)
        dx, dy = (self.goal - self.pose[:2]).tolist()
        tail = np.array([dx, dy, self.pose[2]], dtype=np.float32)
        return np.concatenate([scan, tail], axis=0)

import gymnasium as gym
from gymnasium import spaces

class GymTb3(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self, node: Tb3Env):
        super().__init__()
        self.node = node
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(27,), dtype=np.float32)
        self.action_space = spaces.Discrete(len(self.node.actions))

    def reset(self, *, seed=None, options=None):
        if seed is not None:
            np.random.seed(seed)
        obs = self.node.reset()
        return obs, {}

    def step(self, action):
        obs, reward, done, info = self.node.step(action)
        terminated = done
        truncated = (self.node.episode_steps >= self.node.max_steps) and not terminated
        return obs, reward, terminated, truncated, info

def main():
    import argparse
    from stable_baselines3 import PPO

    parser = argparse.ArgumentParser()
    parser.add_argument('--timesteps', type=int, default=50_000)
    parser.add_argument('--logdir', type=str, default=os.path.expanduser('~/ppo_runs'))
    args = parser.parse_args()

    rclpy.init()
    node = Tb3Env()
    env = GymTb3(node)

    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=args.logdir,
                n_steps=512, batch_size=256, learning_rate=3e-4)
    model.learn(total_timesteps=args.timesteps)
    os.makedirs(args.logdir, exist_ok=True)
    out = os.path.join(args.logdir, 'tb3_ppo.zip')
    model.save(out)
    print("Saved policy to", out)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
