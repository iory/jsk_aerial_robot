#!/usr/bin/env python
"""Run the learned hover / takeoff / landing policy on the CPU at 200 Hz.

The network comes from scripts/export_policy.py of /mnt/workspace/gimbalrotor_mjlab as a
TorchScript file and, next to it, the same weights as a .npz; with the .npz present (or
~backend:=numpy) the node needs numpy only, which is what the VIM4 runs.

The controller plugin aerial_robot_control/gimbalrotor_policy_controller publishes
the policy's observation on ``policy/observation`` (std_msgs/Float32MultiArray,
30 values, see gimbalrotor_mjlab/env_cfg.py) every control step; this node maps
it through the network and answers on ``policy/command`` with the 8 normalized
outputs (thrust / hover_thrust - 1 for the 4 rotors, gimbal angle / limit for
the 4 gimbals). The plugin converts them to thrusts [N] and gimbal angles [rad],
and falls back to its PID whenever a command is older than ``policy/timeout``.

The observation's "last command" slots (indices 18-25) are what the plugin
applied at the previous step, so switching PID <-> policy hands the network a
consistent history.

  rosrun gimbalrotor policy_node.py _policy_file:=/path/to/policy_torchscript.pt
"""

import os
import threading

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray


class NumpyMlp(object):
    """The exported network (scripts/export_policy.py writes <policy>.npz next to the
    TorchScript: observation mean / std, then Linear + ELU layers) in numpy, so the
    robot needs no torch."""

    def __init__(self, path):
        data = np.load(path)
        self.mean = data["obs_mean"].astype(np.float32)
        self.std = data["obs_std"].astype(np.float32) + np.float32(data["obs_eps"])
        n = int(data["num_layers"])
        self.layers = [(data["w%d" % i].astype(np.float32), data["b%d" % i].astype(np.float32))
                       for i in range(n)]
        self.obs_dim = self.mean.shape[0]

    def __call__(self, obs):
        h = (obs - self.mean) / self.std
        last = len(self.layers) - 1
        for i, (w, b) in enumerate(self.layers):
            h = w.dot(h) + b
            if i < last:
                h = np.where(h > 0, h, np.expm1(h))   # ELU
        return h


class TorchModule(object):
    def __init__(self, path, threads):
        import torch
        self.torch = torch
        torch.set_num_threads(threads)
        self.module = torch.jit.load(path, map_location="cpu")
        self.module.eval()

    def __call__(self, obs):
        with self.torch.no_grad():
            return self.module(self.torch.from_numpy(obs).unsqueeze(0))[0].numpy()


class PolicyNode(object):
    INTEGRAL_SLICE = slice(14, 17)   # after gravity 3, rates 3, pose error 8

    def __init__(self):
        policy_file = rospy.get_param("~policy_file")
        backend = rospy.get_param("~backend", "auto")   # auto | numpy | torch
        self.threads = int(rospy.get_param("~torch_threads", 2))
        self.obs_dim = int(rospy.get_param("~obs_dim", 0))   # 0: what the network takes
        self.action_dim = int(rospy.get_param("~action_dim", 8))
        npz = os.path.splitext(policy_file)[0] + ".npz"
        if backend == "numpy" or (backend == "auto" and os.path.exists(npz)):
            self.model = NumpyMlp(npz)
            backend = "numpy"
            if self.obs_dim == 0:
                self.obs_dim = self.model.obs_dim
            elif self.model.obs_dim != self.obs_dim:
                raise RuntimeError("%s takes %d observations, the node is set to %d"
                                   % (npz, self.model.obs_dim, self.obs_dim))
        else:
            self.model = TorchModule(policy_file, self.threads)
            backend = "torch"
            if self.obs_dim == 0:
                self.obs_dim = 30
        self.lock = threading.Lock()
        self.count = 0
        self.last_latency = 0.0
        self.pub = rospy.Publisher("policy/command", Float32MultiArray, queue_size=1)
        self.model(np.zeros(self.obs_dim, dtype=np.float32))   # warm up
        self.sub = rospy.Subscriber("policy/observation", Float32MultiArray, self.callback,
                                    queue_size=1, tcp_nodelay=True)
        rospy.loginfo("policy_node: %s (%s backend), %d -> %d",
                      policy_file, backend, self.obs_dim, self.action_dim)
        rospy.Timer(rospy.Duration(5.0), self.report)

    def callback(self, msg):
        t0 = rospy.Time.now()
        obs = np.asarray(msg.data, dtype=np.float32)
        if obs.shape[0] == self.obs_dim + 3:
            # the controller always adds the position error integral (indices 14-16); this network predates it
            obs = np.delete(obs, self.INTEGRAL_SLICE)
        if obs.shape[0] != self.obs_dim:
            rospy.logwarn_throttle(1.0, "policy_node: observation has %d values, expected %d",
                                   obs.shape[0], self.obs_dim)
            return
        if not np.all(np.isfinite(obs)):
            rospy.logwarn_throttle(1.0, "policy_node: observation is not finite, no command")
            return
        action = self.model(obs)
        if not np.all(np.isfinite(action)):
            rospy.logwarn_throttle(1.0, "policy_node: the network gave a non-finite output, no command")
            return
        out = Float32MultiArray()
        out.data = [float(v) for v in action]
        self.pub.publish(out)
        with self.lock:
            self.count += 1
            self.last_latency = (rospy.Time.now() - t0).to_sec()

    def report(self, _event):
        with self.lock:
            rospy.loginfo_throttle(5.0, "policy_node: %d commands, last inference %.2f ms",
                                   self.count, self.last_latency * 1000.0)


if __name__ == "__main__":
    rospy.init_node("policy_node")
    PolicyNode()
    rospy.spin()
