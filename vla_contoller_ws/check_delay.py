import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateLagChecker(Node):
    def __init__(self):
        super().__init__("joint_state_lag_checker")
        self.prev_stamp = None
        self.prev_arrival = None
        self.create_subscription(JointState, "/joint_states", self.cb, 10)

    def cb(self, msg: JointState):
        now = time.time()  # vla 코드와 같은 기준
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        lag = now - stamp
        stamp_dt = 0.0 if self.prev_stamp is None else stamp - self.prev_stamp
        arrival_dt = 0.0 if self.prev_arrival is None else now - self.prev_arrival

        print(
            f"lag_sec={lag:.3f} | "
            f"now={now:.3f} | "
            f"stamp={stamp:.3f} | "
            f"stamp_dt={stamp_dt:.3f} | "
            f"arrival_dt={arrival_dt:.3f}"
        )

        self.prev_stamp = stamp
        self.prev_arrival = now

rclpy.init()
node = JointStateLagChecker()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
