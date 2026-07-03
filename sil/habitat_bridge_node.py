"""SIL bridge, CONTAINER side: thin ROS2 node standing in for the RealSense driver.

Talks to habitat_bridge_server.py on the host (--network=host) and publishes exactly what the
camera driver publishes (/camera/camera/color/image_raw + camera_info); subscribes /cmd_vel and
integrates it in the sim. Also publishes /sim/gt_pose for scoring — nothing in the nav stack
consumes it.

Run (inside container, ROS sourced): python3 sil/habitat_bridge_node.py [--rate 20] [--port 5601]
"""
import argparse, pickle, socket, struct
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
import cv2


class Bridge(Node):
    def __init__(self, host, port, rate):
        super().__init__("habitat_bridge")
        self.sock = socket.create_connection((host, port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.pub_img = self.create_publisher(Image, "/camera/camera/color/image_raw", 5)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/camera/color/camera_info", 5)
        self.pub_gt = self.create_publisher(PoseStamped, "/sim/gt_pose", 5)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_cb, 5)
        # teleport: position = habitat xyz, orientation.z = yaw (rad). For episode setup/scoring.
        self.create_subscription(PoseStamped, "/sim/reset_pose", self.reset_cb, 5)
        self.vx = 0.0
        self.wz = 0.0
        self.dt = 1.0 / rate
        r = self.rpc({"reset": {}})
        self.K = r["K"]
        self.get_logger().info(f"bridge up, agent at {r['pos']}")
        self.create_timer(self.dt, self.tick)

    def rpc(self, msg):
        b = pickle.dumps(msg)
        self.sock.sendall(struct.pack(">Q", len(b)) + b)
        hdr = b""
        while len(hdr) < 8:
            hdr += self.sock.recv(8 - len(hdr))
        n = struct.unpack(">Q", hdr)[0]
        buf = b""
        while len(buf) < n:
            buf += self.sock.recv(n - len(buf))
        return pickle.loads(buf)

    def reset_cb(self, m):
        pos = [m.pose.position.x, m.pose.position.y, m.pose.position.z]
        yaw = float(m.pose.orientation.z)
        r = self.rpc({"reset": {"pos": pos, "yaw": yaw}})
        self.vx = 0.0; self.wz = 0.0
        self.get_logger().info(f"teleported to {r['pos']} yaw {r['yaw']:.2f}")

    def cmd_cb(self, m):
        self.vx = float(m.linear.x)
        self.wz = float(m.angular.z)

    def tick(self):
        f = self.rpc({"step": {"vx": self.vx, "wz": self.wz, "dt": self.dt}})
        bgr = cv2.imdecode(np.frombuffer(f["jpg"], np.uint8), cv2.IMREAD_COLOR)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = "camera_color_optical_frame"
        msg.height, msg.width = rgb.shape[0], rgb.shape[1]
        msg.encoding = "rgb8"
        msg.step = rgb.shape[1] * 3
        msg.data = rgb.tobytes()
        self.pub_img.publish(msg)
        ci = CameraInfo()
        ci.header = msg.header
        ci.height, ci.width = rgb.shape[0], rgb.shape[1]
        K = self.K
        ci.k = [K[0][0], 0.0, K[0][2], 0.0, K[1][1], K[1][2], 0.0, 0.0, 1.0]
        ci.p = [K[0][0], 0.0, K[0][2], 0.0, 0.0, K[1][1], K[1][2], 0.0, 0.0, 0.0, 1.0, 0.0]
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0] * 5
        self.pub_info.publish(ci)
        T = np.array(f["gt_pose"])
        gp = PoseStamped()
        gp.header.stamp = stamp
        gp.header.frame_id = "gt_world"
        gp.pose.position.x, gp.pose.position.y, gp.pose.position.z = T[0, 3], T[1, 3], T[2, 3]
        self.pub_gt.publish(gp)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5601)
    ap.add_argument("--rate", type=float, default=20.0)
    a = ap.parse_args()
    rclpy.init()
    n = Bridge(a.host, a.port, a.rate)
    rclpy.spin(n)


if __name__ == "__main__":
    main()
