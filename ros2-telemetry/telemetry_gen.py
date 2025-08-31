#!/usr/bin/env python3
import json, math, random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime, timezone

class Telemetry(Node):
    def __init__(self):
        super().__init__('telemetry_gen')
        self.pub = self.create_publisher(String, '/robot/telemetry', 10)
        self.t = 0.0
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        self.t += 0.5
        x = 10 + 2*math.sin(self.t/5)
        y =  5 + 2*math.cos(self.t/5)
        msg = {
            "robot_id": "go2-1",
            "state": random.choice(["idle","enroute","investigating","charging"]),
            "battery": max(0, 90 - int(self.t)),
            "pose": {"x": x, "y": y, "theta": 0.0, "frame": "map"},
            "last_update": datetime.now(timezone.utc).isoformat()
        }
        out = String()
        out.data = json.dumps(msg)
        self.pub.publish(out)

def main():
    rclpy.init()
    node = Telemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
