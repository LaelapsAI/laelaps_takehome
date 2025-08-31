# FastAPI HTTP <-> ROS2 bridge
import asyncio
import json
from datetime import datetime, timezone

from fastapi import FastAPI, Body
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
import uvicorn

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

app = FastAPI(title="ROS2 Bridge")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


class BridgeNode(Node):
    def __init__(self):
        super().__init__("laelaps_bridge")
        self.dispatch_pub = self.create_publisher(String, "/robot/dispatch", 10)
        self.telemetry_sub = self.create_subscription(
            String, "/robot/telemetry", self.on_telemetry, 10
        )
        self.latest = {}  # robot_id -> telemetry dict

    def on_telemetry(self, msg: String):
        try:
            data = json.loads(msg.data)
            rid = data.get("robot_id")
            if rid:
                self.latest[rid] = data
        except Exception as e:
            self.get_logger().warn(f"bad telemetry: {e}")


# TODO: Implement the ROS2-Bridge logic here. Don't forget to take care of the ROS2 lifecycle.


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8081)
