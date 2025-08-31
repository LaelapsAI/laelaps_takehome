import express from "express";
import cors from "cors";
import morgan from "morgan";
import http from "http";

const app = express();
app.use(express.json({ limit: "1mb" }));
app.use(cors());
app.use(morgan("dev"));

const server = http.createServer(app);
const ROS2_BRIDGE_URL = process.env.ROS2_BRIDGE_URL || "http://localhost:8081";

// TODO: Implement the REST API and WebSocket logic below

const PORT = process.env.PORT || 8080;
server.listen(PORT, () => {
  console.log(`Backend listening on :${PORT}`);
  console.log(`ROS2 bridge: ${ROS2_BRIDGE_URL}`);
});
