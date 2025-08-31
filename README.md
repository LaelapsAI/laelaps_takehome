# Laelaps — Take‑Home (Founding SWE)

This repository is a minimal starter kit for a short take‑home used to evaluate system design and backend engineering skills. The goal is to deliver a thin, well‑documented slice end‑to‑end that integrates with ROS2.

High level

- You will implement three pieces in this repo:
  1. ros2-bridge (FastAPI + rclpy) — HTTP ↔ ROS2 bridge. Implement the ROS2 node lifecycle and the HTTP endpoints that publish/subscribe to ROS2 topics.
  2. backend-node (Node.js) — platform backend: ingest events, dispatch robots via the bridge, and surface status. Implement REST endpoints and optional live updates.
  3. frontend — tiny operator UI to list events and robot telemetry. Improve or replace as you like.

What’s already in the repo

- Skeletons for the three services:
  - `ros2-bridge/` — FastAPI app (`app.py`) and a Dockerfile. The bridge currently contains placeholders where ROS2 logic should be implemented.
  - `backend-node/` — minimal Node.js app (`src/index.js`) with TODOs; package.json and a Dockerfile are present.
  - `frontend/` — simple static UI (`index.html`) and a Dockerfile served by nginx.
  - `video/` — configuration files for video streaming services (e.g., MediaMTX, FFmpeg).
- A `docker-compose.yml` that builds the three services (and a few helper services in some branches). Prefer using compose for the canonical dev environment.

Candidate tasks (must implement)

- ros2-bridge
  - Initialize rclpy and create a node that:
    - Publishes dispatch commands to `/robot/dispatch` (std_msgs/String) when `POST /dispatch` is called.
    - Subscribes to `/robot/telemetry` and maintains latest telemetry per robot, exposed at `GET /telemetry`.
    - Ensure the bridge spins ROS2 correctly in the FastAPI lifecycle (startup/shutdown).
- backend-node
  - Implement REST endpoints:
    - POST /event — validate and persist an event (in‑memory is fine for the exercise).
    - POST /events/:id/assign — send a dispatch JSON to the bridge (`/dispatch`) and update event state/assignment.
    - GET /status — return current events and robots.
  - Optional but recommended: WebSocket or SSE for live updates and basic input validation.
- frontend

  - Implement or improve the UI in `/frontend` so it consumes the backend `/status` and shows events + robot telemetry.
  - The frontend should allow triggering an assign/dispatch flow (calls `POST /events/:id/assign`).
  - Video streaming implementation on the frontend with design notes in DESIGN.md.

- Tests & CI (OPTIONAL, bonus)
  - Adding simple tests and a CI workflow is encouraged but NOT required. If you choose to include them, a small smoke-test script (POST /event → POST /events/:id/assign → GET /status) and a GitHub Actions workflow that runs it are sufficient and will be considered a bonus.

Example payloads

- Event (POST /event)

```json
{
  "id": "evt_123",
  "source": {
    "system": "genetec",
    "cameraId": "cam-7",
    "cameraName": "Gate A"
  },
  "type": "motion",
  "severity": "medium",
  "timestamp": "2025-01-01T12:34:56Z",
  "location": { "zone": "gate-a", "lat": 47.3769, "lng": 8.5417 },
  "media": { "rtsp": "rtsp://mediamtx:8554/cam" }
}
```

- Dispatch (Platform → ROS2 /robot/dispatch)

```json
{
  "event_id": "evt_123",
  "task": "investigate",
  "waypoint": { "x": 12.3, "y": 4.5, "frame": "map" },
  "priority": "normal"
}
```

- Telemetry (ROS2 → Platform /robot/telemetry)

```json
{
  "robot_id": "go2-1",
  "state": "enroute",
  "battery": 82,
  "pose": { "x": 11.9, "y": 4.7, "theta": 1.57, "frame": "map" },
  "last_update": "2025-01-01T12:36:10Z"
}
```

How to run (recommended)

- Build and run everything with Docker Compose (from repo root):

  ```bash
  DOCKER_DEFAULT_PLATFORM=linux/arm64 docker compose up --build
  ```

- Services and ports (compose builds these):
  - ros2-bridge → http://localhost:8081
  - backend-node → http://localhost:8080
  - frontend → http://localhost:8082 (served by nginx in the container)
  - camera feed → http://localhost:8889/cam/ (RTSP + WebRTC via MediaMTX)

Development tips

- ros2-bridge requires ROS2 Python packages; running it in the provided Dockerfile is recommended rather than running it locally.
- For backend iteration you can run locally:
  ```bash
  cd backend-node
  npm install
  npm run dev
  ```
  The backend will try to call the bridge at `http://localhost:8081` when running locally.
- Frontend MUST be implemented and can be edited directly in `frontend/index.html` or replaced with a small build pipeline (Vite/React) if you prefer.

What we evaluate

- Correctness: the API behaves as specified and interacts with the bridge.
- Clarity: code is readable, documented, and the candidate describes design decisions in `DESIGN.md`.
- Engineering judgement: simple reliability choices (retries, idempotency, timeouts), clear trade‑offs.
- Bonus: Tests & CI — tests and CI are a bonus if provided.
- Bonus: live updates, frontend polish.

Deliverables

- Your repo (or a branch) containing:
  - Implemented `ros2-bridge`, `backend-node`, and `frontend` changes.
  - `DESIGN.md` describing your architecture, potential issues and trade‑offs. Furthermore, include details on how would you scale the system in real-world usage (e.g., 1000s of events/day, 10s of robots, distributed in different locations).

# Submission instructions

1. Create your solution

   - Use this repository as a template (or fork/clone).
   - Implement changes on a branch named `feature/takehome`.

2. What to include

   - Completed code for `ros2-bridge`, `backend-node`, and `frontend`.
   - Updated `README.md` with exact run instructions.
   - Completed `DESIGN.md`.
   - Screenshots or a short GIF showing the UI and a successful dispatch flow.

3. How to deliver

   - Push your branch to a GitHub repo (public or private).
   - Open a PR and add a short description of what you implemented and trade-offs.
   - Share the repo link with me.

4. Helpful notes
   - Prefer small, focused commits with clear messages.
   - Do not include secrets or credentials.
   - If something is intentionally incomplete, state it clearly in the PR and DESIGN.md.
