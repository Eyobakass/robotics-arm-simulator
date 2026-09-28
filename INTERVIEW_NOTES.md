# Interview Preparation Notes

> For: **Junior/Entry-Level Part-Time Robotics Simulation Developer**
> Location: Addis Ababa | Schedule: 12 hours/week

---

## 1. UDP vs TCP

| Feature | UDP | TCP |
|---|---|---|
| Connection | Connectionless | Connection-oriented (3-way handshake) |
| Reliability | No guarantee of delivery | Guaranteed delivery + retransmission |
| Ordering | No ordering guarantee | Ordered byte stream |
| Latency | Lower — no handshake, no retransmission | Higher — waits for ACKs, retransmits |
| Overhead | 8-byte header | 20+ byte header |
| Use case | Real-time data (robotics, video, gaming) | Reliable data (web, file transfer) |

### Why UDP Is Appropriate for Robotics

- **Low latency matters more than reliability.** A dropped frame is better than a delayed frame.
- **Latest data wins.** If packet #5 is lost, packet #6 already has newer joint angles. Retransmitting #5 is wasteful.
- **Simple for embedded systems.** A C microcontroller can call `sendto()` without managing connections.
- **No connection overhead.** The sender can start transmitting immediately.
- **Industry standard.** Many real-time robotic protocols use UDP (EtherCAT, some ROS transports).

---

## 2. How the Packet Reaches Python

```
C/Python Sender                    Python Simulator
─────────────────                 ──────────────────
1. socket(AF_INET, SOCK_DGRAM)   5. socket(AF_INET, SOCK_DGRAM)
2. sprintf(buf, JSON_FORMAT, ...) 6. bind(("127.0.0.1", 9999))
3. sendto(sock, buf, len, dest)   7. recvfrom(4096) → raw bytes
4. OS network stack delivers      8. bytes.decode("utf-8") → string
   the datagram                   9. json.loads(string) → dict
                                  10. Validate → JointUpdate dataclass
```

Key points:
- Each UDP datagram is independent — no session state
- `recvfrom()` returns the complete datagram (no fragmentation at our size)
- The socket timeout (0.1s) allows the receive loop to check a shutdown flag

---

## 3. How Joint Angles Are Parsed

```python
raw_bytes → UTF-8 string → json.loads() → Python dict → JointUpdate
```

**Validation steps:**
1. Decode bytes as UTF-8 (catch `UnicodeDecodeError`)
2. Parse JSON (catch `json.JSONDecodeError`)
3. Check top-level is a `dict`
4. Extract `"joints"` field, verify it's a `dict`
5. For each joint value: verify `isinstance(v, (int, float))` and **not** `bool`
6. Non-numeric values are silently skipped
7. If no valid joints remain → return `None`

**Design choice:** Return `None` on any error rather than raising exceptions.
This keeps the real-time loop running — one bad packet shouldn't crash the simulator.

---

## 4. How Forward Kinematics Works

**Forward kinematics (FK)** converts joint angles into 3D positions.

### Algorithm

```
Initialize:
    position = top of base
    R_cumulative = Identity matrix (3×3)

For each joint in the chain:
    1. Get rotation matrix for this joint: R = R_axis(angle)
    2. Update cumulative rotation: R_cumulative = R_cumulative × R
    3. Compute link direction in world frame:
       direction = R_cumulative × [0, link_length, 0]
    4. Move to end of link: position = position + direction
    5. Store position and orientation
```

### Why It Works

- Each joint **rotates the coordinate frame** of all subsequent links
- Matrix multiplication **chains rotations** — `R_cumulative` tracks the total rotation
- The link direction `[0, L, 0]` is the link pointing "up" in local coordinates
- Multiplying by `R_cumulative` transforms it to world coordinates

### Example

All angles zero → arm points straight up:
```
Base top:  [0, 0.3, 0]
After j1:  [0, 0.3, 0] + [0, 1.5, 0] = [0, 1.8, 0]
After j2:  [0, 1.8, 0] + [0, 1.2, 0] = [0, 3.0, 0]
After j3:  [0, 3.0, 0] + [0, 1.0, 0] = [0, 4.0, 0]
```

Shoulder = 90°:
```
R_cumulative after j2 = Rx(90°)
Rx(90°) × [0, 1.2, 0] = [0, 0, 1.2]  ← link now points along Z
Position: [0, 1.8, 0] + [0, 0, 1.2] = [0, 1.8, 1.2]
```

---

## 5. How Vectors and Trigonometry Are Used

### Rotation Matrices

Each rotation matrix uses `cos(θ)` and `sin(θ)` to rotate a vector:

```
Rx(θ): rotates around X → affects Y and Z components
Ry(θ): rotates around Y → affects X and Z components
Rz(θ): rotates around Z → affects X and Y components
```

### Key Vector Operations

| Operation | Code | Purpose |
|---|---|---|
| Matrix × vector | `R @ [0, L, 0]` | Transform local direction to world |
| Vector addition | `pos + direction` | Move to end of link |
| Matrix × matrix | `R_cumul @ R_joint` | Chain rotations |
| Degrees → radians | `math.radians(θ)` | Trig functions need radians |

### Properties of Rotation Matrices
- **Orthogonal:** `R × Rᵀ = I` (inverse = transpose)
- **Determinant = 1** (preserves handedness)
- **Preserve length:** `|R × v| = |v|`

---

## 6. How JSON Configuration Makes the Simulator Reusable

```
config/demo_arm.json   →   config_loader.py   →   RobotConfig   →   RobotModel
```

**Benefits:**
- **No code changes** to add joints, change lengths, or adjust limits
- **Factory pattern:** `RobotModel(config)` builds any valid configuration
- **Validation at load time:** bad configs are caught early with clear error messages
- **Separation of concerns:** geometry data vs. simulation logic
- **Multiple robots:** create `config/6dof_arm.json`, pass with `--config`

**What the JSON defines:**
- Base platform geometry (width, height, depth, color)
- Ordered list of joints (name, axis, link_length, limits, visual properties)
- End effector geometry

---

## 7. What a C Embedded Application Would Send

```c
// 1. Create UDP socket
int sock = socket(AF_INET, SOCK_DGRAM, 0);

// 2. Set destination
struct sockaddr_in dest;
dest.sin_family = AF_INET;
dest.sin_port = htons(9999);
inet_pton(AF_INET, "127.0.0.1", &dest.sin_addr);

// 3. Read joint encoders (in a real system)
double base_angle = read_encoder(0);
double shoulder   = read_encoder(1);
double elbow      = read_encoder(2);

// 4. Format JSON with sprintf
char buf[512];
snprintf(buf, sizeof(buf),
    "{\"robot\":\"demo_arm\",\"timestamp\":%ld,"
    "\"joints\":{\"base_rotation\":%.2f,"
    "\"shoulder\":%.2f,\"elbow\":%.2f}}",
    time(NULL), base_angle, shoulder, elbow);

// 5. Send
sendto(sock, buf, strlen(buf), 0,
       (struct sockaddr*)&dest, sizeof(dest));
```

This is exactly what `sender/c_sender.c` implements with animation.

---

## 8. Likely Interview Questions & Model Answers

### Q: "Why did you choose UDP over TCP?"

> "Real-time robotics needs low latency. If a joint-angle packet is lost,
> the next one arrives in ~33ms with fresher data anyway. TCP's retransmission
> would add unnecessary delay. UDP also matches what embedded C systems use —
> just `sendto()` with no connection management."

### Q: "How would you handle packet loss?"

> "The simulator always uses the latest received packet. If one is lost,
> the arm simply holds its current position for one more frame (~33ms).
> The next packet arrives with up-to-date angles. In real-time control,
> latest-wins is the standard approach."

### Q: "Explain forward kinematics in simple terms."

> "Forward kinematics answers: 'Given these joint angles, where is the
> end of the arm?' Starting from the base, I apply each joint's rotation
> as a matrix multiplication, then extend the link in the rotated direction.
> Chaining all joints gives me the end effector position."

### Q: "How would you add a new robot?"

> "Create a new JSON config file defining the joints — names, axes, link
> lengths, and limits. Run the simulator with `--config my_robot.json`.
> No code changes needed because the FK engine handles any number of
> serial joints."

### Q: "What OOP patterns did you use?"

> "**Joint** encapsulates a single revolute joint with angle clamping.
> **RobotModel** composes joints into a kinematic chain. **UDPServer**
> encapsulates threaded socket communication. **VPythonRenderer** handles
> 3D visualization. Each class has a single responsibility and a clean
> interface."

### Q: "What happens when invalid data arrives?"

> "The packet parser returns `None` for any error — invalid UTF-8, malformed
> JSON, wrong types. The receive loop simply discards it and waits for the
> next packet. Valid joint angles that exceed limits are clamped rather than
> rejected, so the arm moves to the nearest safe position."

### Q: "How does threading work here?"

> "The UDP server runs in a daemon thread, continuously calling `recvfrom()`.
> When a valid packet arrives, it stores the parsed `JointUpdate` behind a
> `threading.Lock`. The main thread (VPython's event loop at 60fps) calls
> `get_latest_update()` to consume it. The lock prevents race conditions."

### Q: "What is a rotation matrix?"

> "A 3×3 orthogonal matrix that rotates a vector around an axis without
> changing its length. For angle θ around the X axis, it applies cos(θ)
> and sin(θ) to the Y and Z components. Key properties: determinant is 1,
> and the inverse equals the transpose."

### Q: "Why VPython over alternatives?"

> "VPython is lightweight, requires no GPU setup, and opens directly in the
> browser. It's designed for physics and engineering visualization — perfect
> for a quick 3D robotics demo without the complexity of Unity or OpenGL."

### Q: "How would you extend this to 6-DOF?"

> "Add three more joint entries to the JSON config file. The forward
> kinematics engine already handles any number of joints in a serial chain.
> No code changes are needed — just data."

### Q: "What would you do differently in production?"

> "I'd add message authentication (HMAC) on UDP packets, proper error
> logging with rotation to files, inverse kinematics for target-position
> control, a web dashboard for monitoring, and collision detection between
> links."

---

## 9. 30-Second Explanation

> "This is a real-time 3D robotic arm simulator I built with Python. A C or
> Python program sends joint angles over UDP. The Python simulator parses the
> JSON packets, computes forward kinematics using rotation matrices and
> trigonometry, and renders the moving arm in VPython — all in the browser.
> Everything is JSON-configurable: change the config file to simulate a
> different robot with no code changes."

---

## 10. 2-Minute Explanation

> "I built this project to demonstrate skills relevant to robotics simulation
> development.
>
> **The problem:** We need to visualize a robotic arm that receives real-time
> joint angles from an embedded C controller.
>
> **Architecture:** A sender program — written in either C or Python — sends
> joint angles as JSON over UDP to localhost port 9999. I chose UDP because
> in real-time robotics, low latency matters more than guaranteed delivery.
> If a packet is lost, the next one arrives in 33 milliseconds with newer data.
>
> **On the simulator side,** a threaded UDP server receives the raw bytes,
> decodes them as UTF-8, and parses the JSON into a validated JointUpdate
> object. Invalid packets are silently discarded to keep the loop running.
>
> **The joint angles feed into a forward kinematics engine.** Starting from
> the base, I chain 3×3 rotation matrices — Rx, Ry, Rz — for each joint.
> Each link extends along the local Y axis, and the cumulative rotation
> transforms it into world coordinates. The result is the 3D position of
> every joint and the end effector.
>
> **VPython renders the arm** in a browser-based 3D scene with a floor grid,
> coordinate axes, colored links, joint spheres, and a green end-effector cone.
> It updates at 60 frames per second.
>
> **The key design decision** was making everything JSON-configurable. The
> robot's geometry — link lengths, joint axes, angular limits, even colors —
> all come from a config file. To simulate a 6-DOF arm, you just create a
> new JSON file. No code changes.
>
> **I wrote 75 unit tests** covering packet parsing, joint validation,
> configuration loading, and forward kinematics correctness — verified with
> manual trigonometric calculations.
>
> **What I learned:** How UDP socket programming works at the system level,
> how rotation matrices chain to produce forward kinematics, and how to
> design a data-driven system that separates configuration from logic."

---

## Quick Reference Card

| Command | What it does |
|---|---|
| `python src/main.py` | Start the simulator |
| `python sender/python_sender.py` | Send animated joint angles |
| `python sender/python_sender.py --mode sweep` | Sweep one joint at a time |
| `python -m unittest discover tests/ -v` | Run all 75 tests |
| `python src/main.py --config config/my_robot.json` | Use custom robot |
| `gcc sender/c_sender.c -o c_sender.exe -lws2_32` | Compile C sender (Windows) |
