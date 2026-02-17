# avdr_gz_worlds

A ROS 2 package providing a classic Gazebo (Gazebo 11 / Gazebo Classic) simulation environment for multi-floor robot navigation with elevator simulation. It includes a 4-floor building world, elevator car models, a C++ Gazebo model plugin for individual elevator control, and a Python ROS 2 node for high-level elevator scheduling and coordination.

## Quick Start

Build the workspace and launch the simulation:

```bash
colcon build --packages-select avdr_gz_worlds
source install/setup.bash
ros2 launch avdr_gz_worlds avdr_building_w_elevator.launch.py
```

The launch file automatically sets `GAZEBO_PLUGIN_PATH` — no manual export needed. The elevator scheduler node starts automatically after a 16-second delay to allow Gazebo and the elevator plugins to fully initialise.

### Launch arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `gz_headless_mode` | `False` | Run without GUI (`True` starts `gzserver` only) |
| `gz_log_level` | `2` | Gazebo console verbosity (0–4; ≥3 enables verbose output) |

**Example — headless mode:**
```bash
ros2 launch avdr_gz_worlds avdr_building_w_elevator.launch.py gz_headless_mode:=True
```

## Custom Elevator Services

Each elevator exposes per-elevator services under the `/elevator_<id>/` namespace. The scheduler node exposes the following high-level group-control services:

### `request_elevator`

Request the best available elevator to travel to a given floor and direction.

| Field | Type | Description |
|-------|------|-------------|
| `a`   | int  | From floor (0–3) |
| `b`   | int  | Direction: `1` (up) or `-1` (down) |

**Returns:** elevator ID assigned, or `-1` if none available.

```bash
ros2 service call /request_elevator example_interfaces/srv/AddTwoInts "{a: 0, b: 1}"
```

---

### `bring_all_to_floor`

Send all elevators to a specified floor and hold them there.

| Field | Type | Description |
|-------|------|-------------|
| `a`   | int  | Target floor (0–3) |

**Returns:** number of elevators successfully commanded.

```bash
ros2 service call /bring_all_to_floor example_interfaces/srv/AddTwoInts "{a: 0, b: 0}"
```

---

### `release_all_elevators`

Release all elevators from hold mode, returning them to autonomous operation.

```bash
ros2 service call /release_all_elevators example_interfaces/srv/AddTwoInts "{a: 0, b: 0}"
```

**Returns:** number of elevators successfully released.
