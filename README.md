# floor_cleaner — autonomous floor-cleaning robot (ROS 2 Humble)

A simulated differential-drive floor-cleaning robot with a 2D LiDAR that can

1. drive around a Gazebo apartment,
2. build a 2D map with **SLAM Toolbox**,
3. save that map,
4. localise itself on the saved map with **AMCL**,
5. drive autonomously to any goal you click in **RViz2** with **Nav2**, avoiding obstacles,
6. run a simple **zig-zag cleaning pattern** by sending Nav2 goals one after another.

Everything runs on one Ubuntu 22.04 computer. No hardware needed. The structure is
chosen so that later only the *simulation* pieces have to be swapped for a Raspberry Pi,
a real LiDAR and a motor controller (see section 19).

---

## Contents

1. [What each part does](#1-what-each-part-does)
2. [ROS 2 architecture](#2-ros-2-architecture)
3. [Folder structure](#3-folder-structure)
4. [Dependencies](#4-dependencies)
5. [Installation](#5-installation)
6. [Building](#6-building)
7. [Running Gazebo](#7-running-gazebo)
8. [Running SLAM](#8-running-slam)
9. [Moving the robot](#9-moving-the-robot)
10. [Saving the map](#10-saving-the-map)
11. [Starting Nav2](#11-starting-nav2)
12. [Setting the initial pose](#12-setting-the-initial-pose)
13. [Sending a Nav2 Goal](#13-sending-a-nav2-goal)
14. [Understanding the RViz display](#14-understanding-the-rviz-display)
15. [Troubleshooting](#15-troubleshooting)
16. [Checking TF](#16-checking-tf)
17. [Checking the LiDAR](#17-checking-the-lidar)
18. [Checking odometry](#18-checking-odometry)
19. [Moving to real hardware (Raspberry Pi)](#19-moving-to-real-hardware-raspberry-pi)
20. [Zig-zag cleaning](#20-zig-zag-cleaning)
21. [Test checklist](#21-test-checklist)

---

## 1. What each part does

| Piece | Job |
|-------|-----|
| **Gazebo** | Simulates the physical robot, its wheels, its LiDAR and the apartment. |
| **robot_state_publisher** | Reads the robot description (URDF) and publishes where every part of the robot is (TF). |
| **SLAM Toolbox** | *Creates the map* from LiDAR scans while you drive around. |
| **map_server** | Loads a saved map from disk and publishes it on `/map`. |
| **AMCL** | *Determines where the robot is* on a saved map (localisation). |
| **Nav2** | *Plans and controls navigation*: global path, local obstacle avoidance, recovery behaviours. |
| **cleaning_pattern_node** | *Decides which locations should be cleaned* and sends them to Nav2 as goals. |
| **RViz2** | *Visualises the whole system* and lets you click a start pose and goals. |

Robot facts (all consistent across URDF, Gazebo plugins, SLAM, Nav2 and RViz):

| Item | Value |
|------|-------|
| Chassis | 0.45 m long, 0.35 m wide, 0.15 m tall, 2 cm ground clearance |
| Drive wheels | radius 0.075 m, separation 0.30 m |
| Casters | two frictionless spheres, front and back |
| LiDAR | 360°, 360 samples, 0.12–12 m, 10 Hz, 0.19 m above the floor |
| Topics | `/scan` `/odom` `/cmd_vel` `/map` `/joint_states` `/tf` `/tf_static` |
| Frames | `map` → `odom` → `base_link` → `base_footprint`, `laser`, `chassis`, wheels |
| Max speed (Nav2) | 0.30 m/s, 1.2 rad/s |

---

## 2. ROS 2 architecture

```text
                  ┌───────────────┐
                  │    Gazebo     │
                  │ Robot + World │
                  └───────┬───────┘
                          │
                ┌─────────┴─────────┐
                │                   │
             LiDAR               Odometry (diff drive plugin)
                │                   │
              /scan          /odom + TF odom->base_link
                │                   │
                └─────────┬─────────┘
                          │
                ┌─────────▼─────────┐
                │   SLAM Toolbox    │   (mapping mode)
                │  -> /map + TF     │
                │     map->odom     │
                └─────────┬─────────┘
                          │  map_saver_cli -> my_map.yaml / my_map.pgm
                          ▼
             ┌────────────────────────────┐
             │ map_server + AMCL          │   (navigation mode)
             │ -> /map + TF map->odom     │
             └────────────┬───────────────┘
                          │  Localisation
                     ┌────▼─────────────────────────────┐
  RViz "Nav2 Goal" ─►│ Nav2: bt_navigator               │◄── cleaning_pattern_node
  (NavigateToPose)   │       planner_server  (/plan)    │    (sequence of goals)
                     │       controller_server          │
                     │       behavior_server            │
                     └────┬─────────────────────────────┘
                          │
                       /cmd_vel
                          │
                     ┌────▼────┐
                     │ Robot   │  (Gazebo diff drive plugin, later: real motor controller)
                     └─────────┘
```

TF tree (who publishes what — **exactly one publisher per transform**):

```text
map ──────────► odom ──────────► base_link ──► base_footprint   (robot_state_publisher)
  SLAM Toolbox     Gazebo diff drive       ├──► chassis ──► front_caster, rear_caster
  OR AMCL          plugin                  ├──► left_wheel, right_wheel  (from /joint_states)
  (never both)                             └──► laser
```

Full data flow (validated when designing the project):

```text
Gazebo LiDAR → /scan → SLAM Toolbox / AMCL → /map + TF map->odom → Nav2 costmaps
→ planner (/plan) → controller (/local_plan) → /cmd_vel → Gazebo diff drive → robot moves
→ /odom + TF odom->base_link update → RViz displays everything
```

---

## 3. Folder structure

```text
floor_cleaner_ws/
└── src/
    └── floor_cleaner/
        ├── package.xml                 # package name + dependencies
        ├── CMakeLists.txt              # installs the folders below
        ├── README.md                   # this file
        ├── launch/
        │   ├── simulation.launch.py    # Gazebo + robot_state_publisher + spawn
        │   ├── slam.launch.py          # SLAM Toolbox + RViz2
        │   ├── navigation.launch.py    # map_server + AMCL + Nav2 + RViz2
        │   ├── full_system.launch.py   # simulation + (slam | navigation) in one go
        │   └── cleaning.launch.py      # zig-zag cleaning node
        ├── urdf/
        │   ├── floor_cleaner.urdf.xacro  # main robot description
        │   ├── materials.xacro           # RViz colours
        │   ├── inertial_macros.xacro     # mass/inertia formulas
        │   ├── lidar.xacro               # LiDAR link + simulated LiDAR sensor
        │   └── gazebo_control.xacro      # SIM ONLY: diff drive + joint state plugins
        ├── worlds/
        │   └── floor_cleaner_world.world # apartment: hall + 4 rooms + furniture
        ├── config/
        │   ├── slam_toolbox.yaml
        │   ├── nav2_params.yaml
        │   └── cleaning_pattern.yaml
        ├── rviz/
        │   └── floor_cleaner.rviz
        ├── maps/
        │   └── README.md               # your saved maps go here
        └── scripts/
            └── cleaning_pattern_node.py
```

Notes on the layout:

* There is no `models/` folder: the robot is spawned straight from the URDF and the
  world defines its walls/furniture inline, so no extra model files are needed (and
  Gazebo never tries to download models from the internet).
* The Gazebo-only parts (`gazebo_control.xacro`, the `<sensor>` block in `lidar.xacro`)
  are switched off with `use_sim:=false`, which is what you will use on the real robot.

---

## 4. Dependencies

Ubuntu 22.04 + ROS 2 Humble (desktop) must already be installed
(<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html>).

Then install everything this project uses:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-teleop-twist-keyboard \
  ros-humble-tf2-tools \
  ros-humble-rviz2 \
  python3-colcon-common-extensions
```

What they are for:

| Package | Used for |
|---------|----------|
| `ros-humble-gazebo-ros-pkgs` | Gazebo Classic 11 + ROS plugins (diff drive, LiDAR, spawn) |
| `ros-humble-xacro` | turning `.xacro` into URDF |
| `ros-humble-robot-state-publisher` | robot TF from the URDF |
| `ros-humble-slam-toolbox` | mapping |
| `ros-humble-navigation2` | map_server, AMCL, planner, controller (DWB), behaviours, BT navigator, lifecycle manager, Nav2 RViz plugins, `nav2_msgs` |
| `ros-humble-teleop-twist-keyboard` | driving the robot with the keyboard |
| `ros-humble-tf2-tools` | `view_frames` |
| `python3-colcon-common-extensions` | `colcon build` |

No pip packages are needed.

---

## 5. Installation

Put the project in your home folder so that the path is exactly
`~/floor_cleaner_ws/src/floor_cleaner` (the launch files use this path for the saved map).

If you downloaded the zip:

```bash
cd ~
unzip floor_cleaner_ws.zip
ls ~/floor_cleaner_ws/src/floor_cleaner
```

Make sure the Python node is executable (unzip normally keeps this, copying by hand may not):

```bash
chmod +x ~/floor_cleaner_ws/src/floor_cleaner/scripts/cleaning_pattern_node.py
```

Optional but recommended — source ROS automatically in every new terminal:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

---

## 6. Building

```bash
cd ~/floor_cleaner_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` means edits to launch/yaml/xacro/rviz files take effect without
rebuilding. You only need to rebuild when you **add** new files.

**Every new terminal** needs:

```bash
source /opt/ros/humble/setup.bash
source ~/floor_cleaner_ws/install/setup.bash
```

---

## 7. Running Gazebo

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/floor_cleaner_ws/install/setup.bash
ros2 launch floor_cleaner simulation.launch.py
```

The Gazebo window opens with the apartment and the blue robot in the hall at (0, 0),
facing east (+x). The white strip is the robot's front.

Quick check in another terminal:

```bash
ros2 topic list
```

must include `/clock /cmd_vel /joint_states /odom /robot_description /scan /tf /tf_static`.

Move it with a single command (drives forward slowly, **keeps driving** until told otherwise):

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Stop it:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Turn on the spot:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

(The Gazebo diff drive plugin keeps executing the last command it received. A real
motor controller should instead stop automatically when commands stop arriving — see section 19.)

Launch options: `gui:=false` (no Gazebo window, much lighter), `x_pose:=`, `y_pose:=`, `yaw:=`.

---

## 8. Running SLAM

Keep Gazebo running. Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source ~/floor_cleaner_ws/install/setup.bash
ros2 launch floor_cleaner slam.launch.py
```

RViz2 opens (Fixed Frame = `map`). After a few seconds you see the robot, red LiDAR
points and the first part of the map (white = free, black = walls, grey = unknown).

Check that the map exists:

```bash
ros2 topic echo /map --once --field info
```

---

## 9. Moving the robot

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keep **this terminal focused** while pressing keys:

```text
   u    i    o          i = forward        , = backward
   j    k    l          j = turn left      l = turn right
   m    ,    .          k = STOP
z / x = decrease / increase all speeds by 10 %
```

Press `z` about 6 times first (to ~0.27 m/s) — slow driving gives a much cleaner map.

Mapping tips:

* Drive through the hall and into all four rooms (doors are 1.2 m wide).
* Turn slowly; fast spins smear the map.
* Revisit the start area at the end so SLAM Toolbox can close the loop.
* Watch RViz: the grey unknown area should turn white everywhere you want to clean.

---

## 10. Saving the map

While SLAM is still running, in any sourced terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/floor_cleaner_ws/src/floor_cleaner/maps/my_map
```

You get:

* `my_map.pgm` — the map image (one pixel = 5 cm; white free, black occupied, grey unknown)
* `my_map.yaml` — resolution, origin and thresholds that tell Nav2 how to read the image

Check:

```bash
ls -l ~/floor_cleaner_ws/src/floor_cleaner/maps/
cat ~/floor_cleaner_ws/src/floor_cleaner/maps/my_map.yaml
```

Then **stop SLAM**: press `Ctrl+C` in Terminal 3 (this also closes its RViz).
Stop teleop too (`Ctrl+C` in Terminal 4). **Keep Gazebo running.**

> Why stop SLAM? SLAM Toolbox and AMCL both publish `map -> odom`. Running both
> gives two conflicting publishers and the robot "jumps" around.

---

## 11. Starting Nav2

Gazebo still running, SLAM stopped. Terminal 3 (or a new one):

```bash
source /opt/ros/humble/setup.bash
source ~/floor_cleaner_ws/install/setup.bash
ros2 launch floor_cleaner navigation.launch.py
```

This starts `map_server`, `amcl`, `controller_server`, `planner_server`,
`behavior_server`, `bt_navigator`, two lifecycle managers and RViz2.

The saved map appears in RViz. Until you set the initial pose (next section) the
terminal repeatedly prints something like
`Timed out waiting for transform from base_link to map ...` — **this is normal**: Nav2
waits because AMCL does not know yet where the robot is.

Check that the nodes are running:

```bash
ros2 node list
```

Expected (order may differ):

```text
/amcl
/behavior_server
/bt_navigator
/controller_server
/global_costmap/global_costmap
/lifecycle_manager_localization
/lifecycle_manager_navigation
/local_costmap/local_costmap
/map_server
/planner_server
/robot_state_publisher
/rviz2
...
```

To use a different map: `ros2 launch floor_cleaner navigation.launch.py map:=/full/path/to/map.yaml`

---

## 12. Setting the initial pose

In RViz2:

1. Look at Gazebo to see where the robot really is and which way it faces.
2. Click **2D Pose Estimate** in the top toolbar.
3. Click on the map at the robot's position **and hold** the mouse button.
4. Drag in the direction the robot is facing, then release.

Result: green AMCL particle arrows appear around the robot, the red LiDAR points line
up with the black walls of the map, the costmaps appear, and the **Navigation 2** panel
(left side) shows *Navigation: active* and *Localization: active*.

If the red points do not match the walls, do it again more carefully. Driving a
little (or letting Nav2 drive) makes AMCL converge further.

---

## 13. Sending a Nav2 Goal

1. Click **Nav2 Goal** in the top toolbar.
2. Click a free (white) spot on the map, e.g. inside Room 2, **and hold**.
3. Drag to choose the final orientation, release.
4. Nav2 computes a **blue global path** (`/plan`); the controller's **magenta local path**
   (`/local_plan`) appears in front of the robot.
5. The robot drives there in Gazebo and RViz, avoiding walls and furniture using the
   LiDAR, and turns to the requested orientation at the end.

Try an obstacle test: in Gazebo use the box tool (top toolbar) to drop a box in front
of the moving robot — it appears in the costmaps and the robot goes around it.

Alternatives:

* **2D Goal Pose** (also in the toolbar) publishes `/goal_pose`; `bt_navigator` listens to
  that topic too.
* Command line:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 2.5, z: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

**One-terminal shortcut:** after a map is saved you can restart everything with

```bash
ros2 launch floor_cleaner full_system.launch.py mode:=navigation
```

Here the robot is freshly spawned at (0,0), so AMCL is given that pose automatically and
you can click Nav2 Goal straight away. `full_system.launch.py` (default `mode:=slam`)
likewise starts Gazebo + SLAM + RViz for mapping.

---

## 14. Understanding the RViz display

| Display | Topic | Meaning |
|---------|-------|---------|
| Map | `/map` | White free, black occupied, grey unknown |
| Global Costmap | `/global_costmap/costmap` | Map + live obstacles, "inflated" (blue/cyan/purple halo) so paths keep a distance from walls |
| Local Costmap | `/local_costmap/costmap` | 3 m x 3 m window around the robot, live LiDAR obstacles only |
| RobotModel | `/robot_description` | The 3D robot |
| TF | `/tf`, `/tf_static` | Small axes for every frame (enable *Show Names* to see names) |
| LaserScan | `/scan` | Red points = what the LiDAR sees |
| Odometry | `/odom` | Red arrows = where wheel odometry thinks the robot went |
| Robot Footprint | `/local_costmap/published_footprint` | Green rectangle Nav2 uses for collision checking |
| AMCL Particles | `/particle_cloud` | Green arrows = AMCL's guesses of the robot pose. Tight cluster = well localised |
| Global Path | `/plan` | Blue line = full route from the planner |
| Local Path | `/local_plan` | Magenta = the short trajectory the controller is following right now |
| Cleaning Path | `/cleaning_path` | Yellow = zig-zag plan of the cleaning node |

Fixed Frame is `map`. During SLAM the costmap, path and particle displays show
"no messages received" — expected, they are Nav2 displays.

If you ever run only `simulation.launch.py` and open RViz, set Fixed Frame to `odom`
(there is no `map` frame without SLAM or AMCL).

---

## 15. Troubleshooting

**Gazebo does not open / hangs / "spawn_entity: entity already exists"** — an old Gazebo
is still running in the background:

```bash
killall -9 gzserver gzclient
```

**Gazebo is very slow** — start without the window (`gui:=false`) and use RViz only:

```bash
ros2 launch floor_cleaner simulation.launch.py gui:=false
```

In a virtual machine or WSL2, if Gazebo/RViz crash or show a black window:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

**`Package 'floor_cleaner' not found`** — you forgot `source ~/floor_cleaner_ws/install/setup.bash`
in that terminal (or the build failed).

**Robot does not move with `/cmd_vel`**

```bash
ros2 topic info /cmd_vel          # Subscription count must be >= 1 (the Gazebo plugin)
ros2 topic echo /odom --once      # does odometry exist?
```

If `/odom` does not exist the robot was not spawned: look for red errors in the
simulation terminal.

**No map in RViz during SLAM**

```bash
ros2 topic echo /scan --once --field header    # LiDAR working?
ros2 run tf2_ros tf2_echo map odom             # SLAM publishing TF?
```

**Nav2 never becomes active / "Timed out waiting for transform ... map"** — set the pose
with **2D Pose Estimate** (section 12). Check AMCL is running and that SLAM is stopped.

**Robot jumps around / map shifts** — SLAM and AMCL are running at the same time. Stop one:

```bash
ros2 node list | grep -E "slam_toolbox|amcl"
```

**"Failed to create plan" / goal aborted** — the goal is inside an obstacle, too close
to a wall, or in grey unknown space. Pick a spot in the middle of a white area.

**Robot keeps spinning or backing up** — these are the recovery behaviours; usually the
pose estimate is wrong. Set the 2D Pose Estimate again.

**"Message Filter dropping message: frame 'laser'"** — normal for a few seconds at
start-up. If it never stops, some node is not using simulation time:

```bash
ros2 param get /amcl use_sim_time
ros2 param get /slam_toolbox use_sim_time
```

Both must print `True`.

**Nav2 Goal button missing in RViz** — the Nav2 RViz plugins are missing:
`sudo apt install ros-humble-nav2-rviz-plugins`.

**navigation.launch.py says the map file does not exist** — your workspace is not at
`~/floor_cleaner_ws`, or the map has another name. Pass it explicitly with `map:=/full/path/my_map.yaml`.

---

## 16. Checking TF

Generate a PDF of the TF tree (run while the system is running):

```bash
cd ~
ros2 run tf2_tools view_frames
```

It listens for 5 seconds and writes `frames_<date>.pdf` in the current folder. Open it:

```bash
xdg-open ~/frames_*.pdf
```

Expected: `map → odom → base_link → {base_footprint, chassis, laser, left_wheel, right_wheel}`,
`chassis → {front_caster, rear_caster}`. (`map` only exists while SLAM or AMCL runs.)

Where is the robot on the map?

```bash
ros2 run tf2_ros tf2_echo map base_link
```

Is the LiDAR where it should be (0.19 m above base_link)?

```bash
ros2 run tf2_ros tf2_echo base_link laser
```

Check that there are no duplicate TF publishers:

```bash
ros2 topic info /tf --verbose
```

Publishers should be: the Gazebo node (`odom → base_link`), `robot_state_publisher`
(wheels), and **either** `slam_toolbox` **or** `amcl` (`map → odom`).

---

## 17. Checking the LiDAR

```bash
ros2 topic echo /scan --once
ros2 topic hz /scan                   # about 10 Hz
ros2 topic info /scan --verbose       # type sensor_msgs/msg/LaserScan
```

In the message, `header.frame_id` is `laser`, `range_min` 0.12, `range_max` 12.0,
and `ranges` holds 360 numbers.

---

## 18. Checking odometry

```bash
ros2 topic echo /odom --once
ros2 topic hz /odom                   # about 50 Hz
```

`header.frame_id` is `odom`, `child_frame_id` is `base_link`. Drive forward and watch
`pose.pose.position.x` increase:

```bash
ros2 topic echo /odom --field pose.pose.position
```

---

## 19. Moving to real hardware (Raspberry Pi)

Only the bottom layer changes. Everything from `/scan`, `/odom`, TF and `/cmd_vel`
upwards (SLAM Toolbox, AMCL, Nav2, RViz2, cleaning node, all yaml files) stays the same.

```text
        SIMULATION                               REAL ROBOT
┌───────────────────────────┐         ┌──────────────────────────────────┐
│ Gazebo world + physics    │   ──►   │ the real floor                   │
│ Gazebo LiDAR sensor       │   ──►   │ LiDAR driver node (e.g. rplidar) │  -> /scan, frame "laser"
│ diff drive plugin         │   ──►   │ base controller node (you write) │  /cmd_vel in,
│                           │         │   + motor board firmware         │  /odom + TF odom->base_link out
│ joint state plugin        │   ──►   │ same base controller node        │  -> /joint_states
│ Gazebo /clock             │   ──►   │ real time: use_sim_time:=false   │
│ robot_state_publisher     │   ==    │ robot_state_publisher (use_sim:=false) │
└───────────────────────────┘         └──────────────────────────────────┘
```

```text
Real LiDAR → /scan → SLAM / AMCL → Nav2 → /cmd_vel → base controller → motor board → motors
                                                            ↑
                                  wheel encoders ──► /odom + TF odom->base_link
```

What the **base controller node** must do (it replaces `gazebo_control.xacro`):

1. Subscribe to `/cmd_vel` (`geometry_msgs/Twist`), convert to wheel speeds:
   `v_left = v - w * 0.30 / 2`, `v_right = v + w * 0.30 / 2` (m/s),
   send them to the motor board.
2. Read encoder ticks, integrate x, y, yaw, publish `/odom` (`frame_id: odom`,
   `child_frame_id: base_link`) **and** the TF `odom → base_link`.
3. Publish `/joint_states` for `left_wheel_joint` / `right_wheel_joint`.

Real-robot launch = `robot_state_publisher` with `xacro floor_cleaner.urdf.xacro use_sim:=false`
+ LiDAR driver + base controller, then the **unchanged** `slam.launch.py` /
`navigation.launch.py` with `use_sim_time:=false`. Measure your real wheel radius,
wheel separation and LiDAR mounting position and put them in the xacro.

Example hardware split — Raspberry Pi as the ROS 2 computer, an ESP32 as the motor
board (L298N + encoder motors), a separate microcontroller for pump/fan/brushes:

* The ESP32 runs a PID loop per wheel so each wheel actually reaches the requested
  speed (an L298N alone is open-loop), counts encoder ticks, and **stops the motors if
  no speed command arrives for ~0.5 s** (safety if the Pi or link dies).
* The Pi ↔ ESP32 link carries `/cmd_vel` at ~20 Hz and odometry at 20–50 Hz. Nav2's
  controller and AMCL are sensitive to late or bursty odometry, so a **USB serial link
  (or micro-ROS over serial)** is much more dependable for this loop than Wi-Fi/MQTT.
  Wi-Fi/MQTT is fine for slow, non-critical commands such as "pump on", "fan speed".
* The cleaning subsystem (pump, fan, IR sensors) stays outside Nav2; the cleaning
  node can later switch it on/off while it runs the zig-zag.
* Heavy tools (RViz2, and even Nav2 if the Pi struggles) can run on a laptop on the same
  network with the same `ROS_DOMAIN_ID`.

---

## 20. Zig-zag cleaning

`scripts/cleaning_pattern_node.py` computes a boustrophedon pattern inside a rectangle
and sends each lane end to Nav2 as a `NavigateToPose` goal, one after another:

```text
→ → → → → → → →
              ↓
← ← ← ← ← ← ← ←
↓
→ → → → → → → →
```

The default rectangle (`config/cleaning_pattern.yaml`) covers the hall of the
simulated apartment: x −4.3…4.3, y −0.9…0.9, lanes 0.45 m apart.

With Nav2 running and the robot localised:

```bash
# only show the plan (yellow path in RViz), robot does not move
ros2 launch floor_cleaner cleaning.launch.py dry_run:=true

# run it
ros2 launch floor_cleaner cleaning.launch.py
```

or directly with custom values:

```bash
ros2 run floor_cleaner cleaning_pattern_node.py --ros-args \
  -p use_sim_time:=true -p x_min:=1.0 -p x_max:=4.0 -p y_min:=2.0 -p y_max:=3.5 -p lane_spacing:=0.4
```

Some lanes cross furniture (the sofa, the round table): Nav2 simply plans around it.
A waypoint that is unreachable is skipped and logged.

This is intentionally minimal. Next steps for a real cleaning planner:

* Split the map into rooms and run the zig-zag per room.
* Generate the rectangle(s) from the saved map instead of by hand.
* Use Nav2's `NavigateThroughPoses` or the waypoint follower for smoother lanes.
* Track covered cells to report coverage percentage.
* Switch the cleaning hardware (pump/fan) on only while driving a lane.

---

## 21. Test checklist

Run with simulation (+ SLAM or Nav2) running.

| # | Test | Command | Expected |
|---|------|---------|----------|
| 1 | Gazebo topics | `ros2 topic list` | `/clock /cmd_vel /joint_states /odom /scan /tf /tf_static` |
| 2 | LiDAR | `ros2 topic echo /scan --once` | a `sensor_msgs/msg/LaserScan`, frame `laser` |
| 3 | Odometry | `ros2 topic echo /odom --once` | frame `odom`, child `base_link` |
| 4 | cmd_vel | publish 0.2 m/s (section 7) | robot moves in Gazebo; zero command stops it |
| 5 | TF | `ros2 run tf2_tools view_frames` | tree as in section 16 |
| 6 | SLAM | `ros2 topic echo /map --once --field info` | map info (resolution 0.05) |
| 7 | Map saved | `ls ~/floor_cleaner_ws/src/floor_cleaner/maps` | `my_map.pgm my_map.yaml` |
| 8 | Nav2 nodes | `ros2 node list` | nodes listed in section 11 |
| 9 | Nav2 active | Navigation 2 panel in RViz | Navigation: active, Localization: active |
| 10 | Goal | Nav2 Goal in RViz | blue + magenta paths, robot drives there |
