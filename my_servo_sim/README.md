## 🤖 Servo Simulation for ROS 2 Humble

Brief description: `my_servo_sim` simulates a servo in Gazebo Classic and synchronizes its joint angle with real-time ESP32 serial input.

## 1) Objectives

- Run servo simulation using `ros2_control`.
- Keep the Gazebo joint synchronized with a physical ESP32 angle stream.
- Support one-command startup from a single launch file.

## 2) Recommended Workspace Structure

Use the standard ROS 2 colcon layout:

```text
servo_ws/
├── src/
│   └── my_servo_sim/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       ├── launch/
│       ├── scripts/
│       └── urdf/
├── build/
├── install/
└── log/
```

If `my_servo_sim` is currently in the workspace root, move/copy it into `src/`:

```bash
mkdir -p src
cp -r my_servo_sim src/
```

## 3) System Data Flow (Mermaid)

```mermaid
flowchart LR
    A[ESP32: Serial Output] -- "(-90 to 90)" --> B(serial_to_servo.py)
    B -- "Float64" --> C[/ "/servo_controller/commands" /]
    C --> D[ros2_control]
    D --> E[Gazebo: servo_joint]
    E --> F[/ "/joint_states" /]
```

## 4) Build

```bash
cd ~/servo_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 5) Run

Simulation only (without serial bridge):

```bash
ros2 launch my_servo_sim sim.launch.py start_serial:=false
```

Full system in one command (Gazebo + controllers + serial bridge):

```bash
ros2 launch my_servo_sim sim.launch.py \
	serial_port:=/dev/ttyACM0 \
	serial_baud:=115200 \
	serial_input_unit:=deg \
	serial_min_deg:=-90.0 \
	serial_max_deg:=90.0
```

## 6) Quick Verification

```bash
ros2 control list_controllers
ros2 topic info /servo_controller/commands
ros2 topic echo /joint_states --once
```

Expected result:

- `joint_state_broadcaster` is `active`
- `servo_controller` is `active`
- Topic `/servo_controller/commands` has both a publisher and a subscriber

## 7) Contributor

- GitHub: `biliz_nguyen`
- Name: `Nguyen Le Minh`
- Email: `leminhhu.edu@gmail.com`
