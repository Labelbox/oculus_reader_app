# Oculus Reader: comprehensive documentation

## overview

The Oculus Reader is a teleoperation system that captures data from Meta Quest (Oculus Quest 2) VR headsets to control robotic systems. It consists of two main components:

1. **Android APK application** running on the Quest device
2. **Python reader library** running on the computer/laptop

The system captures 6DOF controller poses, button states, and trigger values to enable intuitive robot teleoperation through hand movements and gestures.

## system architecture

```
┌─────────────────┐    ADB/TCP     ┌──────────────────┐    Python API    ┌─────────────────┐
│   Quest Device  │ ◄──────────► │   Laptop/Desktop │ ◄─────────────► │  Robot Control  │
│                 │               │                  │                  │                 │
│ • APK App       │               │ • oculus_reader  │                  │ • VRPolicy      │
│ • Controller    │               │ • ADB Bridge     │                  │ • RobotEnv      │
│   Tracking      │               │ • Data Parser    │                  │ • Actions       │
│ • Button Input  │               │                  │                  │                 │
└─────────────────┘               └──────────────────┘                  └─────────────────┘
```

## installation and setup

### prerequisites

1. **Meta Quest 2 or Quest Pro** with developer mode enabled
2. **Computer** with Ubuntu 20.04+ (recommended) or compatible Linux distribution
3. **Android Debug Bridge (ADB)** installed
4. **USB-C cable** for initial setup
5. **Git LFS** for downloading the APK file

### step 1: enable developer mode on quest

1. **Create Oculus Developer Account**:

   - Go to https://developer.oculus.com/
   - Create or log into your account
   - Create a new organization if needed

2. **Enable Developer Mode**:

   - Open Oculus app on your phone
   - Go to Settings → [Your Device] → More Settings → Developer Mode
   - Toggle Developer Mode ON
   - Accept the terms

3. **Enable USB Debugging**:
   - Connect Quest to computer via USB-C
   - Put on the headset
   - Accept "Allow USB Debugging" prompt
   - Check "Always allow from this computer"

### step 2: install dependencies

```bash
# Install Git LFS (required for APK download)
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install

# Install ADB and Android tools
sudo apt install android-tools-adb android-sdk-platform-tools-common

# Start ADB server
adb start-server
```

### step 3: clone and install oculus_reader

```bash
# Clone the repository (as submodule in DROID)
git submodule update --init --recursive

# Install oculus_reader Python package
pip install -e ./droid/oculus_reader
```

### step 4: install APK on quest device

```bash
# Verify Quest is connected
adb devices
# Should show: List of devices attached
#              [device_id] device

# Install the APK
python3 droid/oculus_reader/oculus_reader/reader.py
```

The installation script will:

1. Push the APK to the Quest device
2. Install the application
3. Set up necessary permissions
4. Test the connection

## data capture and formats

### controller pose data

The Quest controllers provide 6DOF tracking data as 4×4 transformation matrices:

```python
{
    "r": np.array([
        [r11, r12, r13, tx],    # Rotation matrix + translation
        [r21, r22, r23, ty],    # Right controller pose
        [r31, r32, r33, tz],    # in world coordinates
        [0,   0,   0,   1 ]     # Homogeneous coordinates
    ]),
    "l": np.array([...])        # Left controller pose (similar format)
}
```

**coordinate system**:

- **X-axis**: Right (positive) / Left (negative)
- **Y-axis**: Up (positive) / Down (negative)
- **Z-axis**: Forward (positive) / Backward (negative)
- **Units**: Meters
- **Reference**: Quest tracking space origin

### button state data

Complete button mapping for both controllers:

```python
{
    # Face buttons (boolean values)
    "A": False,          # Right controller A button
    "B": False,          # Right controller B button
    "X": False,          # Left controller X button
    "Y": False,          # Left controller Y button

    # Grip buttons (boolean values)
    "RG": False,         # Right grip button (side button)
    "LG": False,         # Left grip button (side button)

    # Joystick buttons (boolean values)
    "RJ": False,         # Right joystick click
    "LJ": False,         # Left joystick click

    # Trigger values (analog, list format)
    "rightTrig": [0.0],  # Right trigger: 0.0 (released) to 1.0 (fully pressed)
    "leftTrig": [0.0]    # Left trigger: 0.0 (released) to 1.0 (fully pressed)
}
```

### gripper control mechanism

The gripper is controlled via **analog triggers** on the VR controllers. **Only one trigger is active at a time**, determined by the controller selection:

#### trigger selection

```python
# Right controller mode (default)
vr_controller = VRPolicy(right_controller=True)
# Uses RIGHT trigger for gripper control
# Left trigger is ignored

# Left controller mode
vr_controller = VRPolicy(right_controller=False)
# Uses LEFT trigger for gripper control
# Right trigger is ignored
```

#### gripper control mapping

- **Trigger Value 0.0**: Gripper fully **open**
- **Trigger Value 1.0**: Gripper fully **closed**
- **Analog Control**: Partial trigger press = partial gripper closure
- **Rate Control**: Speed of trigger movement controls gripper velocity

#### code implementation

```python
# From VRPolicy._process_reading() - trigger selection logic
vr_gripper = self._state["buttons"]["rightTrig" if self.controller_id == "r" else "leftTrig"][0]

# Gripper action calculation
gripper_action = (self.vr_state["gripper"] * 1.5) - robot_gripper
gripper_action *= self.gripper_action_gain  # Default: 3.0
```

### controller-specific button functions

#### right controller usage

- **A Button**: Mark trajectory as **success** and stop recording
- **B Button**: Mark trajectory as **failure** and stop recording
- **Right Grip (RG)**: Hold to enable robot movement during recording
- **Right Joystick (RJ)**: Reset controller orientation
- **Right Trigger**: **Primary gripper control** (0.0-1.0) - used when `right_controller=True`

#### left controller usage

- **X Button**: Mark trajectory as **success** and stop recording
- **Y Button**: Mark trajectory as **failure** and stop recording
- **Left Grip (LG)**: Hold to enable robot movement during recording
- **Left Joystick (LJ)**: Reset controller orientation
- **Left Trigger**: **Primary gripper control** (0.0-1.0) - used when `right_controller=False`

**Important**: Only one trigger is used for gripper control at a time, determined by the `right_controller` parameter in `VRPolicy`.

### data streaming frequency

- **Default polling rate**: 50 Hz (20ms intervals)
- **Configurable**: Can be adjusted via `hz` parameter
- **Connection timeout**: 5 seconds (configurable via `num_wait_sec`)

## communication protocol

### connection methods

#### USB connection (recommended for setup)

```python
from oculus_reader.reader import OculusReader

# Connect via USB (default)
reader = OculusReader()
poses, buttons = reader.get_transformations_and_buttons()
```

#### network/wifi connection (for untethered operation)

```python
# Connect via network IP
reader = OculusReader(ip_address="192.168.1.100")
poses, buttons = reader.get_transformations_and_buttons()
```

### ADB bridge communication

The system uses Android Debug Bridge (ADB) for communication:

1. **ADB Server**: Runs on host computer (`adb start-server`)
2. **TCP Socket**: Established between computer and Quest
3. **Port Forwarding**: ADB handles USB-to-TCP translation
4. **Data Packets**: JSON-encoded pose and button data

### connection establishment sequence

```bash
# 1. Start ADB server
adb start-server

# 2. Verify device connection
adb devices

# 3. Check Quest IP (for network connection)
adb shell ip route

# 4. Launch Quest application
adb shell am start -n com.rail.oculus.teleop/.MainActivity

# 5. Python reader connects and starts polling
```

## usage examples

### basic data reading

```python
import time
import numpy as np
from oculus_reader.reader import OculusReader

# Initialize reader
reader = OculusReader()

# Continuous reading loop
while True:
    # Get current state
    poses, buttons = reader.get_transformations_and_buttons()

    if poses:  # Check if data is available
        # Access right controller pose
        right_pose = poses["r"]  # 4x4 transformation matrix
        right_position = right_pose[:3, 3]  # Extract position
        right_rotation = right_pose[:3, :3]  # Extract rotation matrix

        # Access button states
        grip_pressed = buttons["RG"]  # Right grip button
        trigger_value = buttons["rightTrig"][0]  # Right trigger (0.0-1.0)

        print(f"Position: {right_position}")
        print(f"Grip: {grip_pressed}, Trigger: {trigger_value:.2f}")

    time.sleep(0.02)  # 50 Hz polling
```

### accessing gripper trigger based on controller selection

```python
# Example showing trigger selection logic
def get_gripper_value(buttons, use_right_controller=True):
    """Get gripper trigger value based on controller selection"""
    if use_right_controller:
        return buttons["rightTrig"][0]  # Use right trigger
    else:
        return buttons["leftTrig"][0]   # Use left trigger

# Usage example
poses, buttons = reader.get_transformations_and_buttons()
if buttons:
    # For right controller mode (default)
    right_gripper = get_gripper_value(buttons, use_right_controller=True)

    # For left controller mode
    left_gripper = get_gripper_value(buttons, use_right_controller=False)

    print(f"Right controller gripper: {right_gripper:.2f}")
    print(f"Left controller gripper: {left_gripper:.2f}")
```

### robot teleoperation integration

```python
from droid.controllers.oculus_controller import VRPolicy
from droid.robot_env import RobotEnv

# Initialize robot environment and VR controller
env = RobotEnv()
vr_controller = VRPolicy(
    right_controller=True,           # Use right controller
    pos_action_gain=5.0,            # Position sensitivity
    rot_action_gain=2.0,            # Rotation sensitivity
    gripper_action_gain=3.0,        # Gripper sensitivity
    max_lin_vel=1.0,                # Max linear velocity
    max_rot_vel=1.0,                # Max rotational velocity
    spatial_coeff=1.0               # Spatial scaling factor
)
# Note: With right_controller=True, RIGHT trigger controls gripper
# With right_controller=False, LEFT trigger controls gripper

# Teleoperation loop
while True:
    # Get current robot state
    obs = env.get_observation()

    # Generate robot action from VR input
    action = vr_controller.forward(obs)

    # Execute action on robot
    env.step(action)

    # Check for stop conditions
    info = vr_controller.get_info()
    if info["success"] or info["failure"]:
        break
```

### data recording with MCAP format

```python
from droid.trajectory_utils.trajectory_writer_mcap import TrajectoryWriterMCAP
from droid.controllers.oculus_controller import VRPolicy

# Initialize recording
writer = TrajectoryWriterMCAP("trajectory.mcap", save_images=True)
vr_controller = VRPolicy()

# Recording loop
for timestep in range(1000):
    # Collect VR controller data
    controller_info = vr_controller.get_info()

    # Create timestep data
    timestep_data = {
        "observation": {
            "controller_info": controller_info,
            # ... other observation data
        },
        "action": action,
        "timestamp": time.time()
    }

    # Write to MCAP file
    writer.write_timestep(timestep_data)

writer.close()
```

## coordinate transformations

### VR to robot coordinate mapping

The system performs several coordinate transformations:

1. **Quest tracking space → Global space**: Via `vr_to_global_mat`
2. **Global space → Robot space**: Via `global_to_env_mat`
3. **Relative motion tracking**: Origin calibration on grip press

```python
# Example transformation pipeline (from VRPolicy._process_reading)
def transform_vr_pose(quest_pose, vr_to_global_mat, global_to_env_mat, spatial_coeff):
    # Apply coordinate transformations
    transformed_pose = global_to_env_mat @ vr_to_global_mat @ quest_pose

    # Extract position and rotation
    position = spatial_coeff * transformed_pose[:3, 3]
    rotation_matrix = transformed_pose[:3, :3]
    quaternion = rotation_matrix_to_quaternion(rotation_matrix)

    return position, quaternion
```

### reorder matrix configuration

Default coordinate reordering (configurable):

```python
rmat_reorder = [-2, -1, -3, 4]  # Maps VR axes to robot axes
# -2: X maps to -Y (right/left → left/right)
# -1: Y maps to -X (up/down → down/up)
# -3: Z maps to -Z (forward/back → back/forward)
#  4: Homogeneous coordinate
```

## calibration and origin handling

### automatic origin calibration

When the grip button is pressed, the system automatically:

1. **Records robot origin**: Current robot position and orientation
2. **Records VR origin**: Current VR controller position and orientation
3. **Calculates relative motion**: All subsequent movements are relative to these origins

```python
# Origin calibration (from VRPolicy._calculate_action)
if self.reset_origin:
    # Store current poses as reference
    self.robot_origin = {"pos": robot_pos, "quat": robot_quat}
    self.vr_origin = {"pos": vr_state["pos"], "quat": vr_state["quat"]}
    self.reset_origin = False

# Calculate relative movements
robot_pos_offset = robot_pos - self.robot_origin["pos"]
target_pos_offset = vr_state["pos"] - vr_origin["pos"]
pos_action = target_pos_offset - robot_pos_offset
```

### orientation reset

Pressing the joystick button resets the "forward" direction:

```python
# Orientation reset (from VRPolicy._update_internal_state)
if self.reset_orientation:
    current_pose = self._state["poses"][self.controller_id]
    try:
        # Invert current rotation to define new "forward"
        self.vr_to_global_mat = np.linalg.inv(current_pose)
    except:
        # Fallback to identity if inversion fails
        self.vr_to_global_mat = np.eye(4)
```

## data schemas and storage

### MCAP trajectory format

VR controller data is stored using this JSON schema:

```json
{
  "type": "object",
  "properties": {
    "timestamp": {
      "type": "object",
      "properties": {
        "sec": { "type": "integer" },
        "nsec": { "type": "integer" }
      }
    },
    "poses": {
      "type": "object",
      "additionalProperties": {
        "type": "array",
        "items": { "type": "number" }
      }
    },
    "buttons": {
      "type": "object",
      "properties": {
        "A": { "type": "boolean" },
        "B": { "type": "boolean" },
        "X": { "type": "boolean" },
        "Y": { "type": "boolean" },
        "RG": { "type": "boolean" },
        "LG": { "type": "boolean" },
        "RJ": { "type": "boolean" },
        "LJ": { "type": "boolean" },
        "rightTrig": { "type": "array", "items": { "type": "number" } },
        "leftTrig": { "type": "array", "items": { "type": "number" } }
      }
    },
    "movement_enabled": { "type": "boolean" },
    "controller_on": { "type": "boolean" },
    "success": { "type": "boolean" },
    "failure": { "type": "boolean" }
  }
}
```

### data directory structure

```
~/recordings/
├── success/YYYY-MM-DD/           # Successful demonstrations
│   ├── trajectory_001.mcap       # MCAP trajectory data
│   ├── trajectory_002.mcap
│   └── recordings/SVO/           # Camera recordings
│       ├── [serial]_left.svo
│       ├── [serial]_right.svo
│       └── [serial]_depth.svo
├── failure/YYYY-MM-DD/           # Failed attempts
└── evaluation_logs/              # Policy evaluation logs
```

## performance tuning

### latency optimization

1. **USB vs WiFi**: USB provides ~5ms lower latency than WiFi
2. **Polling frequency**: 50Hz balances responsiveness and CPU usage
3. **Data filtering**: Optional smoothing filters for noisy tracking

```python
# High-performance configuration
reader = OculusReader()
vr_policy = VRPolicy(
    max_lin_vel=2.0,        # Higher speed limits
    max_rot_vel=2.0,
    pos_action_gain=7.0,    # More responsive control
    rot_action_gain=3.0
)
```

### threading and concurrency

The system uses threaded data collection:

```python
# Threaded state updates (from VRPolicy.__init__)
run_threaded_command(self._update_internal_state)

def _update_internal_state(self, num_wait_sec=5, hz=50):
    while True:
        time.sleep(1 / hz)  # Regulate frequency

        # Non-blocking data read
        poses, buttons = self.oculus_reader.get_transformations_and_buttons()

        # Update internal state
        self._state.update({"poses": poses, "buttons": buttons})
```

## troubleshooting

### common connection issues

#### device not found

```bash
# Check ADB connection
adb devices

# Restart ADB if needed
adb kill-server
adb start-server

# Verify USB debugging is enabled on Quest
```

#### network connection problems

```bash
# Get Quest IP address
adb shell ip route

# Test network connectivity
ping [quest_ip]

# Check firewall settings
sudo ufw allow 5555  # Default ADB port
```

#### permission errors

```bash
# Fix USB permissions
sudo usermod -a -G plugdev $USER
sudo udevadm control --reload-rules

# Restart after permission changes
```

### data quality issues

#### tracking drift

- **Cause**: Quest tracking loses reference points
- **Solution**: Ensure adequate lighting and visual features in tracking space

#### button responsiveness

- **Cause**: APK not running or ADB connection unstable
- **Solution**: Restart APK via `adb shell am force-stop com.rail.oculus.teleop`

#### pose jitter

- **Cause**: Quest controller battery low or interference
- **Solution**: Charge controllers, remove interference sources

### debugging utilities

```bash
# Check APK status
adb shell dumpsys package com.rail.oculus.teleop

# Monitor ADB logcat
adb logcat | grep oculus

# Test connection programmatically
python3 -c "
from oculus_reader.reader import OculusReader
reader = OculusReader()
poses, buttons = reader.get_transformations_and_buttons()
print('Connection successful:', len(poses) > 0)
"
```

## advanced usage

### custom APK modifications

The APK source code is available in `app_source/` for customization:

1. **Android Studio setup**: Import project from `app_source/`
2. **Modify data capture**: Edit tracking frequency, add sensors
3. **Custom UI**: Add visual feedback or controls
4. **Build and deploy**: Use Android Studio or Gradle

### multiple quest devices

```python
# Connect to multiple Quest devices
readers = []
for ip in ["192.168.1.100", "192.168.1.101"]:
    readers.append(OculusReader(ip_address=ip))

# Synchronize data collection
for reader in readers:
    poses, buttons = reader.get_transformations_and_buttons()
```

### integration with other systems

#### ROS integration

```python
import rospy
from geometry_msgs.msg import PoseStamped

# Publish VR poses to ROS
pose_pub = rospy.Publisher('/vr_pose', PoseStamped, queue_size=1)

while True:
    poses, buttons = reader.get_transformations_and_buttons()

    # Convert to ROS message
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    # ... populate pose data

    pose_pub.publish(pose_msg)
```

#### isaac sim integration

```python
# Omniverse Isaac Sim VR control
from omni.isaac.core import World
from oculus_reader.reader import OculusReader

world = World()
reader = OculusReader()

while True:
    poses, buttons = reader.get_transformations_and_buttons()

    # Apply VR input to simulated robot
    if poses:
        world.step()  # Update simulation
```

## configuration reference

### VRPolicy parameters

```python
VRPolicy(
    right_controller=True,          # Use right (True) or left (False) controller
    max_lin_vel=1.0,               # Maximum linear velocity (m/s)
    max_rot_vel=1.0,               # Maximum rotational velocity (rad/s)
    max_gripper_vel=1.0,           # Maximum gripper velocity
    spatial_coeff=1.0,             # Spatial scaling coefficient
    pos_action_gain=5.0,           # Position control gain
    rot_action_gain=2.0,           # Rotation control gain
    gripper_action_gain=3.0,       # Gripper control gain
    rmat_reorder=[-2, -1, -3, 4]   # Coordinate reordering matrix
)
```

### version-specific configurations

#### DROID version 1.0

```json
{
  "oculus_params": {
    "saving_target_info": false,
    "pos_action_gain": 3,
    "rot_action_gain": 3
  }
}
```

#### DROID version 1.1

```json
{
  "oculus_params": {
    "saving_target_info": true,
    "pos_action_gain": 5,
    "rot_action_gain": 2
  }
}
```

## security considerations

### ADB security

- **Developer mode**: Only enable when needed
- **USB debugging**: Disable after setup for security
- **Network exposure**: Use firewall rules to limit ADB access

### data privacy

- **Local processing**: All data stays on local network
- **No cloud transmission**: No data sent to Meta/Facebook servers
- **User control**: Complete control over recorded data

## performance benchmarks

### typical latency measurements

- **USB connection**: ~15-20ms end-to-end latency
- **WiFi connection**: ~25-30ms end-to-end latency
- **Processing overhead**: ~2-3ms for pose transformation

### throughput specifications

- **Data rate**: ~50-100 KB/s for pose + button data
- **CPU usage**: ~5-10% on modern laptops
- **Memory usage**: ~50-100 MB for Python process

## conclusion

The Oculus Reader provides a robust foundation for VR-based robot teleoperation with:

- **Low latency** real-time data streaming
- **High precision** 6DOF tracking
- **Intuitive controls** via natural hand movements
- **Flexible integration** with robot control systems
- **Comprehensive data logging** for machine learning

This documentation covers all aspects of installation, usage, and integration. For additional support, refer to the GitHub repository issues or contribute improvements to the codebase.
