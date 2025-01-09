# ROS 2 Robot Messaging Project

This is a ROS 2 project designed for inter-node communication using custom messages. It consists of two nodes (`info_local` and `info_global`) and a custom message package (`robpos_msg`). The project demonstrates publishing robot information in a rotated global frame of reference.

## Project Structure

The project consists of the following packages:

### 1. `robpos_msg`
This package contains a custom message definition (`Robot.msg`) with the following fields:

```plaintext
string name
float64 temp
geometry_msgs/Pose2D center
```

### 2. `assignment1_muhammad`
This package contains two Python nodes:

- **`info_local`**: Publishes robot information periodically.
- **`info_global`**: Subscribes to the topic, transforms coordinates to a rotated global frame, and republishes the transformed data.

## Nodes Description

### `info_local`
- Publishes a custom message (`Robot`) on the topic `topic`.
- Message includes robot name, temperature, and pose information.

### `info_global`
- Subscribes to the `topic` and transforms the received coordinates into a global frame rotated by 50 degrees.
- Republishes the transformed data on a new topic called `output`.

## Setup Instructions

### Prerequisites
Ensure you have ROS 2 Jazzy installed and a working workspace.

### Build the Workspace
```bash
cd ~/ros2_ws
colcon build --packages-select robpos_msg assignment1_muhammad
source install/setup.bash
```

### Running the Nodes

Open three terminals and run the following commands:

**Terminal 1:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run assignment1_muhammad info_local
```

**Terminal 2:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run assignment1_muhammad info_global
```

**Terminal 3 (Optional - To View Data):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /output
```

## Custom Message Creation Steps

1. Create a message package:
```bash
ros2 pkg create --build-type ament_cmake robpos_msg
```
2. Add the message definition in `msg/Robot.msg`.
3. Modify `CMakeLists.txt` and `package.xml` to build the message.
4. Rebuild the workspace and source the setup file.

## Transform Logic
The global frame is rotated by 50 degrees relative to the local frame. The transformation applied in `info_global` node:

```python
import math
angle = math.radians(50)
msg.center.x = local_x * math.cos(angle) - local_y * math.sin(angle)
msg.center.y = local_x * math.sin(angle) + local_y * math.cos(angle)
```

## License
This project is licensed under the MIT License.

## Contributions
Contributions are welcome! Feel free to open an issue or submit a pull request.

## Author
Muhammad

---

### Happy Coding! 🚀

