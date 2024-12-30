# ROS2-move-robot-in-unknown-world
Software to help robot move in an unknown world of obstacles using sensors.

# PART - 1
## How the Program Works

### Overview
Our project uses a **ROS2 launch file** to run multiple nodes, simulating a differential drive robot in a virtual world. The launch file takes two key arguments: 
- `robot_file`: Specifies the robot's configuration.
- `world_file`: Specifies the description of the world.

Additional arguments, `bag_in` and `bag_out`, manage ROS bag files for playback and recording.

#### Sample Command
```bash
ros2 launch project4b start.launch.py \
  world_file:=project4b/world_files/brick.world \
  robot_file:=project4b/robot_files/normal.robot \
  bag_in:=project4b/input_bags/input5 \
  bag_out:=project4b/output_bags/output5
```

---

### Key Components of `start.launch.py`

1. **Launch Arguments**
   - **`robot_file`:** Used to load robot parameters from a robot file via `load_disc_robot`.
   - **`world_file`:** Used to load the environment map.
   - **`bag_in` and `bag_out`:** Manage input and output ROS bag files using the `DeclareLaunchArgument` method.

2. **Loading Robot Parameters**
   - The function `load_disc_robot` retrieves the robot's description, dimensions, and error characteristics from the specified robot file.

3. **Robot State Publisher Node**
   - Publishes the robot's state to the ROS2 ecosystem using its URDF description.

4. **Simulator Node**
   - Simulates the robot's behavior and interacts with the environment:
     - Periodically broadcasts the robot’s pose based on its velocities (`/vl` and `/vr`).
     - Publishes the environment map (`/map`) loaded from the world file.
     - Simulates laser scans (`/scan`) based on robot position and orientation.
     - Stops the robot upon detecting collisions with obstacles.

5. **Velocity Translator Node**
   - Converts Twist commands into differential drive wheel velocities (`/vl` and `/vr`) based on the robot's parameters.

6. **ROS Bag Playback and Recording**
   - **`ros2 bag play`:** Plays input bag files specified by `bag_in`.
   - **`ros2 bag record`:** Records all topics to the file specified by `bag_out`.

---

### Node Details

#### **`velocity_translator_node.py`**
This node calculates wheel velocities (`vl`, `vr`) for the differential drive robot based on Twist messages (`/cmd_vel`).

- **Subscriptions:**
  - `/cmd_vel` (Twist messages): Provides angular and linear velocity inputs.
- **Publications:**
  - `/vl` and `/vr` (Float64 messages): Outputs wheel velocities.
- **Key Functions:**
  - Declares `distance` and `radius` parameters from the robot file.
  - Computes `vl` and `vr` from linear and angular velocities in the `cmd_vel_callback` function.

#### **`simulator_node.py`**
This node simulates robot movement, environment interactions, and laser scans.

- **Subscriptions:**
  - `/vl` and `/vr` (wheel velocities).
- **Key Functions:**
  1. **Pose Updates:** Calculates and applies pose changes based on differential drive equations.
  2. **Error Handling:** Introduces Gaussian noise to velocities.
  3. **Collision Detection:** Prevents robot movement into obstacles.
  4. **Laser Scan Simulation:**
     - Generates scan data based on map and laser parameters.
     - Accounts for scan failures based on probability settings.
  5. **Map Management:**
     - Reads and parses the world file into occupancy grid arrays.
     - Periodically publishes the map on `/map`.

- **Timers:**
  - A 0.1-second periodic callback updates the robot's pose, broadcasts transforms, and publishes laser scans.

---

### Results and Reflection

#### Did the Results Meet Expectations?
Yes, the results met our expectations. Implementing map publication and collision detection was straightforward, while laser scan simulation was more challenging. Despite the complexities, we successfully met all requirements, including accurate simulation of robot movement, environment interaction, and laser scans.

---

### Screenshots of Test Cases

#### Test Case 1
<img width="378" alt="image" src="https://github.com/user-attachments/assets/103843a8-9760-4cbc-9373-fa0522e51e71" />


#### Test Case 2
<img width="378" alt="image" src="https://github.com/user-attachments/assets/c0716a4f-55f1-4289-b49a-152afe9ed4dd" />


#### Test Case 3
<img width="378" alt="image" src="https://github.com/user-attachments/assets/6119f93c-c980-4c8c-80cd-25e922754573" />


#### Test Case 4
<img width="378" alt="image" src="https://github.com/user-attachments/assets/4cf4558a-2927-45cb-9371-3fb340b4ecea" />


#### Test Case 5
<img width="378" alt="image" src="https://github.com/user-attachments/assets/4e53c076-d131-407b-a403-446adfe6d4ef" />


#### Test Case 6
<img width="378" alt="image" src="https://github.com/user-attachments/assets/fe82108f-1ae4-42b4-9be3-6340973fe1f9" />


---

# PART 2
## How the Program Works
Our launch file is used to launch the nodes that allow our project to run. It takes in the command line arguments:
- `robot_file`: Determines the robot file to get the information about the current robot.
- `world_file`: Determines the world description file to use.

It runs the built-in ROS2 node `robot_state_publisher` as well as our custom nodes:
- `simulator_node`
- `velocity_translator_node`
- `navigation_controller_node`

### Sample Command to Run the Code:
```bash
ros2 launch project4c start.launch.py world_file:=src/project4c/world_files/cave.world robot_file:=src/project4c/robot_files/normal.robot
```

---

## 1. `start.launch.py`
This launches the nodes that together create our simulator. The robot's description is loaded from a specified robot file using the `load_disk_robot` module. If there is an output bag included in the launch argument, it records to the specified output bag.

### Key Components:
- **Launch Arguments**: 
  - `robot_file` is declared as a launch argument to use it within the simulator node for `load_disc_robot`. 
  - We use `sys.argv` to grab its value, allowing us to use it in the robot state publisher node without hardcoding.
  - `world_file` is another launch argument accessed via the `DeclareLaunchArgument` method.

- **`load_disc_robot`**: 
  - This function retrieves the robot's information from the specified robot file to pass to other nodes.

- **Robot State Publisher Node**: 
  - Publishes the robot's state to the ROS2 ecosystem using the robot's URDF description. This is a built-in, unmodified node.

- **Simulator Node**:
  - Simulates the robot's behavior based on parameters like radius, height, wheel distance, and laser specifications from the robot file.
  - Subscribes to topics `/vl` and `/vr` (left and right wheel velocities) to broadcast the robot's location and simulate laser scans on `/scan`.
  - Stops the robot if it detects collisions with obstacles represented on the map.

- **Velocity Translator Node**:
  - Translates `Twist` commands into left and right wheel velocities of a differential drive robot based on its description.

- **Navigation Controller Node**:
  - Subscribes to laser scans and publishes `Twist` messages to move the robot reactively based on detected obstacles.

- **Record Bag**:
  - Records a ROS bag file using the `ros2 bag record` command, storing all topics in the output file.

---

## 2. `velocity_translator_node.py`
This node subscribes to `Twist` messages on the `/cmd_vel` topic and calculates left and right wheel velocities, publishing them as `Float64` messages to `/vl` and `/vr`.

### Key Components:
- Declares parameters:
  - `distance`: Distance between the robot's wheels.
  - `radius`: Radius of the wheels.
- Subscribes to `/cmd_vel` to process `Twist` messages in `cmd_vel_callback`.
- In the callback:
  - Retrieves linear and angular velocity from the `Twist` message.
  - Calculates `vr` and `vl`.
  - Publishes these velocities to `/vl` and `/vr`.

---

## 3. `simulator_node.py`
Simulates the robot's movement using tf transforms based on `/vl` and `/vr`.

### Key Components:
- Subscribes to `/vl` and `/vr` with a 0.1-second timer triggering the `timer_callback` function.
- Declares various robot-related parameters passed in from the launch file.
- **Timer Callback**:
  - Updates velocities (`self.vl`, `self.vr`) when `/vl` or `/vr` messages are received.
  - Sets velocities to 0 if updates are not received within 1 second.
  - Calls helper functions:
    - `update_errors`: Updates error multipliers using a random Gaussian function.
    - `update_robot_pose`: Calculates pose changes and ensures no collisions before broadcasting transforms.
    - `broadcast_transform`: Updates the robot’s position using a tf2 broadcaster.
- **Map-Related Functions**:
  - `get_map_data`: Reads the world file to set the initial pose and occupancy grid.
  - `publish_map`: Publishes the map as an occupancy grid every 2 seconds.
  - `is_collision`: Checks if new coordinates intersect obstacles.
- **Laser Scan Simulation**:
  - `generate_scan_data`: Simulates laser scan data based on robot parameters and map.
  - `publish_laser_scan`: Publishes the simulated scan data.

---

## 4. `navigation_controller_node.py`
Subscribes to `/scan` and publishes `Twist` messages on `/cmd_vel` to reactively navigate the robot based on laser scans.

### Key Components:
- **Subscribes** to `/scan` for `LaserScan` messages.
- Declares parameters like `distance`, `radius`, and a safe distance from obstacles.
- **`laser_callback`**:
  - Loops through scans and adjusts movement vectors based on proximity to obstacles.
  - Publishes a `Twist` message dictating robot movement.

---

## Did the Results Meet Expectations?
Yes, the results met our expectations! We successfully implemented a reactive controller to navigate the robot in 3D space using only laser scans. The process was more straightforward than expected, and we are proud of the outcomes.

---

## Screenshots for Test Cases
### Test Case 1
<img width="378" alt="image" src="https://github.com/user-attachments/assets/196c5c4e-1d35-4c9a-a6f2-c27881f69a27" />


### Test Case 2
<img width="378" alt="image" src="https://github.com/user-attachments/assets/8c3fc38c-55bc-4f11-8ccc-20e70e47164a" />


### Test Case 3
- **Case 1**: Robot directly opposite to its starting position.
<img width="378" alt="image" src="https://github.com/user-attachments/assets/3c35d80b-5008-4c31-b76b-8ca4e153dc10" />

- **Case 2**: Robot completing a round and returning near its starting point.
<img width="378" alt="image" src="https://github.com/user-attachments/assets/f247ad52-2050-4942-b999-3941807c1b6d" />


### Test Case 4
<img width="378" alt="image" src="https://github.com/user-attachments/assets/245c4a13-2820-4a5d-8027-518c65dba175" />


---


### Team Members
- Jonas Land
- Ayushri Jain
