# ROS2-move-robot-in-unknown-world
Software to help robot move in an unknown world of obstacles using sensors.


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

### Team Members
- Jonas Land
- Ayushri Jain
