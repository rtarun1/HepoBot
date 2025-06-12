# HepoBot
ROS2 Humble and Gazebo Harmonic Setup for HepoBot at RRC, IIIT (Summer School 2025)

## How to Use

- **Clone the repo with submodules**
    ```bash
    git clone https://github.com/rtarun1/HepoBot.git --recurse-submodules
    cd HepoBot
    # Open VSCode 
    code .
    ```
> [!NOTE]
> Change the ROS_DOMAIN_ID to a random number between 0 to 101 inside the devcontainer.json file.
- **Build and Source the workspace**

- **To enter the container**
    - Open Command Pallete with `Ctrl+Shift+P`
    - Select **Dev Containers: Rebuild and Reopen in Container** 

  It is suggested to use the [Action Buttons](#action-buttons) to build, but if you're unable due to some reason, you can use the commands below

  ```bash
  # Make sure to be in ~/ros2_ws
  colcon build --symlink-install 
  source install/setup.bash
  ```
- **To setup Permission for LiDAR and Motor Driver**
    ```
    sudo chmod +777 /dev/ttyACM0 #For Motor Driver
    sudo chmod +777 /dev/ttyUSB0 #For LiDAR
    ```
> [!NOTE] 
> If `/dev/ttyUSB0` is not visible, run `sudo apt remove brltty` locally.
- **Robot Launch**

  ```bash
  ros2 launch robot_bringup robot_bringup.launch.py use_rviz:=<bool> use_sim_time:=<bool>
  ```
  - **Mapping**
  ```bash
  ros2 launch robot_bringup online_async_launch.py use_sim_time:=<bool>
  ```
  - **Navigation** 
  ```bash
  ros2 launch robot_bringup nav2.launch.py use_sim_time:=<bool>
  ```
  - **Keyboard Telep**
  ```bash
  ros2 run robot_bringup keyboard_teleop_constant.py
  ```

  Available arguments are as follows:

  | Argument      | Default Value | Description                                    |
  |--------------|--------------|------------------------------------------------|
  | `use_sim_time` | `False`      | Launch in simulation mode (`True` for simulation, `False` for hardware connection). |           |
  | `record`      | `False`      | Enable recording to a rosbag.                  |
  | `use_rviz`    | `False`      | Launch RViz on startup.                        |

