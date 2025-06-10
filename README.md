# HepoBot
ROS2 Setup for HepoBot at RRC, IIIT-H

## Table of contents

- [HepoBot](#hepobot)
  - [Table of contents](#table-of-contents)
  - [Development Setup](#development-setup)
    - [Action Buttons](#action-buttons)
  - [Hardware Setup](#hardware-setup)
  - [How to Use](#how-to-use)


## Development Setup
- To pull latest docker image
    ```bash
    docker pull ghcr.io/soham2560/humble-harmonic:latest
    ```
- To start container
    - Open Command Pallete with `Ctrl+Shift+P`
    - Select Option to Rebuild and Reopen Container

  ### Action Buttons
  We use the [VSCode Action Button Extension](https://marketplace.visualstudio.com/items?itemName=seunlanlege.action-buttons) to facilitate development. They are not necessary but certainly do help. To access these buttons you may need to enable it through the Extensions Tab in VSCode, though the extension should download automatically on container startup. The available buttons are as follows:
  - `Build`

    Builds the workspace packages upto `robot_bringup`

    ```bash
    colcon build --symlink-install
    ```

## How to Use
- **Build and Source the workspace**

  It is suggested to use the [Action Buttons](#action-buttons) to build, but if you're unable due to some reason, you can use the commands below

  ```bash
  colcon build --symlink-install --packages-up-to robot_bringup
  source install/setup.bash
  ```
- **To setup LiDAR and Motor Driver**
    ```
    sudo chmod +777 /dev/ttyACM0 #For Motor Driver
    sudo chmod +777 /dev/ttyUSB0 #For LiDAR

> [!NOTE] 
> If `/dev/ttyUSB0` is not visible, run `sudo apt remove brltty` locally.
- **Launch**

  ```bash
  ros2 launch robot_bringup robot_bringup.launch.py use_rviz:=True use_sim_time:=False
  ```
  Available arguments are as follows:

  | Argument      | Default Value | Description                                    |
  |--------------|--------------|------------------------------------------------|
  | `use_sim_time` | `False`      | Launch in simulation mode (`True` for simulation, `False` for hardware connection). |           |
  | `record`      | `False`      | Enable recording to a rosbag.                  |
  | `use_rviz`    | `False`      | Launch RViz on startup.                        |



Note: The README's in this repository are inspired by [this](https://github.com/TheProjectsGuy/MR21-CS7.503)
