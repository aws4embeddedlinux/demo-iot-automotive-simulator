# ROS2-CARLA Bridge Setup Guide

Please follow the instructions from the project README.md before preforming these.

## Installation Steps

1. **Start the Carla Server**

   Launch the Carla server with the following command:

   ```bash
   /opt/carla-simulator/CarlaUE4.sh -quality-level=Low
   ```

2. **Configure Environment Variables**

   Set up the necessary environment variables for Python and Carla:

   ```bash
   export CARLA_ROOT=/opt/carla-simulator/
   export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
   source /opt/ros/galactic/setup.bash
   ```

3. **Install Additional ROS2 Packages**

   Clone and build additional ROS2 packages required for the bridge:

   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
   git clone https://github.com/astuff/astuff_sensor_msgs.git
   cd ~/ros2_ws
   colcon build --symlink-install
   source ~/ros2_ws/install/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   ```


4. **Build and Install carla-ros-bridge**

   Ensure that you have built and installed the `carla-ros-bridge` and all dependencies:

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   sudo apt install python3-colcon-common-extensions python3-pip python3-rosdep2
   colcon build --symlink-install
   ```

5. **Run the ROS2 Bridge**

   Launch the ROS2 bridge with the Carla simulator:

   ```bash
   source /opt/ros/galactic/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch carla_ros_bridge carla_ros_bridge.launch.py timeout:=20000 register_all_sensors:=false synchronous_mode:=false passive:=true
   ```

6. **Spawn Sensors Using Objects Definition File**

   Use the provided objects definition file to spawn all sensors:

   ```bash
   source /opt/ros/galactic/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 launch carla_spawn_objects carla_spawn_objects.launch.py spawn_sensors_only:=True objects_definition_file:=carla_spawn_objects/config/objects.json
   ```

7. **Run the Client**

   To control the vehicle manually in the Carla simulator, run the client using:

   ```bash
   cd ~/carla-client
   python3 manual_control_steeringwheel.py --sync --rolename ego_vehicle --filter vehicle.tesla.model3
   ```

## Troubleshooting

If you encounter any issues, please ensure all environment variables are correctly set and that all dependencies are properly installed.
