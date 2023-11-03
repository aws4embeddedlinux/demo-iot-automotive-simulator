#!/bin/bash


log_dir="./logs"
mkdir -p $log_dir

# Function to log messages with timestamp
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" | tee -a "$log_dir/main.log"
}

# Function to start a process
start_process() {
    local command=$1
    local log_file=$2
    # log "Runnig $command - Logging to $log_file"
    setsid $command > $log_file 2>&1 &
    echo $! # Return the PID
}

# Function to kill all process groups
cleanup() {
    echo "Cleaning up..."
    for pid in "${pids[@]}"; do
        kill -- -$pid  # Kills the entire process group
    done
}

# Trap script termination signals to cleanup
trap cleanup EXIT SIGINT SIGTERM

# Main execution
main() {
    # Array to store process group IDs
    declare -a pids

    # Start processes
    pids+=($(start_process "/opt/carla-simulator/CarlaUE4.sh -no-rendering -quality-level=Low -prefernvidia" "$log_dir/carla.log"))
    log "pids: $pids"
    sleep 15

    source /opt/ros/galactic/setup.bash
    source ~/ros2_ws/install/setup.bash

    cd carla-client
    pids+=($(start_process "python3 ./manual_control_steeringwheel.py --sync --rolename ego_vehicle --filter vehicle.tesla.model3 -i can0" "../$log_dir/control.log"))
    pids+=($(start_process "python3 /opt/carla-simulator/PythonAPI/examples/generate_traffic.py -n 15 -w 20" "../$log_dir/traffic_gen.log"))
    cd ..
    log "pids: $pids"

    pids+=($(start_process "ros2 launch carla_ros_bridge carla_ros_bridge.launch.py timeout:=20000 register_all_sensors:=false synchronous_mode:=false passive:=true" "$log_dir/carla_ros_bridge.log"))
    pids+=($(start_process "ros2 launch carla_spawn_objects carla_spawn_objects.launch.py spawn_sensors_only:=True objects_definition_file:=./carla-client/ros2/objects.json" "$log_dir/carla_spawn_objects.log"))

    log "pids: $pids"

    cd carla-client/ros2
    pids+=($(start_process "python3 ./image_converter.py --input_topic /carla/ego_vehicle/rgb_front/image --output_topic /carla/ego_vehicle/rgb_front/image_compressed" "../../$log_dir/image_conv1.log"))
    pids+=($(start_process "python3 ./image_converter.py --input_topic /carla/ego_vehicle/depth_front/image --output_topic /carla/ego_vehicle/depth_front/image_compressed" "../../$log_dir/image_conv2.log"))
    cd ../../
    log "pids: $pids"

    cd breeze
    pids+=($(start_process "python3 breeze.py" "../$log_dir/breeze.log"))
    cd ..
    log "pids: $pids"
    
    pids+=($(start_process "python3 ./observability/can-stats/generate_stats.py" "./$log_dir/generate_stats.log"))

    log "pids: $pids"

    # Monitor all process groups
    while : ; do
        for pid in "${pids[@]}"; do
            if ! ps -p $pid > /dev/null 2>&1; then
                echo "Process group $pid has terminated. Exiting."
                cleanup
                exit 1
            fi
        done
        sleep 5
    done
}

# Run the main function
main
