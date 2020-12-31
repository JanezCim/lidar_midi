# lidar_midi package

lidar scan is polarly devided into x number of spans (x defined with number of midi ranges wanted). It then remaps the minimum detected lidar range into a value between min_val and max_val defined for each midi range and outputs only the changes onto a ros topic of type std_msgs/Int16MultiArray. Midi settings for each key can be defined in midi_settings.yaml.


## Installation
Only tested on ROS Melodic

Go to ros workspace root and run

    cd <path_to_workspace>
    rosdep install --from-paths src --ignore-src -r -y

## Running
### a) Running the node with Lidar in gazebo (no hardware needed)
Run in separate terminals from ros root workspace

    roslaunch lidar_midi lidar_gazebo.launch
    roslaunch lidar_midi lidar_midi.launch
    rviz -d src/lidar_midi/rviz/lidar_midi.rviz # optional but usefull visualisation

### b) Runing the node with an external scan source (usually with physical lidar)
Make sure you have the lidar setup so it publishes onto /scan topic. Then run in separate terminals

    roslaunch lidar_midi lidar_midi.launch
    rviz -d src/lidar_midi/rviz/lidar_midi.rviz # optional but usefull visualisation


### You should see the output if you type in another terminal
    
    rostopic echo /phosporm_midi_input