### Description
This is a ROS Workspace to publish and subscribe to gps messages sources from csv file.  Please find below the installation guide and notes for the project.

### Libraries and Architecture
* ROS2 Humble
* Python
* Colcon
* Ubuntu 22.04 (VM)

### Installation Guide
#### ROS Humble installation guide
https://docs.ros.org/en/humble/Installation.html

#### Setup 
* Clone the Fanthom-R repository
* Source ROS2 using \
``source /opt/ros/humble/setup.bash``
* Navigate to <workspace_name>/ directory \
`cd <workspace_name>`
* Build using colcon and source the setup config
`colcon build` \
`source install/setup.bash`

#### Execution
* Run the subscriber node first to ensure that no messages are lost \
`ros2 run csv_processor subscriber`
* Source the config \
`source install/setup.bash`
* Run the publisher node in a new terminal \
`ros2 run csv_processor publisher`
* Source the config \
`source install/setup.bash`

#### Tracking
* Open a new terminal and source the config \
`source install/setup.bash`
* Check the list of topics and verify that `\data` and `\diff` are created \
`ros2 topic list`
* View the data being published in these topics
`ros2 topic echo <topic_name>`
