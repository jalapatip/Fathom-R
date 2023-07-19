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

#### Unit Tests
* Run unit tests using colcon
`colcon test --package-select csv_processor`

### Package and Design Navigation
#### Message Interface
* Message interface contains the `Data.msg` message structure that is published to the `\data` topic
* For time, String was used instead of Int32. This way, the subsequent nodes are not limited by the date_time config used by the parent node (like relative vs absolute time).

#### CSV Processor
* This package contains all the python, config, and setup files to run and maintain the nodes and topics and also the corresponding helper functions.

#### Publisher Node
* The publisher node parses a `.csv` file and publishes each row at a frequency of `3Hz`. For space optimization, `csv.reader()` is used to read one line at a time. Also, the file opened as a read binary to ensure universal file system compatibility.
* Given the use case is simple and straight forward, I avoided using complex exception handling for data format errors and logged the critical errors instead of throwing an exception.
* In the interest of readability, instead of viewing info logs as `stdout`, I added a script to save them to a text file. This also helps us better track the exception messages if any.
* For data validation, I used the most preliminary checks.
* If this were to be designed for a more complex usecase, focusing on the single responsibility principle and segregating the solution into multiple classes would be a good next step. 

#### Subscriber Node
* The subscriber node listens to the `\data` topic, keeps track of the time value from the previous message, computes the difference w.r.t to the current message, and publishes the message as a `Float32` value. Given the timestamp is primitively an integer, it can also be modelled as `Int32`.

### Test cases
#### Publisher Unit Test
* Verify that publisher is publishing at least 2 messages in 3 seconds to ensure that the sender is working.
* The `Data` message published by the publisher is same as the actual record in the csv (using assertEqual). Since we are using a static csv file to load the data, I used the same file for unit testing instead of stubbing the values. A good next step would be to test sad an brown cases for exception handling.

#### Subscriber Unit Test
* The test verifies that the time differance published in the `\diff` topic is equal to the time differance in the csv file. 
* The test also verifies that the subscriber is actively ingesting and publishing messages.

### Good Design Practices
* The queue size is set to the default 10 which is sufficient given the frequency of the publisher and the processing capability of the subscriber.
* The codeflows and dataflows can be improved to make the application extendable.
* Unit and integration tests can be added to test all corner cases.
* 