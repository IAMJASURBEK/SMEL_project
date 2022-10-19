# Launch
<br/>

## Creating a launch file
<br/>

>Goal: Creating a launch file to run a complex ROS 2 system.

<br/>

First, I will create a new directory to store my launch filesL
```
mkdir launch
```
Inside this directory, I will create a new file ```turtlesim_mimic_launch.py``` and paste the code below:
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
```
To run the launch file created above, I will enter into the directory I created earlier and run the following command:
```
ros2 launch turtlesim_mimic_launch.py
```
It will open two Turtlesim windows and I can see INFO messages in the terminal about which nodes my launch file has started:
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [turtlesim_node-1]: process started with pid [11714]
[INFO] [turtlesim_node-2]: process started with pid [11715]
[INFO] [mimic-3]: process started with pid [11716]
```
In order t see the system in action, I will open a new terminal and run the command below:
```
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}"
```
Now, I can see that both turtles follow the same path.

<br/>

To see the system in a graph, I will open another terminal and run the following code:
```
rqt_graph
```
The final result will be as below:
![rqt   circle turtles](https://user-images.githubusercontent.com/90167023/196712096-d268973c-fd64-4328-a9d8-890a4968cf26.png)

