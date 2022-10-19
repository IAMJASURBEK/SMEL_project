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
The final result will be as below: <br/>
![rqt   circle turtles](https://user-images.githubusercontent.com/90167023/196712096-d268973c-fd64-4328-a9d8-890a4968cf26.png)

<br/>

## 2. Integrating launch files into ROS 2 packages
<br/>

>Goal: Adding a launch file to a ROS 2 package

<br/>

First, I will create a workspace:
```
mkdir -p launch_ws/src
cd launch_ws/src
```
The, I will create the new package:
```
ros2 pkg create py_launch_example --build-type ament_python
```
Inside of this package, I will create a new directory called ```launch```. Then, I will open the ```setup.py``` file and make changes to it according to the code lines below:
```
import os
from glob import glob
from setuptools import setup

package_name = 'py_launch_example'

setup(
    # Other parameters ...
    data_files=[
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*'))
    ]
)
```
Then, inside of my newly-created ```launch``` directory, I will create a new file called ```my_script_launch.py``` and paste the following code in:
```
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp',
            executable='talker',
            name='talker'),
  ])
```
Then, I will navigate to the root of my workspace and build the package as well as source it:
```
colcon build
```
```
. install/setup.bash
```
After being successful in building, I can run the launch file as below:
```
ros2 launch py_launch_example my_script_launch.py
```
The output is:
<br/>

![integrating launch files](https://user-images.githubusercontent.com/90167023/196714393-d8e2a359-0ac1-4ac7-a54d-5245682cc043.png)
