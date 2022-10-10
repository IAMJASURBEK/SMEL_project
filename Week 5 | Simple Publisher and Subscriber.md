# Python & Simple Publisher and Subscriber

<br />

## Step 1: Creating a Python package

<br />

At first, we open the terminal in our Virtual Machine and navigate to the ```ros2_ws/src``` folder that we created before. Then, we run the following code to create the package:
```
ros2 pkg create --build-type ament_python py_pubsub
```

<br />

## Step 2: Writing the publisher node

<br />

After that, the python package will be installed in a folder ```ros2_ws/src/py_pubsub/py_pubsub``` where we will navigate to and download the example talker code by running the following command:
```
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```
Then, if we type ```ls``` command, we can see two files ```publisher_member_function.py``` and ```__init__.py```. In order to open the file and edit using the text editor, we use the command ```open``` before the filename:
```
open publisher_member_function.py
```

### Adding dependencies
If we navigate one level back to the ```ros2_ws/src/py_pubsub```, we can see a file ```package.xml```. We open the file in a text editor using the command below:
```
open package.xml
```
Then, we must fill in the following tags using our personal data:
```
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="ozodjonkunishev@gmail.com">Ozodjon</maintainer>
<license>Apache License 2.0</license>
```
Next, we should add the following dependencies after the lines above.
```
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

<br />

### Adding an entry point
Next, we open the ```setup.py``` file and math the personal information with the ```package.xml``` file:
```
maintainer='Ozodjon',
maintainer_email='ozodjonkunishev@gmail.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```
In the ```entry_points``` field, we should add the following lines and save the file:
```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```

<br />

## Step 3: Writing the subscriber node

<br />

After navigating to the ```ros2_ws/src/py_pubsub/py_pubsub```, we enter the following code:
```
wget https://raw.githubusercontent.com/ros2/examples/humble/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```
Then we will have three files in the folder: ```__init__.py```, ```publisher_member_function.py```, ```subscriber_member_function.py```.<br />
Now, we reopen the ```setup.py``` file and add an entry point for the subscriber as below:
```
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

<br />

## Step 4: Building and running

<br />

After saving and closing the file, we run the following code in the ```ros2_ws``` repository to check for missing dependencies.
```
rosdep install -i --from-path src --rosdistro humble -y
```
Then, we can build our new package:
```
colcon build --packages-select py_pubsub
```
Then we open a new terminal and source the setup files:
```
. install/setup.bash
```
Then, we can run the Talker node:
```
ros2 run py_pubsub talker
```
As an output, we will see that the terminal publishes "Hello World" Message in every 0.5 seconds. To run the Listener node, we open another terminal and source the setup files again, then run the listener node:
```
. install/setup.bash
```
```
ros2 run py_pubsub listener
```
As an output, the second terminal shows wxactly what the talker node is publishing.
![talker](https://user-images.githubusercontent.com/90167023/192698866-2a130d95-6534-4f8b-ae20-66c85e0340b4.png)
![listener](https://user-images.githubusercontent.com/90167023/192699020-a675662c-f6e5-4c0d-8224-1cffdfd0ab65.png)


## The End!!!
