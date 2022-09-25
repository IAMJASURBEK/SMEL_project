# SMEL_project Task-1 | Kunishev Ozodjon - 12190259
## Step 1: Turtle installation
<br>

First we need to make sure our software is up to date
```
sudo apt update
```
![1](https://user-images.githubusercontent.com/90167023/192158679-6658ec3e-eb58-4d26-a1e1-c4ff92665eca.jpg)

<br />

Then, we will install turtleism
```
sudo apt install ros-rolling-turtlesim
```
![2](https://user-images.githubusercontent.com/90167023/192159258-4b09568d-90ce-4c8f-8796-479f9a00f851.png)

<br />

After that we should make sure the turtleism package is installed
```
ros2 pkg executables turtlesim
```

![3](https://user-images.githubusercontent.com/90167023/192159364-c884d30d-cf75-4bb7-8e5b-44f436a9a103.png)

<br />

## Step 2: Run & use turtleism

<br />

After installing all te required packages, we can run turtleism with code below:
```
ros2 run turtlesim turtlesim_node
```
![4](https://user-images.githubusercontent.com/90167023/192159539-fa3c78b8-1dc7-47bf-b15e-15e75dede6f8.png)

<br />

In order to control the turtle with the arrow keys on our keyboard, we should run the following code in a new terminal
```
ros2 run turtlesim turtle_teleop_key
```
![5](https://user-images.githubusercontent.com/90167023/192159988-9bac1b27-1dcf-48fe-9315-121852ebf928.png)

<br />

## Step 3: Install RQT tool

<br />

As I have already updated the software, I can directly install rqt using the following command:
```
sudo apt install ~nros-rolling-rqt*
```

<br />

To run rqt, just running the ```rqt``` command is enough. <br />
After running that command in terminal, we will see a new window: <br />
<br />

![6](https://user-images.githubusercontent.com/90167023/192160284-8ff6b41f-7254-4716-88de-0fb13db0ae40.png)

<br />

## Step 4: Creating asecond turtle using RQT

<br />

We can create another turtle using the RQT tool. In order to do that, we should go to the tab **Plugins > Services > Service cellar**. Then, choose **/spawn** from the dropdown options and give a name and coordinates for our second turtle.

<br />

![7](https://user-images.githubusercontent.com/90167023/192160718-2a46367c-b9af-4a32-9eaf-dd4784b95bdc.png)

<br />

We can also change the color and line width of drawing line of turtles. To do that, we select **/turtle1/set_pen** option from the dropdown menu and give particular values to the RGB and width in the RQT window. Then click **call**:

<br />

![8](https://user-images.githubusercontent.com/90167023/192161353-ce2472fa-a33a-497a-8cc4-8ee1561e96be.png)

<br />

To control the second turtle, we should open a new terminal while ```ros2 run turtlesim turtlesim_node```, ```ros2 run turtlesim turtle_teleop_key``` and ```rqt``` commands are running in separate terminals. Then, run the following command to remap from turtle1 to My_2_turtle:
```
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=My_2_turtle/cmd_vel
```
Now, we can control both turtles by switching to the corresponding terminals and pressing the arrow keys on our keyboard:

<br />

![9](https://user-images.githubusercontent.com/90167023/192161646-b9d938bd-6be7-4054-a486-82619323000d.png)

<br />

To close the windows, we simply press ```ctrl```+```c``` in the terminal.
