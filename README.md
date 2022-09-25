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

## Step 2: Run turtleism

<br />

After installing all te required packages, we can run turtleism with code below:
```
ros2 run turtlesim turtlesim_node
```
