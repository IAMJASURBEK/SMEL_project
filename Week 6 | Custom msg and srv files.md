# Creating custom msg and srv files

<br/>

First, we will navigate to the ```ros2_ws/src``` folder and install a new package called 'tutorial interfaces':
```
ros2 pkg create --build-type ament_cmake tutorial_interfaces
```
After that, we will create two folders: ```msg``` and ```srv``` for .msg and .srv files accordingly:
```
mkdir msg

mkdir srv
```
Next, we will go to the ```msg``` folder we have just made, and create a new file called ```Num.msg```:
```
gedit Num.msg
```
Then we will add the following line into the file and save it:
```
int64 num
```
After that, we will navigate back to ```tutorial_interfaces/srv``` folder, and create a new file called ```AddThreeInts.srv``` and paste the below code in it:
```
int64 a
int64 b
int64 c
---
int64 sum
```
If we go back to the ```tutorial_interfaces```, we can see a text file ```CMakeLists.txt```. We will open it and append the below code:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
 )
 ```
 There is also a file called ```package.xml``` in the same directory. We will open it and change it by adding the following:
 ```
 <build_depend>rosidl_default_generators</build_depend>

<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>
```
Now, all the files are ready to be utilized. We will navigate back to the root directory ```ros2_ws``` and run the code below to build 'tutorial_interfaces' package:
```
colcon build --packages-select tutorial_interfaces
```
After that, we will open a new terminal, go to our workspace ```ros2_ws``` and source it:
```
. install/setup.bash
```
To assure that the interface which I have created working properly, run the following command:
```
ros2 interface show tutorial_interfaces/msg/Num
```
which returns me:
