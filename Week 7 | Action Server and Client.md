# Creating Action Server and Client
<br/>

## Step 1: Creating an action
<br/>

First, we navigate to our workspace ```ros2_ws/src``` and create a package with the name of ```action_tutorials_interfaces```:
```
ros2 pkg create action_tutorials_interfaces
```
Then we navigate to the ction_tutorials_interfaces and create a new folder called ```action```
```
mkdir action
```
Inside of this folder, we wcreate a new file called ```Fibonacci.action```:
```
gedit Fibonacci.action
```
Then input the following code inside of it:
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```
Then, we go back to the ```
