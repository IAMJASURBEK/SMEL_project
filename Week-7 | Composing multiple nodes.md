# Composing multiple nodes in a single process
<br/>
So far, I have created several components and installed number of packages. Now, I am going to use some of the components (nodes) in one a single process. First, I will check what kind of components available in my workspace:
```
ros2 component types
```
IAfter confirming the list of available components in my workspace, I will start a component container:
```
ros2 run rclcpp_components component_container
```
Then I open another terminal and check the container is running:
```
ros2 component list
```
This command gives me an output of ```/ComponentManager``` which is the name of the current component. Then, I will load the talker component in the second terminal that I have opened:
```
ros2 component load /ComponentManager composition composition::Talker
```
I can see the Talker component has been loaded with a unique ID:
```
Loaded component 1 into '/ComponentManager' container node as '/talker'
```
In the second terminal, I will load the listener component too:
```
ros2 component load /ComponentManager composition composition::Listener
```
... and, as an output, I can see that Listener component was loaded as a component 2:
```
Loaded component 2 into '/ComponentManager' container node as '/listener'
```
To see the state of my container, I can run the below code below:
```
ros2 component list
```
... which gives me an output of:
```
/ComponentManager
   1  /talker
   2  /listener
```
Now, I will run the below command in the first shell:
```
ros2 run rclcpp_components component_container
```
In the second terminal, I will run the code below to execute the loaded components:
```
ros2 component load /ComponentManager composition composition::Server
ros2 component load /ComponentManager composition composition::Client
```
