# Composing multiple nodes in a single process
<br/>

>So far, I have created several components and installed number of packages. Now, I am going to use some of the components (nodes) in one a single process. 

<br/>

First, I will check what kind of components available in my workspace:
```
ros2 component types
```
After confirming the list of available components in my workspace, I will start a component container:
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
The output of the command will be as below:
![multiple nodes0](https://user-images.githubusercontent.com/90167023/196695332-f54316f2-1525-4466-8479-e4990fbdc2d1.png)

<br/>

I can also combine the couple components together by running:
```
ros2 run composition manual_composition
```
To open each component library and create one instance of each node, I can run the below command:
```
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so
```
This will return a repeated output for each sent and received message.
<br/>

I can also use:
```
ros2 launch composition composition_demo.launch.py
```
to compose and start the components at the same time.

<br/>

Now, I will try to unload the components from the container. To do that, I will run the command below:
```
ros2 run rclcpp_components component_container
```
Then, I will verify the container is running:
```
ros2 component list
```
After verifying, I will run load listener and talker components in the second terminal:
```
ros2 component load /ComponentManager composition composition::Talker
ros2 component load /ComponentManager composition composition::Listener
```
After that, I will unload the node from the component container using a unique ID:
```
ros2 component unload /ComponentManager 1 2
```
My terminal will return:
```
Unloaded component 1 from '/ComponentManager' container
Unloaded component 2 from '/ComponentManager' container
```
Now, I can see that, the messages from talker and listener have stopped in the forst terminal.

<br/>

It is possible to remap container name and namespace using standard command line arguments:
```
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns
```
Now, in the second shell, I can load the components with the updated container name:
```
ros2 component load /ns/MyContainer composition composition::Listener
```
I can also adjust the names and namespaces of the components through arguments. To do that, I will run the component container in the first terminal:
```
ros2 run rclcpp_components component_container
```
In the second terminal I can remap: <br/>
Node name:
```
ros2 component load /ComponentManager composition composition::Talker --node-name talker2
```
Namespace:
```
ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
```
Both:
```
ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2
```
To see the entries, I can use ```ros2``` command line:
```
ros2 component list
```
... which returns me the entries as below:
```
/ComponentManager
   1  /talker2
   2  /ns/talker
   3  /ns2/talker3
```
I can also pass additional arguments into components as foolows:
```
ros2 component load /ComponentManager composition composition::Talker -e use_intra_process_comms:=true
```

<br/>

![multiple nodes](https://user-images.githubusercontent.com/90167023/196700462-e7725f5d-7bd1-463f-b04a-f70272f3f057.png)

