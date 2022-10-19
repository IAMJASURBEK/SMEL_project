# Building a visual robot model from scratch
<br/>

>Goal: Learn how to build a visual model of a robot that you can view in Rviz
<br/>

After installing ```joint_state_publisher``` and ```urdf_tutorial``` packages, I will examine the model:
```
ros2 launch urdf_tutorial display.launch.py model:=urdf/01-myfirst.urdf
```
After launching ```display.launch.py```, I will end up with RViz showing me the following:<br/>
![rviz](https://user-images.githubusercontent.com/90167023/196744244-18bdce64-6713-45c1-b988-1094f599c69b.png)

<br/>

Now I will try to add multiple shapes/links. If I just add more link elements to the urdf, the parser won’t know where to put them. So, I have to add joints. Joint elements can refer to both flexible and inflexible joints. I’ll start with inflexible, or fixed joints. [Source: 02-multipleshapes.urdf]:
```
<?xml version="1.0"?>
<robot name="multipleshapes">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
  </joint>

</robot>
```
Now, I defined a 0.6m x 0.1m x 0.2m box. Let me try to see the shape:
```
ros2 launch urdf_tutorial display.launch.py model:=urdf/02-multipleshapes.urdf
```
![rviz2](https://user-images.githubusercontent.com/90167023/196745064-9b7739f3-855f-41b2-a90c-e32fbdb8d9dc.png)
<br/>

R2D2’s leg attaches to the top half of his torso, on the side. So that’s where I specify the origin of the JOINT to be. Also, it doesn’t attach to the middle of the leg, it attaches to the upper part, so I must offset the origin for the leg as well. I also rotate the leg so it is upright. [Source: 03-origins.urdf]:
```
<?xml version="1.0"?>
<robot name="origins">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

</robot>
```
Now, it will look like this:
```
