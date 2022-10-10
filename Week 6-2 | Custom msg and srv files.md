# 6-2 | Creating custom msg and srv files

<br/>

## 1. Create a new package

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

<br/>

## 2. Create custom definitions

<br/>

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

<br/>

## 3. Make changes to the existing files

<br/>

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

<br/>

## 4. Build the ```tutorial_interfaces``` package

<br/>

Now, all the files are ready to be utilized. We will navigate back to the root directory ```ros2_ws``` and run the code below to build 'tutorial_interfaces' package:
```
colcon build --packages-select tutorial_interfaces
```

<br/>

## 5. Confirm msg and srv creation

<br/>

After that, we will open a new terminal, go to our workspace ```ros2_ws``` and source it:
```
. install/setup.bash
```
To assure that the interface which I have created working properly, run the following command:
```
ros2 interface show tutorial_interfaces/msg/Num
```
which returns me: <br/>
![in64](https://user-images.githubusercontent.com/90167023/194913585-12aa1d95-7375-4f03-ae49-ea0013cc7294.png)

<br/>

And the following
```
ros2 interface show tutorial_interfaces/srv/AddThreeInts
```
returns me: <br/>
![sum64](https://user-images.githubusercontent.com/90167023/194913821-f2f712e1-b9cc-48f3-a31e-f554a6517591.png)

<br/>

<br/>

## 6. Testing ```Num.msg``` with publisher and subscriber

<br/>

Next, we will test the changes with publisher and subscriber. To do this, we navigate to ```ros2_ws/src/py_pubsub/py_pubsub``` and open ```publisher_member_function.py``` to edit:
```
gedit publisher_member_function.py
```
Then add the following lines that has the comment 'CHANGE' after them and save the file:
```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num    # CHANGE


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Num, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Num()                                           # CHANGE
        msg.num = self.i                                      # CHANGE
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.num)  # CHANGE
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

<br/>

Next, open ```subscriber_member_function.py``` to edit:
```
gedit subscriber_member_function.py
```
Then add the following lines that has the comment 'CHANGE' after them and save the file:
```
import rclpy
from rclpy.node import Node

from tutorial_interfaces.msg import Num        # CHANGE


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Num,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            self.get_logger().info('I heard: "%d"' % msg.num) # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

<br/>

After this step, we will navigate to the ```tutorial_interfaces``` directory and edit 'CMakeLists.txt' file according to the following lines of code and save:
```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)                         # CHANGE

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp tutorial_interfaces)         # CHANGE

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp tutorial_interfaces)     # CHANGE

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

<br/>

Finally, we will go to ```ros2_ws/src/py_pubsub``` again and edit 'package.xml' by adding the line of code below and save:
```
<exec_depend>tutorial_interfaces</exec_depend>
```

<br/>

All changes have been done. It is time to build the package:
```
colcon build --packages-select py_pubsub
```

<br/>

Then we will open two new terminals and go to the root of our workspace ```ros2_ws``` and source it in both of them:
```
. install/setup.bash
```
Then, we will run:
```
ros2 run py_pubsub talker
```
in one of them, and:
```
ros2 run py_pubsub listener
```
in another. Afterwards, we will get the following ouputs accordingly: <br/>
![talk](https://user-images.githubusercontent.com/90167023/194913084-7bdff8c4-960f-4684-8b9f-1df7aa899938.png)

<br/>

![listen](https://user-images.githubusercontent.com/90167023/194917387-bfe1c105-0f82-421e-a2f5-faf78ef794fc.png)

<br/>

## 7. Testing ```AddThreeInts.srv``` with service and client

<br/>

Now, it is time to test 'AddThreeInts.srv' file with service and client.<br/>
After navigating to the ```ros2_ws/src/py_srvcli/py_srvcli```, we will open the file 'service_member_function.py' and make changes as below:
```
from tutorial_interfaces.srv import AddThreeInts     # CHANGE

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)        # CHANGE

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c                                                  # CHANGE
        self.get_logger().info('Incoming request\na: %d b: %d c: %d' % (request.a, request.b, request.c)) # CHANGE

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

<br/>

We will also change 'client_member_function.py':
```
from tutorial_interfaces.srv import AddThreeInts       # CHANGE
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddThreeInts, 'add_three_ints')       # CHANGE
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddThreeInts.Request()                                   # CHANGE

    def send_request(self):
        self.req.a = int(sys.argv[1])
        self.req.b = int(sys.argv[2])
        self.req.c = int(sys.argv[3])                  # CHANGE
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info(
                    'Result of add_three_ints: for %d + %d + %d = %d' %                               # CHANGE
                    (minimal_client.req.a, minimal_client.req.b, minimal_client.req.c, response.sum)) # CHANGE
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

<br/>

Also, change 'CMakeLists.txt':
```
#...

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tutorial_interfaces REQUIRED)        # CHANGE

add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
  rclcpp tutorial_interfaces)                      #CHANGE

add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
  rclcpp tutorial_interfaces)                      #CHANGE

install(TARGETS
  server
  client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

<br/>

Finally, change 'package.xml':
```
<exec_depend>tutorial_interfaces</exec_depend>
```

<br/>

After making changes and saving, we will build the package:
```
colcon build --packages-select py_srvcli
```
Then, we will open two new terminals, source our root directory and run the following commands one each:
```
ros2 run py_srvcli service
```
```
ros2 run py_srvcli client 7 10 3
```

<br/>

That will give us the following outouts in those terminals: <br/>
![output1](https://user-images.githubusercontent.com/90167023/194922067-fff5402c-af83-450b-9fbc-099c64fa3452.png)

<br/>

![output2](https://user-images.githubusercontent.com/90167023/194922221-8a6bd853-2b9b-40ad-a4db-5ebe005cfafa.png)

## The end!
