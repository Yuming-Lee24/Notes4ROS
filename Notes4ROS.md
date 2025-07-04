# ROS Notes
## Ⅰ. ROS Basic Concepts
### 1. Node
A node is an **executable script or binary** inside a package. It does a specific job like processing sensor data or controlling hardware.
#### Functions
`rospy.init_node('hello_node')` Creates a node
` rospy.Rate()` Controls loop frequency
` rospy.is_shutdown()` Checks if the node should exit
#### Commands
`rosrun <package_name> <node_name>` Run a single node
`rosnode list` Show all running nodes
`rosnode info /<node_name>` Inspect a node’s topics/services
`rosnode list` Find Node Names
### 2. Package
A module for a function. Consists of several nodes. Each nodes cooperate together to accomplish a specific task.

    my_package/
    ├── scripts/			# Python scripts (nodes)
    ├── src/                # C++ source files (nodes)
	├── launch/             # Launch files
	├── msg/                # Custom message definitions
	├── srv/                # Custom service definitions
	├── CMakeLists.txt      # Build config
	└── package.xml			# Package metadata & dependencies

#### Commands
`catkin_create_pkg <pkg_name> [deps]` Create a new package  

`rospack list` List all available packages
`rospack find <pkg_name>` Show path to a specific package
`roscd <pkg_name>` Quickly change directory to a package
### 3. Launch File
The launch file is something to run several nodes.

    <launch>
	  <node name="my_node_1" 
	  pkg="my_package_1" 	
	  type="my_script.py" 
	  output="screen" />
	</launch>
#### Commands
`roslaunch <package_name> <file.launch>` Starts the launch file from the specified package
## Ⅱ. Topic
**ROS topics** enable nodes to **communicate asynchronously** by publishing and subscribing to **messages**.
### 1. Topic
A named channel for data, like a radio frequency
#### Commands
`rostopic list` List all active 
`rostopic echo <topic>` Print messages being published to a topic
`rostopic info <topic>` Show message type, publishers, subscribers
`rostopic type <topic>` Show the message type used by a topic
`rostopic pub <topic> <type> <msg>` Manually publish a message to a topic
### 2. Publisher
A node that sends data to a topic.
#### Commands
`pub = rospy.Publisher('/chatter', String, queue_size=10)` Create a publisher `pub` and topic `/chatter`
`pub.publish("Hello ROS!")`  Publish a message
#### Example

    import rospy
	from std_msgs.msg import String

	rospy.init_node('talker')
	pub = rospy.Publisher('chatter', String, queue_size=10)

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
	    pub.publish("Hello ROS!")
	    rate.sleep()


### 3. Subscriber
A node that receives data from a topic.
#### Commands
`sub = rospy.Subscriber('/chatter', String, callback)`  Listen to the topic and trigger the callback function
#### Example

    import rospy
	from std_msgs.msg import String

	def callback(msg):
	    rospy.loginfo("Received: %s", msg.data)

	rospy.init_node('listener')
	sub = rospy.Subscriber('chatter', String, callback)
	rospy.spin()
#### Note
In the callback function, `msg` is the message object automatically passed in by ROS — think of it as the actual message that was received.
### 4. Message
The data format sent over topics. 
#### Message Type
For example `std_msgs/String` is a message type. It belongs to the `std_msgs` message package. And the data format is specified in the file `std_msgs/String.msg`
#### Message Type import
`from std_msgs.msg import String` although 	`std_msgs` is a package, but due to Python compile, it should be written as `Message package.msg` and then import the message type.
#### Message Publish
If there are several fields in the message type, first create a message instance: `msg = String()`, then assign values, then publish using `pub.publish(msg)`.
