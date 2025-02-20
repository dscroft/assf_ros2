[Back to README.md](../README.md)

# Simple activity

Make sure that you are in the simple_ws directory, have built the workspace and sourced it.

The simple_ws workspace contains two packages, py_pubsub and cpp_pubsub.
These are Python and C++ versions of simple publisher and subscriber nodes.

Pick your preferred language and examine the code for the publisher node.

- Python: [simple_ws/src/py_pubsub/py_pubsub/publisher.py](../simple_ws/src/py_pubsub/py_pubsub/publisher.py)
- C++: [simple_ws/src/cpp_pubsub/src/publisher.cpp](../simple_ws/src/cpp_pubsub/src/publisher.cpp)

This is pretty much the simplest ROS node that you can create. 
The node publishes a message, in this case a std_msgs/String message.
To a topic, in this case the /chatter topic.

## Run node

Run the publisher node by running one of the following commands.

```bash
ros2 run py_pubsub publisher
```

```bash
ros2 run cpp_pubsub publisher
```

You should see various status messages as the node starts up and begins publishing messages.
This information is useful for debugging and understanding what the node is doing, it is not necessary for the node to function correctly and it is not the same as the ROS topics that the node is publishing.

## Examine node

We can use various ros commands to examine our running workspace. You will need to do this in a separate terminal window to the one running the node.

<div style="color: green">Use the following commands to examine the running node and topic.</div>

| Description                  | Command                      |
| ---------------------------- | ---------------------------- |
| See which nodes are running. | `ros2 node list`             |
| See information about node.  | `ros2 node info <node_name>` |
| See available topics.        | `ros2 topic list` |
| See information about topic. | `ros2 topic info <topic_name>` |
| See topic data as it's published. | `ros2 topic echo <topic_name>` |



## Mix and match

One of the key benefits of ROS is that the standardised message types allow 
you to mix and match nodes from different packages. 
This means that you can use a Python node to publish messages that are consumed by a 
C++ node or vice versa.

It also means that you can reuse nodes from other projects without needed to modify them. 
This is particularly useful when you are working on a project that requires a specific 
sensor or actuator that has already been implemented in another project.

<div style="border: 4px solid green; padding: 10px; margin: 10px;">

**Try running combining nodes from the py_pubsub and cpp_pubsub packages by running one node from each.**

You will need to use two terminal windows, one to run each node.

- py_pubsub -> cpp_pubsub
- cpp_pubsub -> py_pubsub
</div>

[Back to README.md](../README.md)
