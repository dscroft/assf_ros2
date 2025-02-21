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

You can stop the node by pressing `Ctrl+C` in the terminal window that is running the node.

## Examine node

We can use various ros commands to examine our running workspace. You will need to do this in a separate terminal window to the one running the node.


| Description                  | Command                      |
| ---------------------------- | ---------------------------- |
| See which nodes are running.      | `ros2 node list`             |
| See information about a node.     | `ros2 node info <node_name>` |
| See available topics.             | `ros2 topic list` |
| See information about a topic.    | `ros2 topic info <topic_name>` |
| See topic data as it's published. | `ros2 topic echo <topic_name>` |



<div style="border: 4px solid green; padding: 10px; margin: 10px;">

**Use the following commands above to examine the running nodes and topics and answer the following questions.**

1. What is the name of the topic that the node is publishing data to?
2. What type of message is being published on that topic?
3. What is the actual data that is being published?

<details>
    <summary>Answer 1</summary>

Using the `ros2 topic list` or the `ros2 node info /minimal_publisher` commands you can see that the node is publishing to the */chatter* topic.
</details>

<details>
    <summary>Answer 2</summary>

Using the `ros2 topic info /chatter` command you can see that the node is publishing a *std_msgs/String* message. 
This is just a string.
</details>

<details>
    <summary>Answer 3</summary>

Using the `ros2 topic echo /chatter` command you can see the actual data that is being published.

There should be a piece of text published every second that looks something like "Hello, world! 42".
</details>
</div>

<div style="border: 4px solid green; padding: 10px; margin: 10px;">

**What happens to the */chatter* topic if you run both publisher nodes at the same time?**

You will need to use two terminal windows, one to run each node.

<details>
    <summary>Answer</summary>

The information from the two nodes will be interleaved on the */chatter* topic.

Because each node is publishing a message every second and each node has it's own counter, the messages will appear twice as fast and the numbers will be out of sync.
</details>
</div>

## Mix and match

One of the key benefits of ROS is that the standardised message types allow 
you to mix and match nodes from different packages. 
This means that you can use a Python node to publish messages that are consumed by a 
C++ node or vice versa (or Java, C#, Swift, Node.js, Ada, Rust etc).

It also means that you can reuse nodes from other projects without needed to modify them. 
This is particularly useful when you are working on a project that requires a specific 
sensor or actuator that has already been implemented in another project.

For example, if using a Velodyne Lidar, we do not need to write our own driver node. 
We can use the manufacter provided one [here](https://docs.ros.org/en/humble/p/velodyne_driver/).

This applies to many hardware providers and a large amount of software functionality.

<div style="border: 4px solid green; padding: 10px; margin: 10px;">

**Try running combining nodes from the py_pubsub and cpp_pubsub packages by running one node from each.**

You will need to use two terminal windows, one to run each node.

- py_pubsub -> cpp_pubsub
- cpp_pubsub -> py_pubsub
</div>

[Back to README.md](../README.md)
