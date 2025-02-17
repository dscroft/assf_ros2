# ROS2 ASSF Lab

This lab assumes that you have some familiarity with the Linux terminal, 
in particular directory navigation commands.

This lab will take you through the process of:

- Building a ROS2 workspace.
- Running ROS2 nodes.
- Examining ROS2 nodes and topics.
- Combining nodes from multiple ROS2 packages.
- Editing ROS2 nodes.

[Running ROS2 in a Docker container via a GitHub Codespace](.guide/CODESPACES.md)

[Linux terminal commands](.guide/TERMINAL.md)

[ROS introduction](.guide/ROS.md)

[Examining ROS environment](.guide/ROS_EXAMINE.md)

## Structure



## Mix and match

One of the key benefits of ROS is that the standardised message types allow 
you to mix and match nodes from different packages. 
This means that you can use a Python node to publish messages that are consumed by a 
C++ node or vice versa.

It also means that you can reuse nodes from other projects without needed to modify them. 
This is particularly useful when you are working on a project that requires a specific 
sensor or actuator that has already been implemented in another project.

**Try running combining nodes from the py_pubsub and cpp_pubsub packages by running one node from each.**

- py_pubsub -> cpp_pubsub
- cpp_pubsub -> py_pubsub



### Building the Workspace
Once you've made sure you're in the correct directory, run the following command to build the workspace:
```bash
source build.sh
```
**Do not run `./build.sh` or `bash build.sh` as it will not have the intended effect.** You must use `source build.sh` or its shorthand `. build.sh`.

Lastly, run the following command to configure your shell:
```bash
source ~/.bashrc
```
The two commands above need to be run _only once_ after you start the container or codespace. These commands need to be run again only if you kill the container or create a new codespace.

## Running the Nodes

### Turtlesim
To run the turtlesim node, run the following command:
```bash
ros2 run turtlesim turtlesim_node
```
If you are developing locally, the turtlesim GUI will open automatically in a separate window through XQuartz/MobaXTerm. If you're using GitHub Codespaces, you need to view the Desktop to see the turtlesim GUI. See the [Desktop Viewer](#desktop-viewer) section below.

### Teleop
To run the teleop node to move the turtle with your keyboard, run the following command:
```bash
ros2 run turtlesim turtle_teleop_key
```

### Move Turtle (Your Custom Node)
To run your custom node, run the following command:
```bash
ros2 run my_turtlesim move_turtle
```

## Desktop Viewer
If you're using GitHub Codespaces, to view the turtlesim GUI, perform the following:
1. Press Ctrl/Cmd + Shift + P to open the command palette.
2. Type ">Ports" and select "Ports: Focus on Ports View".
3. In the "Ports" view, right-click on `6080` in the first column and select "Open in Browser" to open the desktop in a new tab. Alternatively, select "Preview in Editor" to view the desktop in the codespace.
5. You may need to wait a few minutes for the desktop to load.
6. Once you see the "noVNC" logo, click the "Connect" button to open the desktop.
7. The turtlesim GUI will be visible in the desktop (if you've started the turtlesim node).

## Topics
### `/turtle1/cmd_vel`
- Type: `geometry_msgs/msg/Twist`
- Description: The velocity command for the turtle. The turtle will move forward/backward if X linear velocity is positive/negative and rotate counterclockwise/clockwise if the Z angular velocity is positive/negative.

### `/turtle1/pose`
- Type: `turtlesim/msg/Pose`
- Description: The pose of the turtle. The pose includes the X and Y position of the turtle, the orientation of the turtle, and the linear and angular velocities of the turtle.

## Services
### `/clear`
- Type: `std_srvs/srv/Empty`
- Description: Clears all drawings. Does not affect the turtle's position.

### `/reset`
- Type: `std_srvs/srv/Empty`
- Description: Clears all drawings, resets the turtle to its initial position and orientation (in the center of the screen facing right), resets the pen color, and changes the sprite of the turtle.

### `/turtle1/set_pen`
- Type: `turtlesim/srv/SetPen`
- Description: Sets the pen properties for the turtle. You can specify the color, width, and whether the pen is on or off.

### `/turtle1/set_parameters`
- Type: `turtlesim/srv/SetParameters`
- Description: Sets the parameters for the turtle. You can specify the background r, g, and b values. Used only for setting the background color via code. To set the background color using the terminal, see the [parameters](#parameters) section below.

## Parameters
### `/turtlesim/background_(r|g|b)`
- Type: `int`
- Description: The red, green, and blue values of the background color of the turtlesim GUI.

## ROS Commands
You can append the `-h` flag to any command to view a message describing the command and its options.

### Nodes
- To list all nodes:
    ```bash
    ros2 node list
    ```
- To view information about a node:
    ```bash
    ros2 node info /node_name
    ```
    Replace `/node_name` with the name of the node you want to view information about.

### Topics
- To list all topics:
    ```bash
    ros2 topic list
    ```
- To view information about a topic:
    ```bash
    ros2 topic info /topic_name
    ```
    Replace `/topic_name` with the name of the topic you want to view information about.
- To view the messages being published on a topic:
    ```bash
    ros2 topic echo /topic_name
    ```
    Replace `/topic_name` with the name of the topic you want to view messages from.
- To publish a message on a topic:
    ```bash
    ros2 topic pub /topic_name message_type '{data: value}'
    ```
    Replace `/topic_name` with the name of the topic you want to publish a message to, `message_type` with the type of message you want to publish, and `{data: value}` with the values you want to publish. The values must be in YAML format and must match the message type.

    - Add the `-1` flag to publish only once:
        ```bash
        ros2 topic pub -1 /topic_name message_type '{data: value}'
        ```
    - Add the `-r rate` flag to publish at a specific rate in Hz:
        ```bash
        ros2 topic pub -r rate /topic_name message_type '{data: value}'
        ```
        Replace `rate` with the rate you want to publish at.

    *Tip:* After typing the message type, type a space, quote, and the first letter of the message type, then press Tab to auto-complete the message type.
- To view the rate at which messages are being published on a topic:
    ```bash
    ros2 topic hz /topic_name
    ```
    Replace `/topic_name` with the name of the topic you want to view the rate of messages being published on.

### Services
- To list all services:
    ```bash
    ros2 service list
    ```
- To view information about a service:
    ```bash
    ros2 service info /service_name
    ```
    Replace `/service_name` with the name of the service you want to view information about.
- To call a service:
    ```bash
    ros2 service call /service_name service_type '{data: value}'
    ```
    Replace `/service_name` with the name of the service you want to call, `service_type` with the type of service you want to call, and `{data: value}` with the values you want to call the service with. The values must be in YAML format and must match the service type.

### Parameters
- To list all parameters:
    ```bash
    ros2 param list
    ```
- To get the value of a parameter:
    ```bash
    ros2 param get /node_name parameter_name
    ```
    Replace `/node_name` with the name of the node the parameter is in and `parameter_name` with the name of the parameter you want to get the value of.
- To set the value of a parameter:
    ```bash
    ros2 param set /node_name parameter_name value
    ```
    Replace `/node_name` with the name of the node the parameter is in, `parameter_name` with the name of the parameter you want to set the value of, and `value` with the value you want to set the parameter to.

### Messages
- To view the definition of a message:
    ```bash
    ros2 interface show package_name/(msg or srv)/message_type
    ```
    Replace `package_name` with the name of the package the message is in, `(msg or srv)` with either `msg` for messages or `srv` for services, and `message_type` with the type of message you want to view the definition of.

## Additional Resources
- [ROS2 Documentation](https://docs.ros.org/en/jazzy/index.html)
- [Bash Command Cheatsheet](https://www.educative.io/blog/bash-shell-command-cheat-sheet)