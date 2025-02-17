[Back to README.md](../README.md)

# ROS Introduction

A ROS2 workspace is a directory that contains all the packages that you are working on. Each package contains nodes, messages, services and other ROS2 components.

Any meaningful ROS2 project will have multiple packages, each containing a different aspect of the project. For example, a robot project might have packages for motor control, sensor reading, path planning and obstacle avoidance.


## Building

In order to build your ROS2 workspace nodes, make sure that you are in the root of your workspace and run colcon.

```bash
colcon build
```

For example, if you have just opened a new terminal in your codespace and you wanted to build the simple_ws workspace; the commands will be along the lines of:

```bash
cd simple_ws
colcon build
```

This will build all the packages in your workspace regardless of language. If you only want build a specific package, you can use the `--packages-select` flag.

```bash
colcon build --packages-select <package_name>
```

colon will identify any failed builds and provide you with the error messages. If you have a failed build, you will need to fix the error before you can run your nodes.

## Sourcing

Once you have successfully built your workspace you need to source it. 
Sourcing the workspace is important because it tells ROS where to find your packages, what packages are available and allows to distinguish between different workspaces which can be important if you have multiple projects or versions of ROS on the same machine.

From the root of your workspace, run the following command:

```bash
source install/setup.bash
```


## Running nodes

To run a node, you can use the `ros2 run` command. This command takes the package name and the executable name as arguments. Make sure that you have sourced your workspace before trying to run the node.

```bash
ros2 run <package_name> <executable_name>
```

For example, the following will run the publisher node from the py_pubsub package which forms part of the simple_ws workspace.

```bash
ros2 run py_pubsub publisher
```

[Back to README.md](../README.md)