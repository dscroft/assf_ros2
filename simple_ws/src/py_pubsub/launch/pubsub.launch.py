from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # run an instance of the publisher node
    pub = Node( package='py_pubsub',
                executable='publisher',
                name='pub')

    # run an instance of the subscriber node
    sub = Node( package='py_pubsub',
                executable='subscriber',
                name='sub')

    # make sure to list all the nodes that you want to run
    return LaunchDescription([
        pub, sub
    ])
