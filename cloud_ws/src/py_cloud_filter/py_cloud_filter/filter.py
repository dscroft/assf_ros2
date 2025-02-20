import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2
from sensor_msgs.msg import PointCloud2

import random
import numpy as np

class Filter(Node):
    def __init__(self):
        """
        ROS Node that filters the ego vehicle from a point cloud.

        Subscriptions
        -------------
        ~/input (sensor_msgs/msg/PointCloud2): The point cloud to republish

        Publishers
        ----------
        ~/output (sensor_msgs/msg/PointCloud2): The filtered point cloudd
        """
        super().__init__("py_filter")
        self.__sub = self.create_subscription(PointCloud2, "~/input", self.__callback, 10)
        self.__pub = self.create_publisher(PointCloud2, "~/output", 10)

    def __callback(self, msg: PointCloud2):
        """Get the point cloud, perform some transformations, and publish them."""

        # === GET INPUT ===
        self.get_logger().info("Received point cloud message")

        # convert ros2 message into numpy array        
        numpy_cloud = sensor_msgs_py.point_cloud2.read_points_numpy(msg, ["x", "y", "z"])
        # if you want other formats:
        # - for a pure python list 
        #         numpy_cloud.tolist()
        # - for pcl
        #         pcl.PointCloud(numpy_cloud)

        # === PROCESSING ===
        self.get_logger().info(f"Processing {len(numpy_cloud)} points")

        # replace this code so that it filters out the ego vehicle from the input cloud
        #   this code currently generates a random point cloud
        #   it is possible to do this in pure python but using numpy or pcl will give 
        #   better performance
        # at the moment that code generates a series of random points so that you can see
        #   how the data is created and stored
        out_cloud = []
        random_point = lambda: [ random.uniform(-3, 3) for _ in range(3) ]
        for i in range(1000):
            x, y, z = random_point()
            out_cloud.append((x,y,z))

        # === PUBLISH OUTPUT ===
        self.get_logger().info(f"Publishing {len(out_cloud)} points")

        # convert the output point cloud to a ros message
        out_msg = sensor_msgs_py.point_cloud2.create_cloud_xyz32(
            msg.header,
            out_cloud
        )

        # publish the output point cloud
        self.__pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Filter()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
