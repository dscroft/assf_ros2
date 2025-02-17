#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>

class Filter : public rclcpp::Node {
public:
  Filter() : Node("cpp_filter") {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "~/input", 10, std::bind(&Filter::callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/output", 10);
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received point cloud message");

    // === GET INPUT POINT CLOUD ===
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(*msg, pcl_cloud);

    // === PROCESSING === 
    RCLCPP_INFO_STREAM(this->get_logger(), "Processing " << pcl_cloud.points.size() << " points");

    // replace this code so that it filters out the ego vehicle from the input cloud
    //   this code currently generates a random point cloud
    //   it is possible to do this in pure C++ and that is going to be the simplest to 
    //   coded but using the pcl library will probably be faster if you were doing this
    //   for real
    // at the moment this code just generates a series of random points so that you can
    //   see how the data is created and stored
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::default_random_engine generator;
    generator.seed(msg->header.stamp.nanosec);
    std::uniform_real_distribution<float> distribution(-3.0, 3.0);

    for (size_t i = 0; i < 1000; ++i) {
      pcl::PointXYZ point;
      point.x = distribution(generator);
      point.y = distribution(generator);
      point.z = distribution(generator);
      out_cloud->points.push_back(point);
    }

    // === PUBLISH OUTPUT ===
    RCLCPP_INFO(this->get_logger(), "Publishing %zu points", out_cloud->points.size());

    // Convert the output point cloud to a ROS message
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*out_cloud, out_msg);
    out_msg.header = msg->header;

    // Publish the output point cloud
    pub_->publish(out_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Filter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
