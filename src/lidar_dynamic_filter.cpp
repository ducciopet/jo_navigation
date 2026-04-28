#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "jo_msgs/msg/obstacle_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class LidarDynamicFilter : public rclcpp::Node
{
public:
  LidarDynamicFilter()
  : Node("lidar_dynamic_filter"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("tracking_timeout", 0.5);
    declare_parameter("obstacle_padding", 0.10);

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points_filtered", 10);

    obs_sub_ = create_subscription<jo_msgs::msg::ObstacleArray>(
      "/onboard_detector/tracked_dynamic_obstacles",
      10,
      std::bind(&LidarDynamicFilter::obstaclesCallback, this, std::placeholders::_1));

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points",
      10,
      std::bind(&LidarDynamicFilter::lidarCallback, this, std::placeholders::_1));
  }

private:
  struct ObsBox
  {
    double x;
    double y;
    double z;
    double hx;
    double hy;
    double hz;
  };

  void obstaclesCallback(const jo_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    latest_obstacles_.clear();
    latest_frame_ = msg->header.frame_id;
    last_obstacles_time_ = now();

    const double padding = get_parameter("obstacle_padding").as_double();

    for (const auto & obs : msg->obstacles) {
      ObsBox box;
      box.x = obs.pose.position.x;
      box.y = obs.pose.position.y;
      box.z = obs.pose.position.z;

      box.hx = obs.size.x * 0.5 + padding;
      box.hy = obs.size.y * 0.5 + padding;
      box.hz = obs.size.z * 0.5 + padding;

      latest_obstacles_.push_back(box);
    }
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<ObsBox> obstacles;
    std::string obstacle_frame;
    bool active = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      const double timeout = get_parameter("tracking_timeout").as_double();

      active =
        last_obstacles_time_.has_value() &&
        (now() - *last_obstacles_time_).seconds() < timeout &&
        !latest_obstacles_.empty();

      if (active) {
        obstacles = latest_obstacles_;
        obstacle_frame = latest_frame_;
      }
    }

    if (!active) {
      pub_->publish(*msg);
      return;
    }

    if (obstacle_frame.empty()) {
      pub_->publish(*msg);
      return;
    }

    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;

    for (const auto & field : msg->fields) {
      if (field.name == "x") {
        x_offset = static_cast<int>(field.offset);
      } else if (field.name == "y") {
        y_offset = static_cast<int>(field.offset);
      } else if (field.name == "z") {
        z_offset = static_cast<int>(field.offset);
      }
    }

    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
      pub_->publish(*msg);
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;

    try {
      tf_msg = tf_buffer_.lookupTransform(
        obstacle_frame,
        msg->header.frame_id,
        rclcpp::Time(0),
        tf2::durationFromSec(0.05));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "TF not ready, publishing unfiltered cloud: %s",
        ex.what());

      pub_->publish(*msg);
      return;
    }

    tf2::Transform tf;
    tf2::fromMsg(tf_msg.transform, tf);

    const uint32_t point_step = msg->point_step;
    const uint32_t n_points = msg->width * msg->height;
    const uint8_t * raw = msg->data.data();

    std::vector<uint8_t> kept;
    kept.reserve(n_points * point_step);

    for (uint32_t i = 0; i < n_points; ++i) {
      const uint8_t * base = raw + i * point_step;

      float px_lidar = 0.0f;
      float py_lidar = 0.0f;
      float pz_lidar = 0.0f;

      std::memcpy(&px_lidar, base + x_offset, sizeof(float));
      std::memcpy(&py_lidar, base + y_offset, sizeof(float));
      std::memcpy(&pz_lidar, base + z_offset, sizeof(float));

      tf2::Vector3 p_lidar(px_lidar, py_lidar, pz_lidar);
      tf2::Vector3 p_obs_frame = tf * p_lidar;

      const double px = p_obs_frame.x();
      const double py = p_obs_frame.y();
      const double pz = p_obs_frame.z();

      bool inside_dynamic_obstacle = false;

      for (const auto & obs : obstacles) {
        const bool inside =
          std::abs(px - obs.x) <= obs.hx &&
          std::abs(py - obs.y) <= obs.hy &&
          std::abs(pz - obs.z) <= obs.hz;

        if (inside) {
          inside_dynamic_obstacle = true;
          break;
        }
      }

      if (!inside_dynamic_obstacle) {
        kept.insert(kept.end(), base, base + point_step);
      }
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = 1;
    out.width = static_cast<uint32_t>(kept.size() / point_step);
    out.fields = msg->fields;
    out.is_bigendian = msg->is_bigendian;
    out.point_step = point_step;
    out.row_step = static_cast<uint32_t>(kept.size());
    out.data = std::move(kept);
    out.is_dense = false;

    pub_->publish(out);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<jo_msgs::msg::ObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mutex_;
  std::vector<ObsBox> latest_obstacles_;
  std::string latest_frame_;
  std::optional<rclcpp::Time> last_obstacles_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarDynamicFilter>());
  rclcpp::shutdown();
  return 0;
}