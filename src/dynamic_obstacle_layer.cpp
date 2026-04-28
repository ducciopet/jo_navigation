#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "jo_msgs/msg/obstacle_array.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "visualization_msgs/msg/marker_array.hpp"

namespace jo_sim
{

class DynamicObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  DynamicObstacleLayer() = default;

  void onInitialize() override
  {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("obstacle_padding", rclcpp::ParameterValue(0.10));
    declareParameter("tracking_timeout", rclcpp::ParameterValue(1.0));
    declareParameter("velocity_inflation_k", rclcpp::ParameterValue(3.0));
    declareParameter("velocity_inflation_min", rclcpp::ParameterValue(0.05));

    auto node = node_.lock();

    if (node) {
      node->get_parameter(name_ + ".enabled", enabled_);
      node->get_parameter(name_ + ".obstacle_padding", obstacle_padding_);
      node->get_parameter(name_ + ".tracking_timeout", tracking_timeout_);
      node->get_parameter(name_ + ".velocity_inflation_k", velocity_inflation_k_);
      node->get_parameter(name_ + ".velocity_inflation_min", velocity_inflation_min_);

      clock_ = node->get_clock();

      sub_ = node->create_subscription<jo_msgs::msg::ObstacleArray>(
        "/onboard_detector/tracked_dynamic_obstacles",
        10,
        [this](const jo_msgs::msg::ObstacleArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_msg_ = msg;
          last_msg_time_ = clock_->now();
        });

      marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/dynamic_obstacle_footprints",
        rclcpp::QoS(1));
    }

    global_frame_ = layered_costmap_->getGlobalFrameID();
    current_ = true;
  }

  void reset() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_msg_.reset();
    previous_bounds_.clear();
    current_bounds_.clear();
    current_ = true;
  }

  bool isClearable() override
  {
    return false;
  }

  void updateBounds(
    double /* robot_x */,
    double /* robot_y */,
    double /* robot_yaw */,
    double * min_x,
    double * min_y,
    double * max_x,
    double * max_y) override
  {
    if (!enabled_) {
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    for (const auto & b : previous_bounds_) {
      *min_x = std::min(*min_x, b.x - b.r);
      *min_y = std::min(*min_y, b.y - b.r);
      *max_x = std::max(*max_x, b.x + b.r);
      *max_y = std::max(*max_y, b.y + b.r);
    }

    current_bounds_.clear();

    if (!latest_msg_ || !isTrackingValidNoLock()) {
      return;
    }

    for (const auto & obs : latest_msg_->obstacles) {
      const double ox = obs.pose.position.x;
      const double oy = obs.pose.position.y;

      const double hx = obs.size.x * 0.5 + obstacle_padding_;
      const double hy = obs.size.y * 0.5 + obstacle_padding_;

      const double vx = obs.twist.linear.x;
      const double vy = obs.twist.linear.y;
      const double speed = std::hypot(vx, vy);

      const double velocity_inflation =
        speed > velocity_inflation_min_ ? speed * velocity_inflation_k_ : 0.0;

      const double radius =
        std::max(hx + velocity_inflation, hy) + 0.20;

      current_bounds_.push_back({ox, oy, radius});

      *min_x = std::min(*min_x, ox - radius);
      *min_y = std::min(*min_y, oy - radius);
      *max_x = std::max(*max_x, ox + radius);
      *max_y = std::max(*max_y, oy + radius);
    }
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int /* min_i */,
    int /* min_j */,
    int /* max_i */,
    int /* max_j */) override
  {
    if (!enabled_) {
      return;
    }

    jo_msgs::msg::ObstacleArray::SharedPtr msg;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      previous_bounds_ = current_bounds_;

      if (!latest_msg_ || !isTrackingValidNoLock()) {
        publishClearMarkers();
        return;
      }

      msg = latest_msg_;
    }

    const double resolution = master_grid.getResolution();

    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_all);

    int marker_id = 0;

    for (const auto & obs : msg->obstacles) {
      const double ox = obs.pose.position.x;
      const double oy = obs.pose.position.y;
      const double oz = obs.pose.position.z;

      const double hx = obs.size.x * 0.5 + obstacle_padding_;
      const double hy = obs.size.y * 0.5 + obstacle_padding_;
      const double hz = obs.size.z * 0.5 + obstacle_padding_;

      const double vx = obs.twist.linear.x;
      const double vy = obs.twist.linear.y;
      const double speed = std::hypot(vx, vy);

      const double velocity_inflation =
        speed > velocity_inflation_min_ ? speed * velocity_inflation_k_ : 0.0;

      const double heading = std::atan2(vy, vx);
      const double cos_h = std::cos(heading);
      const double sin_h = std::sin(heading);

      const double hx_front = hx + velocity_inflation;
      const double hx_back = hx;

      const double safe_hy = std::max(hy, resolution);
      const double safe_hx_front = std::max(hx_front, resolution);
      const double safe_hx_back = std::max(hx_back, resolution);

      const double search_radius =
        std::max({safe_hx_front, safe_hx_back, safe_hy}) + resolution;

      for (double dx = -search_radius; dx <= search_radius; dx += resolution) {
        for (double dy = -search_radius; dy <= search_radius; dy += resolution) {
          const double wx = ox + dx;
          const double wy = oy + dy;

          unsigned int mx = 0;
          unsigned int my = 0;

          if (!master_grid.worldToMap(wx, wy, mx, my)) {
            continue;
          }

          const double u = dx * cos_h + dy * sin_h;
          const double v = -dx * sin_h + dy * cos_h;

          const double semi_u = u >= 0.0 ? safe_hx_front : safe_hx_back;

          const double eu = u / semi_u;
          const double ev = v / safe_hy;

          if (eu * eu + ev * ev > 1.0) {
            continue;
          }

          master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }

      visualization_msgs::msg::Marker physical_marker;
      physical_marker.header.frame_id = global_frame_;
      physical_marker.header.stamp = clock_->now();
      physical_marker.ns = "dynamic_obstacle_padded_box";
      physical_marker.id = marker_id++;
      physical_marker.type = visualization_msgs::msg::Marker::CUBE;
      physical_marker.action = visualization_msgs::msg::Marker::ADD;

      physical_marker.pose.position.x = ox;
      physical_marker.pose.position.y = oy;
      physical_marker.pose.position.z = oz;
      physical_marker.pose.orientation.w = 1.0;

      physical_marker.scale.x = 2.0 * hx;
      physical_marker.scale.y = 2.0 * hy;
      physical_marker.scale.z = 2.0 * hz;

      physical_marker.color.r = 1.0f;
      physical_marker.color.g = 1.0f;
      physical_marker.color.b = 0.0f;
      physical_marker.color.a = 0.45f;

      physical_marker.lifetime.sec = 0;
      physical_marker.lifetime.nanosec = 300000000;

      markers.markers.push_back(physical_marker);

      if (velocity_inflation > 0.0) {
        visualization_msgs::msg::Marker prediction_marker;
        prediction_marker.header.frame_id = global_frame_;
        prediction_marker.header.stamp = clock_->now();
        prediction_marker.ns = "dynamic_obstacle_velocity_inflation";
        prediction_marker.id = marker_id++;
        prediction_marker.type = visualization_msgs::msg::Marker::SPHERE;
        prediction_marker.action = visualization_msgs::msg::Marker::ADD;

        prediction_marker.pose.position.x =
          ox + 0.5 * velocity_inflation * cos_h;
        prediction_marker.pose.position.y =
          oy + 0.5 * velocity_inflation * sin_h;
        prediction_marker.pose.position.z = oz;
        prediction_marker.pose.orientation.w = 1.0;

        prediction_marker.scale.x = std::max(2.0 * hx_front, resolution);
        prediction_marker.scale.y = std::max(2.0 * hy, resolution);
        prediction_marker.scale.z = std::max(2.0 * hz, resolution);

        prediction_marker.color.r = 1.0f;
        prediction_marker.color.g = 0.3f;
        prediction_marker.color.b = 0.0f;
        prediction_marker.color.a = 0.25f;

        prediction_marker.lifetime.sec = 0;
        prediction_marker.lifetime.nanosec = 300000000;

        markers.markers.push_back(prediction_marker);
      }
    }

    if (marker_pub_) {
      marker_pub_->publish(markers);
    }
  }

private:
  struct Bound
  {
    double x;
    double y;
    double r;
  };

  bool isTrackingValidNoLock() const
  {
    if (!clock_) {
      return false;
    }

    return (clock_->now() - last_msg_time_).seconds() < tracking_timeout_;
  }

  void publishClearMarkers()
  {
    if (!marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;

    markers.markers.push_back(delete_all);
    marker_pub_->publish(markers);
  }

  rclcpp::Subscription<jo_msgs::msg::ObstacleArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Clock::SharedPtr clock_;

  std::mutex mutex_;

  jo_msgs::msg::ObstacleArray::SharedPtr latest_msg_;
  rclcpp::Time last_msg_time_{0, 0, RCL_ROS_TIME};

  std::string global_frame_{"odom"};

  bool enabled_{true};

  double obstacle_padding_{0.10};
  double tracking_timeout_{1.0};
  double velocity_inflation_k_{3.0};
  double velocity_inflation_min_{0.05};

  std::vector<Bound> previous_bounds_;
  std::vector<Bound> current_bounds_;
};

}  // namespace jo_sim

PLUGINLIB_EXPORT_CLASS(jo_sim::DynamicObstacleLayer, nav2_costmap_2d::Layer)