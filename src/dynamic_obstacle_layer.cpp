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

// ─────────────────────────────────────────────────────────────────────────────

class DynamicObstacleLayer : public nav2_costmap_2d::Layer
{
public:
  DynamicObstacleLayer() = default;

  // ── lifecycle ──────────────────────────────────────────────────────────────

  void onInitialize() override
  {
    declareParameter("tracking_timeout",       rclcpp::ParameterValue(1.0));
    declareParameter("velocity_inflation_k",   rclcpp::ParameterValue(0.5));
    declareParameter("velocity_inflation_min", rclcpp::ParameterValue(0.05));
    auto node = node_.lock();
    if (node) {
      node->get_parameter(name_ + ".tracking_timeout",       tracking_timeout_);
      node->get_parameter(name_ + ".velocity_inflation_k",   vel_inflation_k_);
      node->get_parameter(name_ + ".velocity_inflation_min", vel_inflation_min_);
      clock_ = node->get_clock();
      sub_ = node->create_subscription<jo_msgs::msg::ObstacleArray>(
        "/onboard_detector/tracked_dynamic_obstacles", 10,
        [this](const jo_msgs::msg::ObstacleArray::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(mtx_);
          latest_    = msg;
          last_recv_ = clock_->now();
        });
      marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/dynamic_obstacle_footprints", rclcpp::QoS(1));
    }
    global_frame_ = layered_costmap_->getGlobalFrameID();
    current_ = true;
    enabled_ = true;
  }

  void reset() override {}

  // ── bounds ─────────────────────────────────────────────────────────────────
  //
  // LayeredCostmap::updateMap() calls resetMap() on the union of all layers'
  // bounds BEFORE updateCosts().  Including both prev and curr frame footprints
  // guarantees old LETHAL marks are always inside the reset region → no trail.

  void updateBounds(
    double /*rx*/, double /*ry*/, double /*ryaw*/,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override
  {
    if (!enabled_) {return;}

    std::lock_guard<std::mutex> lk(mtx_);

    for (const auto & b : prev_bounds_) {
      *min_x = std::min(*min_x, b.x - b.r);
      *min_y = std::min(*min_y, b.y - b.r);
      *max_x = std::max(*max_x, b.x + b.r);
      *max_y = std::max(*max_y, b.y + b.r);
    }

    curr_bounds_.clear();
    if (!latest_ || !is_tracking()) {return;}

    for (const auto & obs : latest_->obstacles) {
      const double speed = std::hypot(obs.twist.linear.x, obs.twist.linear.y);
      const double infl  = speed * vel_inflation_k_;
      // Bounding radius = farthest point of the asymmetric ellipse from its centre.
      const double r = std::max(obs.size.x * 0.5 + infl, obs.size.y * 0.5) + 0.1;
      const double ox = obs.pose.position.x;
      const double oy = obs.pose.position.y;
      curr_bounds_.push_back({ox, oy, r});
      *min_x = std::min(*min_x, ox - r);
      *min_y = std::min(*min_y, oy - r);
      *max_x = std::max(*max_x, ox + r);
      *max_y = std::max(*max_y, oy + r);
    }
  }

  // ── costs ──────────────────────────────────────────────────────────────────

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int /*min_i*/, int /*min_j*/,
    int /*max_i*/, int /*max_j*/) override
  {
    if (!enabled_) {return;}

    {
      std::lock_guard<std::mutex> lk(mtx_);
      prev_bounds_ = curr_bounds_;
    }

    jo_msgs::msg::ObstacleArray::SharedPtr msg;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!latest_ || !is_tracking()) {
        publish_clear_markers();
        return;
      }
      msg = latest_;
    }

    const double res = master_grid.getResolution();
    visualization_msgs::msg::MarkerArray marker_array;

    // DELETEALL removes markers from the previous frame.
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(del);

    int marker_id = 0;
    for (const auto & obs : msg->obstacles) {
      const double ox = obs.pose.position.x;
      const double oy = obs.pose.position.y;

      // Half-extents of the AABB published by the obstacle detector.
      const double hx = obs.size.x * 0.5;
      const double hy = obs.size.y * 0.5;

      // ── Velocity-inflated egg footprint for costmap marking ───────────────
      // Asymmetric ellipse centred on the obstacle:
      //   • rear  semi-axis (against velocity): hx  (physical size)
      //   • front semi-axis (along  velocity):  hx + k*speed  (inflated)
      //   • lateral semi-axis: hy  (unchanged)
      // No centre shift — the asymmetry itself creates the egg shape.
      const double vx    = obs.twist.linear.x;
      const double vy    = obs.twist.linear.y;
      const double speed = std::hypot(vx, vy);
      const double infl  = speed * vel_inflation_k_;

      const double mark_heading = std::atan2(vy, vx);

      const double cos_m  = std::cos(mark_heading);
      const double sin_m  = std::sin(mark_heading);
      const double hx_fwd = hx + infl;   // front semi-axis
      // search radius covers the farthest possible point
      const double bound  = std::max(hx_fwd, hy) + res;

      for (double dx = -bound; dx <= bound; dx += res) {
        for (double dy = -bound; dy <= bound; dy += res) {
          // velocity-aligned frame: u = along vel, v = perpendicular
          const double u =  dx * cos_m + dy * sin_m;
          const double v = -dx * sin_m + dy * cos_m;

          // Asymmetric ellipse test
          const double semi_u = (u >= 0.0) ? hx_fwd : hx;
          const double eu = u / semi_u;
          const double ev = v / hy;
          if (eu * eu + ev * ev > 1.0) {continue;}

          unsigned int mx, my;
          if (master_grid.worldToMap(ox + dx, oy + dy, mx, my)) {
            master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
          }
        }
      }

      // ── Visualization: yellow box = physical obstacle size (no inflation) ──
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = global_frame_;
      mk.header.stamp    = clock_->now();
      mk.ns              = "dynamic_obstacle_footprint";
      mk.id              = marker_id++;
      mk.type            = visualization_msgs::msg::Marker::CUBE;
      mk.action          = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x = ox;
      mk.pose.position.y = oy;
      mk.pose.position.z = obs.pose.position.z;
      mk.pose.orientation.w = 1.0;  // AABB — axis-aligned, no rotation
      mk.scale.x = obs.size.x;
      mk.scale.y = obs.size.y;
      mk.scale.z = obs.size.z;
      mk.color.r = 1.0f;
      mk.color.g = 1.0f;
      mk.color.b = 0.0f;
      mk.color.a = 0.5f;
      mk.lifetime.sec     = 0;
      mk.lifetime.nanosec = 300'000'000;
      marker_array.markers.push_back(mk);
    }

    if (marker_pub_) {
      marker_pub_->publish(marker_array);
    }
  }

  bool isClearable() override {return false;}

private:
  bool is_tracking()
  {
    if (!latest_ || !clock_) {return false;}
    return (clock_->now() - last_recv_).seconds() < tracking_timeout_;
  }

  void publish_clear_markers()
  {
    if (!marker_pub_) {return;}
    visualization_msgs::msg::MarkerArray empty;
    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    empty.markers.push_back(del);
    marker_pub_->publish(empty);
  }

  rclcpp::Subscription<jo_msgs::msg::ObstacleArray>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Clock::SharedPtr clock_;
  std::mutex mtx_;
  jo_msgs::msg::ObstacleArray::SharedPtr latest_;
  rclcpp::Time last_recv_{0, 0, RCL_ROS_TIME};
  std::string global_frame_{"odom"};

  double tracking_timeout_{1.0};
  double vel_inflation_k_{0.5};    // lookahead [s]: costmap extends by k*speed metres
  double vel_inflation_min_{0.05}; // speed threshold [m/s] below which no inflation

  struct Bound {double x, y, r;};
  std::vector<Bound> prev_bounds_;
  std::vector<Bound> curr_bounds_;
};

}  // namespace jo_sim

PLUGINLIB_EXPORT_CLASS(jo_sim::DynamicObstacleLayer, nav2_costmap_2d::Layer)
