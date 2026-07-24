#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

struct Point2D
{
  double x;
  double y;
};

class LidarScanOdometryNode : public rclcpp::Node
{
public:
  LidarScanOdometryNode()
  : Node("lidar_scan_odometry_node")
  {
    scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/lidar/odom");

    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

    min_range_ = this->declare_parameter<double>("min_range", 0.10);
    max_range_ = this->declare_parameter<double>("max_range", 12.0);
    max_points_ = this->declare_parameter<int>("max_points", 500);

    icp_iterations_ = this->declare_parameter<int>("icp_iterations", 10);
    max_correspondence_distance_ =
      this->declare_parameter<double>("max_correspondence_distance", 0.35);

    max_translation_per_scan_ =
      this->declare_parameter<double>("max_translation_per_scan", 0.50);
    max_rotation_per_scan_ =
      this->declare_parameter<double>("max_rotation_per_scan", 0.50);

    xy_covariance_ = this->declare_parameter<double>("xy_covariance", 0.05);
    yaw_covariance_ = this->declare_parameter<double>("yaw_covariance", 0.05);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      10,
      std::bind(&LidarScanOdometryNode::scanCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "LiDAR scan odometry node started");
    RCLCPP_INFO(this->get_logger(), "Input scan topic: %s", scan_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output odom topic: %s", odom_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "This node publishes Odometry only, no TF");
  }

private:
  std::string scan_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  double min_range_;
  double max_range_;
  int max_points_;

  int icp_iterations_;
  double max_correspondence_distance_;

  double max_translation_per_scan_;
  double max_rotation_per_scan_;

  double xy_covariance_;
  double yaw_covariance_;

  bool has_previous_scan_ = false;

  std::vector<Point2D> previous_points_;
  rclcpp::Time previous_stamp_;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
  {
    geometry_msgs::msg::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
    return q;
  }

  std::vector<Point2D> scanToPoints(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    std::vector<Point2D> points;
    points.reserve(scan->ranges.size());

    double angle = scan->angle_min;

    for (const auto & r_float : scan->ranges) {
      const double r = static_cast<double>(r_float);

      if (std::isfinite(r) && r >= min_range_ && r <= max_range_) {
        Point2D p;
        p.x = r * std::cos(angle);
        p.y = r * std::sin(angle);
        points.push_back(p);
      }

      angle += scan->angle_increment;
    }

    if (static_cast<int>(points.size()) <= max_points_) {
      return points;
    }

    std::vector<Point2D> downsampled;
    downsampled.reserve(max_points_);

    const double step = static_cast<double>(points.size() - 1) /
                        static_cast<double>(max_points_ - 1);

    for (int i = 0; i < max_points_; ++i) {
      const int index = static_cast<int>(std::round(i * step));
      downsampled.push_back(points[index]);
    }

    return downsampled;
  }

  bool findCorrespondences(
    const std::vector<Point2D> & source,
    const std::vector<Point2D> & target,
    std::vector<Point2D> & matched_source,
    std::vector<Point2D> & matched_target)
  {
    matched_source.clear();
    matched_target.clear();

    const double max_dist_sq =
      max_correspondence_distance_ * max_correspondence_distance_;

    for (const auto & p : source) {
      double best_dist_sq = std::numeric_limits<double>::max();
      std::size_t best_index = 0;

      for (std::size_t i = 0; i < target.size(); ++i) {
        const double dx = target[i].x - p.x;
        const double dy = target[i].y - p.y;
        const double dist_sq = dx * dx + dy * dy;

        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best_index = i;
        }
      }

      if (best_dist_sq <= max_dist_sq) {
        matched_source.push_back(p);
        matched_target.push_back(target[best_index]);
      }
    }

    return matched_source.size() >= 10;
  }

  bool computeBestFitTransform(
    const std::vector<Point2D> & source,
    const std::vector<Point2D> & target,
    double & delta_x,
    double & delta_y,
    double & delta_yaw)
  {
    if (source.size() != target.size() || source.size() < 2) {
      return false;
    }

    double source_cx = 0.0;
    double source_cy = 0.0;
    double target_cx = 0.0;
    double target_cy = 0.0;

    const double n = static_cast<double>(source.size());

    for (std::size_t i = 0; i < source.size(); ++i) {
      source_cx += source[i].x;
      source_cy += source[i].y;
      target_cx += target[i].x;
      target_cy += target[i].y;
    }

    source_cx /= n;
    source_cy /= n;
    target_cx /= n;
    target_cy /= n;

    double sxx = 0.0;
    double sxy = 0.0;

    for (std::size_t i = 0; i < source.size(); ++i) {
      const double sx = source[i].x - source_cx;
      const double sy = source[i].y - source_cy;
      const double tx = target[i].x - target_cx;
      const double ty = target[i].y - target_cy;

      sxx += sx * tx + sy * ty;
      sxy += sx * ty - sy * tx;
    }

    delta_yaw = std::atan2(sxy, sxx);

    const double c = std::cos(delta_yaw);
    const double s = std::sin(delta_yaw);

    delta_x = target_cx - (c * source_cx - s * source_cy);
    delta_y = target_cy - (s * source_cx + c * source_cy);

    return true;
  }

  void applyTransform(
    std::vector<Point2D> & points,
    double dx,
    double dy,
    double dyaw)
  {
    const double c = std::cos(dyaw);
    const double s = std::sin(dyaw);

    for (auto & p : points) {
      const double x_new = c * p.x - s * p.y + dx;
      const double y_new = s * p.x + c * p.y + dy;

      p.x = x_new;
      p.y = y_new;
    }
  }

  bool icp2D(
    const std::vector<Point2D> & current,
    const std::vector<Point2D> & previous,
    double & dx,
    double & dy,
    double & dyaw)
  {
    std::vector<Point2D> transformed = current;

    double total_x = 0.0;
    double total_y = 0.0;
    double total_yaw = 0.0;

    for (int i = 0; i < icp_iterations_; ++i) {
      std::vector<Point2D> matched_source;
      std::vector<Point2D> matched_target;

      const bool correspondences_ok = findCorrespondences(
        transformed,
        previous,
        matched_source,
        matched_target
      );

      if (!correspondences_ok) {
        return false;
      }

      double step_x = 0.0;
      double step_y = 0.0;
      double step_yaw = 0.0;

      const bool transform_ok = computeBestFitTransform(
        matched_source,
        matched_target,
        step_x,
        step_y,
        step_yaw
      );

      if (!transform_ok) {
        return false;
      }

      applyTransform(transformed, step_x, step_y, step_yaw);

      const double c = std::cos(step_yaw);
      const double s = std::sin(step_yaw);

      const double new_total_x = c * total_x - s * total_y + step_x;
      const double new_total_y = s * total_x + c * total_y + step_y;

      total_x = new_total_x;
      total_y = new_total_y;
      total_yaw = normalizeAngle(total_yaw + step_yaw);
    }

    // Le transform calculé aligne current -> previous.
    // Le mouvement du robot est previous -> current, donc on inverse.
    dx = total_x;
    dy = total_y;
    dyaw = total_yaw;

    return true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const auto current_points = scanToPoints(scan);

    if (current_points.size() < 20) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Not enough valid LiDAR points, skipping scan"
      );
      return;
    }

    const rclcpp::Time current_stamp(scan->header.stamp);

    if (!has_previous_scan_) {
      previous_points_ = current_points;
      previous_stamp_ = current_stamp;
      has_previous_scan_ = true;

      publishOdometry(scan->header.stamp, 0.0, 0.0, 0.0);
      return;
    }

    double dx_local = 0.0;
    double dy_local = 0.0;
    double dyaw = 0.0;

    const bool icp_ok = icp2D(
      current_points,
      previous_points_,
      dx_local,
      dy_local,
      dyaw
    );

    if (!icp_ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "ICP failed, keeping previous pose"
      );

      publishOdometry(scan->header.stamp, 0.0, 0.0, 0.0);
      return;
    }

    const double translation = std::sqrt(dx_local * dx_local + dy_local * dy_local);

    if (translation > max_translation_per_scan_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected LiDAR odom jump: translation = %.3f m",
        translation
      );

      previous_points_ = current_points;
      previous_stamp_ = current_stamp;
      return;
    }

    if (std::abs(dyaw) > max_rotation_per_scan_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Rejected LiDAR odom jump: rotation = %.3f rad",
        dyaw
      );

      previous_points_ = current_points;
      previous_stamp_ = current_stamp;
      return;
    }

    const double c = std::cos(yaw_);
    const double s = std::sin(yaw_);

    const double dx_global = c * dx_local - s * dy_local;
    const double dy_global = s * dx_local + c * dy_local;

    x_ += dx_global;
    y_ += dy_global;
    yaw_ = normalizeAngle(yaw_ + dyaw);

    double dt = (current_stamp - previous_stamp_).seconds();
    if (dt <= 0.0) {
      dt = 1e-6;
    }

    const double vx = dx_local / dt;
    const double vy = dy_local / dt;
    const double wz = dyaw / dt;

    publishOdometry(scan->header.stamp, vx, vy, wz);

    previous_points_ = current_points;
    previous_stamp_ = current_stamp;
  }

  void publishOdometry(
    const builtin_interfaces::msg::Time & stamp,
    double vx,
    double vy,
    double wz)
  {
    nav_msgs::msg::Odometry msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = base_frame_;

    msg.pose.pose.position.x = x_;
    msg.pose.pose.position.y = y_;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = yawToQuaternion(yaw_);

    msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.linear.z = 0.0;

    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = wz;

    for (auto & value : msg.pose.covariance) {
      value = 0.0;
    }

    for (auto & value : msg.twist.covariance) {
      value = 0.0;
    }

    msg.pose.covariance[0] = xy_covariance_;
    msg.pose.covariance[7] = xy_covariance_;
    msg.pose.covariance[35] = yaw_covariance_;

    msg.pose.covariance[14] = 999.0;
    msg.pose.covariance[21] = 999.0;
    msg.pose.covariance[28] = 999.0;

    msg.twist.covariance[0] = xy_covariance_;
    msg.twist.covariance[7] = xy_covariance_;
    msg.twist.covariance[35] = yaw_covariance_;

    msg.twist.covariance[14] = 999.0;
    msg.twist.covariance[21] = 999.0;
    msg.twist.covariance[28] = 999.0;

    odom_pub_->publish(msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LidarScanOdometryNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
