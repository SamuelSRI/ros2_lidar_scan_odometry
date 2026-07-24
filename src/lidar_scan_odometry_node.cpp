#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Keyframe
{
  Pose2D pose;
  std::vector<Point2D> points_odom;
};

class LidarScanOdometryNode : public rclcpp::Node
{
public:
  LidarScanOdometryNode()
  : Node("lidar_scan_odometry_node")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/lidar/odom");

    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    min_range_ = declare_parameter<double>("min_range", 0.10);
    max_range_ = declare_parameter<double>("max_range", 12.0);
    max_scan_points_ = declare_parameter<int>("max_scan_points", 500);

    icp_iterations_ = declare_parameter<int>("icp_iterations", 20);
    max_correspondence_distance_ =
      declare_parameter<double>("max_correspondence_distance", 0.40);
    min_correspondences_ = declare_parameter<int>("min_correspondences", 40);
    min_inlier_ratio_ = declare_parameter<double>("min_inlier_ratio", 0.25);
    max_icp_rmse_ = declare_parameter<double>("max_icp_rmse", 0.20);
    convergence_translation_ =
      declare_parameter<double>("convergence_translation", 0.001);
    convergence_rotation_ =
      declare_parameter<double>("convergence_rotation", 0.001);

    keyframe_translation_ =
      declare_parameter<double>("keyframe_translation", 0.20);
    keyframe_rotation_ =
      declare_parameter<double>("keyframe_rotation", 0.15);
    max_keyframes_ = declare_parameter<int>("max_keyframes", 15);

    submap_voxel_size_ =
      declare_parameter<double>("submap_voxel_size", 0.05);
    max_submap_points_ =
      declare_parameter<int>("max_submap_points", 2500);

    max_translation_per_scan_ =
      declare_parameter<double>("max_translation_per_scan", 0.35);
    max_rotation_per_scan_ =
      declare_parameter<double>("max_rotation_per_scan", 0.35);

    use_constant_velocity_prediction_ =
      declare_parameter<bool>("use_constant_velocity_prediction", true);

    xy_covariance_ = declare_parameter<double>("xy_covariance", 0.03);
    yaw_covariance_ = declare_parameter<double>("yaw_covariance", 0.03);
    rejected_xy_covariance_ =
      declare_parameter<double>("rejected_xy_covariance", 1.0);
    rejected_yaw_covariance_ =
      declare_parameter<double>("rejected_yaw_covariance", 1.0);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(
        &LidarScanOdometryNode::scanCallback,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "LiDAR scan-to-submap odometry started");
    RCLCPP_INFO(get_logger(), "Input:  %s", scan_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Output: %s", odom_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Local map: up to %d keyframes and %d points",
      max_keyframes_,
      max_submap_points_);
  }

private:
  std::string scan_topic_;
  std::string odom_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  double min_range_;
  double max_range_;
  int max_scan_points_;

  int icp_iterations_;
  double max_correspondence_distance_;
  int min_correspondences_;
  double min_inlier_ratio_;
  double max_icp_rmse_;
  double convergence_translation_;
  double convergence_rotation_;

  double keyframe_translation_;
  double keyframe_rotation_;
  int max_keyframes_;

  double submap_voxel_size_;
  int max_submap_points_;

  double max_translation_per_scan_;
  double max_rotation_per_scan_;
  bool use_constant_velocity_prediction_;

  double xy_covariance_;
  double yaw_covariance_;
  double rejected_xy_covariance_;
  double rejected_yaw_covariance_;

  bool initialized_{false};
  Pose2D pose_;
  Pose2D previous_pose_;
  Pose2D last_motion_;
  Pose2D last_keyframe_pose_;
  rclcpp::Time previous_stamp_;

  std::deque<Keyframe> keyframes_;
  std::vector<Point2D> local_submap_;

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
    q.z = std::sin(0.5 * yaw);
    q.w = std::cos(0.5 * yaw);
    return q;
  }

  static Point2D transformPoint(const Point2D & point, const Pose2D & pose)
  {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);

    return {
      c * point.x - s * point.y + pose.x,
      s * point.x + c * point.y + pose.y
    };
  }

  static Pose2D compose(const Pose2D & first, const Pose2D & second)
  {
    // T_result = T_first * T_second
    const double c = std::cos(first.yaw);
    const double s = std::sin(first.yaw);

    Pose2D result;
    result.x = first.x + c * second.x - s * second.y;
    result.y = first.y + s * second.x + c * second.y;
    result.yaw = normalizeAngle(first.yaw + second.yaw);
    return result;
  }

  static Pose2D inverse(const Pose2D & pose)
  {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);

    Pose2D result;
    result.x = -c * pose.x - s * pose.y;
    result.y = s * pose.x - c * pose.y;
    result.yaw = normalizeAngle(-pose.yaw);
    return result;
  }

  static Pose2D relativePose(const Pose2D & from, const Pose2D & to)
  {
    return compose(inverse(from), to);
  }

  std::vector<Point2D> scanToPoints(
    const sensor_msgs::msg::LaserScan::SharedPtr & scan) const
  {
    std::vector<Point2D> points;
    points.reserve(scan->ranges.size());

    double angle = scan->angle_min;

    for (const float range_float : scan->ranges) {
      const double range = static_cast<double>(range_float);

      if (
        std::isfinite(range) &&
        range >= min_range_ &&
        range <= max_range_)
      {
        points.push_back({
          range * std::cos(angle),
          range * std::sin(angle)
        });
      }

      angle += scan->angle_increment;
    }

    if (
      max_scan_points_ <= 0 ||
      static_cast<int>(points.size()) <= max_scan_points_)
    {
      return points;
    }

    std::vector<Point2D> downsampled;
    downsampled.reserve(static_cast<std::size_t>(max_scan_points_));

    const double step =
      static_cast<double>(points.size() - 1) /
      static_cast<double>(max_scan_points_ - 1);

    for (int i = 0; i < max_scan_points_; ++i) {
      const auto index = static_cast<std::size_t>(
        std::llround(static_cast<double>(i) * step));
      downsampled.push_back(points[index]);
    }

    return downsampled;
  }

  std::vector<Point2D> transformCloud(
    const std::vector<Point2D> & points,
    const Pose2D & pose) const
  {
    std::vector<Point2D> transformed;
    transformed.reserve(points.size());

    for (const auto & point : points) {
      transformed.push_back(transformPoint(point, pose));
    }

    return transformed;
  }

  std::vector<Point2D> voxelDownsample(
    const std::vector<Point2D> & points,
    double voxel_size,
    int maximum_points) const
  {
    if (points.empty()) {
      return {};
    }

    if (voxel_size <= 0.0) {
      return uniformDownsample(points, maximum_points);
    }

    struct IndexedPoint
    {
      long long ix;
      long long iy;
      Point2D point;
    };

    std::vector<IndexedPoint> indexed;
    indexed.reserve(points.size());

    for (const auto & point : points) {
      indexed.push_back({
        static_cast<long long>(std::floor(point.x / voxel_size)),
        static_cast<long long>(std::floor(point.y / voxel_size)),
        point
      });
    }

    std::sort(
      indexed.begin(),
      indexed.end(),
      [](const IndexedPoint & a, const IndexedPoint & b) {
        if (a.ix != b.ix) {
          return a.ix < b.ix;
        }
        return a.iy < b.iy;
      });

    std::vector<Point2D> filtered;
    filtered.reserve(indexed.size());

    std::size_t begin = 0;
    while (begin < indexed.size()) {
      std::size_t end = begin + 1;
      double sum_x = indexed[begin].point.x;
      double sum_y = indexed[begin].point.y;

      while (
        end < indexed.size() &&
        indexed[end].ix == indexed[begin].ix &&
        indexed[end].iy == indexed[begin].iy)
      {
        sum_x += indexed[end].point.x;
        sum_y += indexed[end].point.y;
        ++end;
      }

      const double count = static_cast<double>(end - begin);
      filtered.push_back({sum_x / count, sum_y / count});
      begin = end;
    }

    return uniformDownsample(filtered, maximum_points);
  }

  static std::vector<Point2D> uniformDownsample(
    const std::vector<Point2D> & points,
    int maximum_points)
  {
    if (
      maximum_points <= 0 ||
      static_cast<int>(points.size()) <= maximum_points)
    {
      return points;
    }

    std::vector<Point2D> result;
    result.reserve(static_cast<std::size_t>(maximum_points));

    const double step =
      static_cast<double>(points.size() - 1) /
      static_cast<double>(maximum_points - 1);

    for (int i = 0; i < maximum_points; ++i) {
      const auto index = static_cast<std::size_t>(
        std::llround(static_cast<double>(i) * step));
      result.push_back(points[index]);
    }

    return result;
  }

  bool findCorrespondences(
    const std::vector<Point2D> & source,
    const std::vector<Point2D> & target,
    std::vector<Point2D> & matched_source,
    std::vector<Point2D> & matched_target,
    double & rmse,
    double & inlier_ratio) const
  {
    struct Match
    {
      Point2D source;
      Point2D target;
      double squared_distance;
    };

    std::vector<Match> matches;
    matches.reserve(source.size());

    const double max_distance_squared =
      max_correspondence_distance_ * max_correspondence_distance_;

    for (const auto & source_point : source) {
      double best_squared_distance = std::numeric_limits<double>::max();
      std::size_t best_index = 0;

      for (std::size_t i = 0; i < target.size(); ++i) {
        const double dx = target[i].x - source_point.x;
        const double dy = target[i].y - source_point.y;
        const double squared_distance = dx * dx + dy * dy;

        if (squared_distance < best_squared_distance) {
          best_squared_distance = squared_distance;
          best_index = i;
        }
      }

      if (best_squared_distance <= max_distance_squared) {
        matches.push_back({
          source_point,
          target[best_index],
          best_squared_distance
        });
      }
    }

    inlier_ratio =
      source.empty() ?
      0.0 :
      static_cast<double>(matches.size()) /
      static_cast<double>(source.size());

    if (
      static_cast<int>(matches.size()) < min_correspondences_ ||
      inlier_ratio < min_inlier_ratio_)
    {
      return false;
    }

    // Enlève les 20 % de correspondances les plus mauvaises.
    std::sort(
      matches.begin(),
      matches.end(),
      [](const Match & a, const Match & b) {
        return a.squared_distance < b.squared_distance;
      });

    const std::size_t kept_count = std::max<std::size_t>(
      static_cast<std::size_t>(min_correspondences_),
      static_cast<std::size_t>(
        std::floor(0.80 * static_cast<double>(matches.size()))));

    matched_source.clear();
    matched_target.clear();
    matched_source.reserve(kept_count);
    matched_target.reserve(kept_count);

    double squared_error_sum = 0.0;

    for (std::size_t i = 0; i < kept_count; ++i) {
      matched_source.push_back(matches[i].source);
      matched_target.push_back(matches[i].target);
      squared_error_sum += matches[i].squared_distance;
    }

    rmse = std::sqrt(
      squared_error_sum / static_cast<double>(kept_count));

    return true;
  }

  static bool computeBestFitTransform(
    const std::vector<Point2D> & source,
    const std::vector<Point2D> & target,
    Pose2D & transform)
  {
    if (
      source.size() != target.size() ||
      source.size() < 2)
    {
      return false;
    }

    double source_cx = 0.0;
    double source_cy = 0.0;
    double target_cx = 0.0;
    double target_cy = 0.0;

    for (std::size_t i = 0; i < source.size(); ++i) {
      source_cx += source[i].x;
      source_cy += source[i].y;
      target_cx += target[i].x;
      target_cy += target[i].y;
    }

    const double count = static_cast<double>(source.size());
    source_cx /= count;
    source_cy /= count;
    target_cx /= count;
    target_cy /= count;

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

    transform.yaw = std::atan2(sxy, sxx);

    const double c = std::cos(transform.yaw);
    const double s = std::sin(transform.yaw);

    transform.x =
      target_cx - (c * source_cx - s * source_cy);
    transform.y =
      target_cy - (s * source_cx + c * source_cy);

    return
      std::isfinite(transform.x) &&
      std::isfinite(transform.y) &&
      std::isfinite(transform.yaw);
  }

  bool alignScanToSubmap(
    const std::vector<Point2D> & scan_points,
    const Pose2D & predicted_pose,
    Pose2D & estimated_pose,
    double & final_rmse,
    double & final_inlier_ratio) const
  {
    if (local_submap_.size() < static_cast<std::size_t>(min_correspondences_)) {
      return false;
    }

    // Le scan courant est d'abord placé dans odom avec une prédiction.
    std::vector<Point2D> transformed_scan =
      transformCloud(scan_points, predicted_pose);

    Pose2D total_correction;

    final_rmse = std::numeric_limits<double>::infinity();
    final_inlier_ratio = 0.0;

    for (int iteration = 0; iteration < icp_iterations_; ++iteration) {
      std::vector<Point2D> matched_source;
      std::vector<Point2D> matched_target;
      double rmse = 0.0;
      double inlier_ratio = 0.0;

      if (!findCorrespondences(
          transformed_scan,
          local_submap_,
          matched_source,
          matched_target,
          rmse,
          inlier_ratio))
      {
        return false;
      }

      Pose2D correction;
      if (!computeBestFitTransform(
          matched_source,
          matched_target,
          correction))
      {
        return false;
      }

      for (auto & point : transformed_scan) {
        point = transformPoint(point, correction);
      }

      total_correction = compose(correction, total_correction);
      final_rmse = rmse;
      final_inlier_ratio = inlier_ratio;

      const double correction_translation =
        std::hypot(correction.x, correction.y);

      if (
        correction_translation < convergence_translation_ &&
        std::abs(correction.yaw) < convergence_rotation_)
      {
        break;
      }
    }

    estimated_pose = compose(total_correction, predicted_pose);

    return
      std::isfinite(final_rmse) &&
      final_rmse <= max_icp_rmse_ &&
      final_inlier_ratio >= min_inlier_ratio_;
  }

  Pose2D predictPose() const
  {
    if (!use_constant_velocity_prediction_) {
      return pose_;
    }

    return compose(pose_, last_motion_);
  }

  bool motionIsPlausible(
    const Pose2D & old_pose,
    const Pose2D & new_pose,
    Pose2D & relative_motion) const
  {
    relative_motion = relativePose(old_pose, new_pose);

    const double translation =
      std::hypot(relative_motion.x, relative_motion.y);

    return
      translation <= max_translation_per_scan_ &&
      std::abs(relative_motion.yaw) <= max_rotation_per_scan_;
  }

  bool shouldCreateKeyframe(const Pose2D & current_pose) const
  {
    const Pose2D delta =
      relativePose(last_keyframe_pose_, current_pose);

    return
      std::hypot(delta.x, delta.y) >= keyframe_translation_ ||
      std::abs(delta.yaw) >= keyframe_rotation_;
  }

  void addKeyframe(
    const std::vector<Point2D> & scan_points,
    const Pose2D & keyframe_pose)
  {
    Keyframe keyframe;
    keyframe.pose = keyframe_pose;
    keyframe.points_odom = transformCloud(scan_points, keyframe_pose);

    keyframes_.push_back(std::move(keyframe));

    while (
      max_keyframes_ > 0 &&
      static_cast<int>(keyframes_.size()) > max_keyframes_)
    {
      keyframes_.pop_front();
    }

    last_keyframe_pose_ = keyframe_pose;
    rebuildSubmap();
  }

  void rebuildSubmap()
  {
    std::vector<Point2D> all_points;

    std::size_t total_size = 0;
    for (const auto & keyframe : keyframes_) {
      total_size += keyframe.points_odom.size();
    }

    all_points.reserve(total_size);

    for (const auto & keyframe : keyframes_) {
      all_points.insert(
        all_points.end(),
        keyframe.points_odom.begin(),
        keyframe.points_odom.end());
    }

    local_submap_ = voxelDownsample(
      all_points,
      submap_voxel_size_,
      max_submap_points_);

    RCLCPP_DEBUG(
      get_logger(),
      "Submap rebuilt: %zu keyframes, %zu points",
      keyframes_.size(),
      local_submap_.size());
  }

  void scanCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const std::vector<Point2D> current_points = scanToPoints(scan);

    if (
      current_points.size() <
      static_cast<std::size_t>(min_correspondences_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Not enough valid LiDAR points: %zu",
        current_points.size());
      return;
    }

    const rclcpp::Time current_stamp(scan->header.stamp);

    if (!initialized_) {
      pose_ = Pose2D{};
      previous_pose_ = pose_;
      last_motion_ = Pose2D{};
      last_keyframe_pose_ = pose_;
      previous_stamp_ = current_stamp;

      addKeyframe(current_points, pose_);
      initialized_ = true;

      publishOdometry(
        scan->header.stamp,
        0.0,
        0.0,
        0.0,
        xy_covariance_,
        yaw_covariance_);
      return;
    }

    const Pose2D predicted_pose = predictPose();

    Pose2D estimated_pose;
    double rmse = 0.0;
    double inlier_ratio = 0.0;

    const bool icp_ok = alignScanToSubmap(
      current_points,
      predicted_pose,
      estimated_pose,
      rmse,
      inlier_ratio);

    Pose2D relative_motion;

    if (
      !icp_ok ||
      !motionIsPlausible(pose_, estimated_pose, relative_motion))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "ICP rejected: rmse=%.3f m, inliers=%.1f%%",
        rmse,
        100.0 * inlier_ratio);

      // On conserve la pose et on augmente la covariance.
      publishOdometry(
        scan->header.stamp,
        0.0,
        0.0,
        0.0,
        rejected_xy_covariance_,
        rejected_yaw_covariance_);
      return;
    }

    previous_pose_ = pose_;
    pose_ = estimated_pose;
    last_motion_ = relative_motion;

    double dt = (current_stamp - previous_stamp_).seconds();
    if (dt <= 0.0 || dt > 1.0) {
      dt = 1e-3;
    }

    const double vx = relative_motion.x / dt;
    const double vy = relative_motion.y / dt;
    const double wz = relative_motion.yaw / dt;

    publishOdometry(
      scan->header.stamp,
      vx,
      vy,
      wz,
      xy_covariance_,
      yaw_covariance_);

    previous_stamp_ = current_stamp;

    if (shouldCreateKeyframe(pose_)) {
      addKeyframe(current_points, pose_);
    }

    RCLCPP_DEBUG(
      get_logger(),
      "pose=(%.3f, %.3f, %.3f), rmse=%.3f, inliers=%.1f%%",
      pose_.x,
      pose_.y,
      pose_.yaw,
      rmse,
      100.0 * inlier_ratio);
  }

  void publishOdometry(
    const builtin_interfaces::msg::Time & stamp,
    double vx,
    double vy,
    double wz,
    double xy_covariance,
    double yaw_covariance)
  {
    nav_msgs::msg::Odometry message;

    message.header.stamp = stamp;
    message.header.frame_id = odom_frame_;
    message.child_frame_id = base_frame_;

    message.pose.pose.position.x = pose_.x;
    message.pose.pose.position.y = pose_.y;
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation = yawToQuaternion(pose_.yaw);

    message.twist.twist.linear.x = vx;
    message.twist.twist.linear.y = vy;
    message.twist.twist.linear.z = 0.0;

    message.twist.twist.angular.x = 0.0;
    message.twist.twist.angular.y = 0.0;
    message.twist.twist.angular.z = wz;

    std::fill(
      message.pose.covariance.begin(),
      message.pose.covariance.end(),
      0.0);
    std::fill(
      message.twist.covariance.begin(),
      message.twist.covariance.end(),
      0.0);

    message.pose.covariance[0] = xy_covariance;
    message.pose.covariance[7] = xy_covariance;
    message.pose.covariance[35] = yaw_covariance;

    message.pose.covariance[14] = 999.0;
    message.pose.covariance[21] = 999.0;
    message.pose.covariance[28] = 999.0;

    message.twist.covariance[0] = xy_covariance;
    message.twist.covariance[7] = xy_covariance;
    message.twist.covariance[35] = yaw_covariance;

    message.twist.covariance[14] = 999.0;
    message.twist.covariance[21] = 999.0;
    message.twist.covariance[28] = 999.0;

    odom_pub_->publish(message);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarScanOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
