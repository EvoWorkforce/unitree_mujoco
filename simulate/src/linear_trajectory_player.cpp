#include "linear_trajectory_player.h"

// Global trajectory player instance definition
LinearTrajectoryPlayer trajectory_player;

void LinearTrajectoryPlayer::AddSegment(double x_start, double y_start, double x_end, double y_end,
                                         double yaw_start, double yaw_end)
{
  segments_.push_back({x_start, y_start, x_end, y_end, yaw_start, yaw_end});
}

void LinearTrajectoryPlayer::ClearSegments()
{
  segments_.clear();
  current_segment_index_ = 0;
}

void LinearTrajectoryPlayer::SetTrajectory(double x_start, double y_start, double x_end, double y_end,
                                            double yaw_start, double yaw_end)
{
  ClearSegments();
  AddSegment(x_start, y_start, x_end, y_end, yaw_start, yaw_end);
}

void LinearTrajectoryPlayer::Reset()
{
  current_distance_ = 0.0;
  current_velocity_ = 0.0;
  current_yaw_progress_ = 0.0;
  current_angular_velocity_ = 0.0;
  current_segment_index_ = 0;
  is_active_ = false;
  is_finished_ = false;
  
  if (!segments_.empty())
  {
    LoadSegment(0);
  }
}

void LinearTrajectoryPlayer::Start()
{
  if (segments_.empty())
  {
    std::cout << "No trajectory segments defined!" << std::endl;
    return;
  }
  
  current_segment_index_ = 0;
  current_distance_ = 0.0;
  current_velocity_ = 0.0;
  current_yaw_progress_ = 0.0;
  current_angular_velocity_ = 0.0;
  is_active_ = true;
  is_finished_ = false;
  LoadSegment(0);
  
  std::cout << "Starting trajectory sequence with " << segments_.size() << " segment(s)" << std::endl;
}

void LinearTrajectoryPlayer::Stop()
{
  is_active_ = false;
  std::cout << "Trajectory sequence stopped" << std::endl;
}

void LinearTrajectoryPlayer::Advance(double dt)
{
  if (!is_active_ || is_finished_ || segments_.empty())
    return;

  // === Linear motion ===
  // Calculate remaining distance
  double remaining_distance = total_distance_ - current_distance_;

  // Calculate stopping distance at current velocity
  double stopping_distance = (current_velocity_ * current_velocity_) / (2.0 * max_acceleration_);

  // Decide whether to accelerate, maintain, or decelerate
  if (remaining_distance <= stopping_distance)
  {
    // Decelerate
    current_velocity_ -= max_acceleration_ * dt;
    if (current_velocity_ < 0.0)
      current_velocity_ = 0.0;
  }
  else if (current_velocity_ < max_velocity_)
  {
    // Accelerate
    current_velocity_ += max_acceleration_ * dt;
    if (current_velocity_ > max_velocity_)
      current_velocity_ = max_velocity_;
  }
  // else maintain current velocity (at max)

  // Update position
  current_distance_ += current_velocity_ * dt;

  // === Angular motion ===
  // Calculate remaining angular distance
  double remaining_yaw = total_yaw_distance_ - current_yaw_progress_;

  // Calculate stopping angular distance at current angular velocity
  double stopping_yaw = (current_angular_velocity_ * current_angular_velocity_) / (2.0 * max_angular_acceleration_);

  // Decide whether to accelerate, maintain, or decelerate angular velocity
  if (std::abs(remaining_yaw) <= stopping_yaw)
  {
    // Decelerate
    if (current_angular_velocity_ > 0.0)
    {
      current_angular_velocity_ -= max_angular_acceleration_ * dt;
      if (current_angular_velocity_ < 0.0)
        current_angular_velocity_ = 0.0;
    }
    else if (current_angular_velocity_ < 0.0)
    {
      current_angular_velocity_ += max_angular_acceleration_ * dt;
      if (current_angular_velocity_ > 0.0)
        current_angular_velocity_ = 0.0;
    }
  }
  else if (std::abs(current_angular_velocity_) < max_angular_velocity_)
  {
    // Accelerate in the direction of remaining yaw
    double accel_sign = (remaining_yaw > 0.0) ? 1.0 : -1.0;
    current_angular_velocity_ += accel_sign * max_angular_acceleration_ * dt;
    if (std::abs(current_angular_velocity_) > max_angular_velocity_)
      current_angular_velocity_ = accel_sign * max_angular_velocity_;
  }

  // Update yaw progress
  current_yaw_progress_ += current_angular_velocity_ * dt;

  // Clamp yaw progress
  if (total_yaw_distance_ > 0.0 && current_yaw_progress_ > total_yaw_distance_)
    current_yaw_progress_ = total_yaw_distance_;
  else if (total_yaw_distance_ < 0.0 && current_yaw_progress_ < total_yaw_distance_)
    current_yaw_progress_ = total_yaw_distance_;

  // Check if finished with current segment
  if (current_distance_ >= total_distance_)
  {
    current_distance_ = total_distance_;
    current_velocity_ = 0.0;
    current_yaw_progress_ = total_yaw_distance_;
    current_angular_velocity_ = 0.0;
    
    // Check if there are more segments
    if (current_segment_index_ + 1 < segments_.size())
    {
      // Move to next segment
      current_segment_index_++;
      current_distance_ = 0.0;
      current_yaw_progress_ = 0.0;
      // Don't reset velocities - maintain smooth transition
      LoadSegment(current_segment_index_);
      std::cout << "Advancing to segment " << (current_segment_index_ + 1) << "/" << segments_.size() << std::endl;
    }
    else
    {
      // All segments complete
      is_finished_ = true;
      is_active_ = false;
      std::cout << "Trajectory sequence playback finished (all " << segments_.size() << " segments complete)" << std::endl;
    }
  }
}

void LinearTrajectoryPlayer::GetCurrentGoal(double &x, double &y, double &yaw) const
{
  if (total_distance_ < 1e-6)
  {
    x = start_x_;
    y = start_y_;
    yaw = start_yaw_;
    return;
  }

  x = start_x_ + direction_x_ * current_distance_;
  y = start_y_ + direction_y_ * current_distance_;
  
  // Interpolate yaw
  yaw = start_yaw_ + current_yaw_progress_;
}

double LinearTrajectoryPlayer::NormalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

void LinearTrajectoryPlayer::LoadSegment(size_t index)
{
  if (index >= segments_.size())
    return;

  const auto &seg = segments_[index];
  start_x_ = seg.start_x;
  start_y_ = seg.start_y;
  end_x_ = seg.end_x;
  end_y_ = seg.end_y;
  start_yaw_ = seg.start_yaw;
  end_yaw_ = seg.end_yaw;

  // Calculate total distance
  double dx = end_x_ - start_x_;
  double dy = end_y_ - start_y_;
  total_distance_ = sqrt(dx * dx + dy * dy);

  // Direction unit vector
  if (total_distance_ > 1e-6)
  {
    direction_x_ = dx / total_distance_;
    direction_y_ = dy / total_distance_;
  }
  else
  {
    direction_x_ = 0.0;
    direction_y_ = 0.0;
  }

  // Calculate total yaw rotation (shortest path)
  total_yaw_distance_ = NormalizeAngle(end_yaw_ - start_yaw_);
}
