#pragma once

#include <vector>
#include <iostream>
#include <cmath>

class LinearTrajectoryPlayer
{
public:
  struct TrajectorySegment
  {
    double start_x, start_y;
    double end_x, end_y;
    double start_yaw, end_yaw;
  };

  LinearTrajectoryPlayer() = default;

  // Add a single trajectory segment to the sequence
  void AddSegment(double x_start, double y_start, double x_end, double y_end,
                  double yaw_start = 0.0, double yaw_end = 0.0);

  // Clear all segments
  void ClearSegments();

  // Set a single trajectory (clears existing segments)
  void SetTrajectory(double x_start, double y_start, double x_end, double y_end,
                     double yaw_start = 0.0, double yaw_end = 0.0);

  // Control methods
  void Reset();
  void Start();
  void Stop();
  void Advance(double dt);

  // Get current goal position and orientation
  void GetCurrentGoal(double &x, double &y, double &yaw) const;

  // Status queries (inline for performance)
  bool IsActive() const { return is_active_; }
  bool IsFinished() const { return is_finished_; }
  size_t GetSegmentCount() const { return segments_.size(); }
  size_t GetCurrentSegmentIndex() const { return current_segment_index_; }

  // Tunable parameters
  double max_velocity_ = 1.0;              // m/s
  double max_acceleration_ = 0.5;          // m/s^2
  double max_angular_velocity_ = 1.0;      // rad/s
  double max_angular_acceleration_ = 0.5;  // rad/s^2

private:
  // Helper function to normalize angle to [-pi, pi]
  static double NormalizeAngle(double angle);

  // Load a segment's parameters
  void LoadSegment(size_t index);

  std::vector<TrajectorySegment> segments_;
  size_t current_segment_index_ = 0;

  double start_x_ = 0.0;
  double start_y_ = 0.0;
  double end_x_ = 0.0;
  double end_y_ = 0.0;
  double start_yaw_ = 0.0;
  double end_yaw_ = 0.0;

  double total_distance_ = 0.0;
  double direction_x_ = 0.0;
  double direction_y_ = 0.0;
  double total_yaw_distance_ = 0.0;

  double current_distance_ = 0.0;
  double current_velocity_ = 0.0;
  double current_yaw_progress_ = 0.0;
  double current_angular_velocity_ = 0.0;

  bool is_active_ = false;
  bool is_finished_ = false;
};

// Global trajectory player instance
extern LinearTrajectoryPlayer trajectory_player;
