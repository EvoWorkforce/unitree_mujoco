#pragma once

#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>

class TrajectoryScorer
{
public:
  TrajectoryScorer() = default;

  // Reset all statistics
  void Reset();

  // Update with new error measurement (position error norm and orientation error)
  void UpdatePositionError(double x_error, double y_error);
  void UpdateOrientationError(double yaw_error);

  // Get statistics
  double GetAveragePositionError() const;
  double GetAverageOrientationError() const;
  double GetCompositeScore(double orientation_weight = 0.5) const;
  double GetTotalPositionError() const { return total_position_error_; }
  double GetTotalOrientationError() const { return total_orientation_error_; }
  size_t GetNumSamples() const { return num_samples_; }
  double GetMinPositionError() const { return min_position_error_; }
  double GetMaxPositionError() const { return max_position_error_; }
  double GetMinOrientationError() const { return min_orientation_error_; }
  double GetMaxOrientationError() const { return max_orientation_error_; }

  // Print summary statistics
  void PrintSummary() const;

  // Check if any data has been collected
  bool HasData() const { return num_samples_ > 0; }

private:
  // Position error statistics (2D Euclidean distance)
  double total_position_error_ = 0.0;
  double min_position_error_ = std::numeric_limits<double>::infinity();
  double max_position_error_ = 0.0;

  // Orientation error statistics (absolute yaw difference)
  double total_orientation_error_ = 0.0;
  double min_orientation_error_ = std::numeric_limits<double>::infinity();
  double max_orientation_error_ = 0.0;

  size_t num_samples_ = 0;
};

// Global trajectory scorer instance
extern TrajectoryScorer trajectory_scorer;
