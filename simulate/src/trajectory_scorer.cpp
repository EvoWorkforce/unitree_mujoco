#include "trajectory_scorer.h"

// Global trajectory scorer instance definition
TrajectoryScorer trajectory_scorer;

void TrajectoryScorer::Reset()
{
  total_position_error_ = 0.0;
  total_orientation_error_ = 0.0;
  num_samples_ = 0;
  min_position_error_ = std::numeric_limits<double>::infinity();
  max_position_error_ = 0.0;
  min_orientation_error_ = std::numeric_limits<double>::infinity();
  max_orientation_error_ = 0.0;
}

void TrajectoryScorer::UpdatePositionError(double x_error, double y_error)
{
  // Calculate 2D Euclidean distance (position error norm)
  double position_error_norm = std::sqrt(x_error * x_error + y_error * y_error);
  
  total_position_error_ += position_error_norm;
  min_position_error_ = std::min(min_position_error_, position_error_norm);
  max_position_error_ = std::max(max_position_error_, position_error_norm);
  
  num_samples_++;
}

void TrajectoryScorer::UpdateOrientationError(double yaw_error)
{
  // Use absolute value for orientation error
  double abs_yaw_error = std::abs(yaw_error);
  
  total_orientation_error_ += abs_yaw_error;
  min_orientation_error_ = std::min(min_orientation_error_, abs_yaw_error);
  max_orientation_error_ = std::max(max_orientation_error_, abs_yaw_error);
}

double TrajectoryScorer::GetAveragePositionError() const
{
  if (num_samples_ == 0)
    return 0.0;
  return total_position_error_ / static_cast<double>(num_samples_);
}

double TrajectoryScorer::GetAverageOrientationError() const
{
  if (num_samples_ == 0)
    return 0.0;
  return total_orientation_error_ / static_cast<double>(num_samples_);
}

double TrajectoryScorer::GetCompositeScore(double orientation_weight) const
{
  // Composite score: lower is better
  // Combines position error (m) and orientation error (rad) into a single metric
  // Default weight of 0.5 means 1 radian error ≈ 0.5 meter position error
  return GetAveragePositionError() + orientation_weight * GetAverageOrientationError();
}

void TrajectoryScorer::PrintSummary() const
{
  if (num_samples_ == 0)
  {
    std::cout << "Trajectory Scorer: No data collected" << std::endl;
    return;
  }

  std::cout << "\n============== Trajectory Scoring Summary ===============\n" << std::endl;
  std::cout << "  Time Steps: " << num_samples_ << std::endl;
  std::cout << "\n  Position Error (2D):" << std::endl;
  std::cout << "    Average: " << GetAveragePositionError() << " m" << std::endl;
  std::cout << "    Total:   " << total_position_error_ << " m" << std::endl;
  std::cout << "    Min:     " << min_position_error_ << " m" << std::endl;
  std::cout << "    Max:     " << max_position_error_ << " m" << std::endl;
  std::cout << "\n  Orientation Error (Yaw):" << std::endl;
  std::cout << "    Average: " << (GetAverageOrientationError() * 180.0 / M_PI) << " deg (" 
  << GetAverageOrientationError() << " rad)" << std::endl;
  std::cout << "    Total:   " << (total_orientation_error_ * 180.0 / M_PI) << " deg (" 
  << total_orientation_error_ << " rad)" << std::endl;
  std::cout << "    Min:     " << (min_orientation_error_ * 180.0 / M_PI) << " deg (" 
  << min_orientation_error_ << " rad)" << std::endl;
  std::cout << "    Max:     " << (max_orientation_error_ * 180.0 / M_PI) << " deg (" 
  << max_orientation_error_ << " rad)" << std::endl;
  std::cout << "\n=========================================================\n" << std::endl;
  std::cout << "  *** Your FINAL SCORE: " << GetCompositeScore(0.1) << " (lower is better) ***" << std::endl;
  std::cout << "\n=========================================================\n" << std::endl;
}
