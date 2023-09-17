#pragma once

#include <vector>
#include "Eigen/Dense"

namespace perception {
namespace base {

class KMSolver {
 public:
  KMSolver() = default;
  ~KMSolver() = default;

  bool Init(const std::vector<std::vector<float>>& association_mat, 
            const int track_size, const int measurement_size);

  std::vector<std::pair<int, int>> MinimizeAssignment();
  
 private:
  bool DFS(const int idx);

 private:
  // Eigen::MatrixXf solver_matrix_;
  std::vector<std::vector<float>> solver_matrix_;
  int solver_matrix_size_;

  std::vector<float> expect_x_, expect_y_;
  std::vector<int> assign_x_, assign_y_;
  std::vector<int> match_x_, match_y_;
  std::vector<std::pair<int, int>> match_;

};

}
}