#include "perception/base/km_solver/km_solver.h"

#include <numeric>

namespace perception {
namespace base {

bool KMSolver::Init(const std::vector<std::vector<float>>& association_mat, 
                    const int track_size, const int measurement_size) {

  if (track_size != association_mat.size()) {
    return false;
  }

  solver_matrix_size_ = std::max(track_size, measurement_size);
  // solver_matrix_ = Eigen::MatrixXf::Zero(solver_matrix_size_, solver_matrix_size_);
  solver_matrix_.assign(solver_matrix_size_, 
                        std::vector<float>(solver_matrix_size_, 0));

  expect_x_.assign(solver_matrix_size_, 0);
  expect_y_.assign(solver_matrix_size_, 0);
  assign_x_.assign(solver_matrix_size_, 0);
  assign_y_.assign(solver_matrix_size_, 0);
  match_x_.assign(solver_matrix_size_, -1);
  match_y_.assign(solver_matrix_size_, -1);
  match_.resize(solver_matrix_size_);

  for (int tidx = 0; tidx < track_size; ++tidx) {
    expect_x_[tidx] = (std::numeric_limits<float>::min)();
    for (int midx = 0; midx < measurement_size; ++midx) {
      solver_matrix_[tidx][midx] = association_mat[tidx][midx];
      expect_x_[tidx] = std::max(association_mat[tidx][midx], expect_x_[tidx]);
    }
  }
  return true;
}

bool KMSolver::DFS(const int tidx) {
  assign_x_[tidx] = 1;
  for (int midx = 0; midx < solver_matrix_size_; ++midx) {
    float expect = expect_x_[tidx] + expect_y_[midx];
    if (!assign_y_[midx] && fabs(expect - solver_matrix_[tidx][midx]) < 1e-3 ) {
      assign_y_[midx] = 1;
      // if (match_[midx].second == -1 || DFS(match_[midx].second)) {
      if (match_y_[midx] == -1 || DFS(match_y_[midx])) {
        match_x_[tidx] = midx;
        match_y_[midx] = tidx;
        // match_[tidx].first = midx;
        // match_[midx].second = tidx;
        return true;
      }
    }
  }
  return false;
}

std::vector<std::pair<int, int>> KMSolver::MinimizeAssignment() {
  for (int tidx = 0; tidx < solver_matrix_size_; ++tidx) {
    while (true) {

      assign_x_.assign(solver_matrix_size_, 0);
      assign_y_.assign(solver_matrix_size_, 0);

      if (DFS(tidx)) // 给当前候选找一个最佳匹配目标
        break;

      float weight_offset = (std::numeric_limits<float>::max)();
      for (int row = 0; row < solver_matrix_size_; ++row) {
        if (!assign_x_[row]) {
          continue;
        }

        for (int col = 0; col < solver_matrix_size_; ++col) {
          if (!assign_y_[col]) {
            weight_offset = 
              std::min(weight_offset, expect_x_[row] + expect_y_[col] - solver_matrix_[row][col]);
          }
        }
      }
      if (weight_offset == 0) {
        // max_w = -1;
        // cout << "No max weight match" << endl;
        // return false;
        return std::vector<std::pair<int, int>>(0);
      }
      for (int j = 0; j < solver_matrix_size_; ++j) {
        expect_x_[j] -= assign_x_[j] ? weight_offset : 0;
        expect_y_[j] += assign_y_[j] ? weight_offset : 0;
      }
    }
  }
  for (int tidx = 0; tidx < solver_matrix_size_; ++tidx) {
    float weight_sum = 
      std::accumulate(solver_matrix_[tidx].begin(), solver_matrix_[tidx].end(), 0);
    if (weight_sum == 0 || solver_matrix_[tidx][match_x_[tidx]] == 0) {
      match_x_[tidx] = -1;
    }
    if (weight_sum < 0) { // 不做track
      match_x_[tidx] = -2;
    }
  }
  for (int midx = 0; midx < solver_matrix_size_; ++midx) {
    if (match_y_[midx] != -1 && match_x_[match_y_[midx]] == -1) {
      match_y_[midx] = -1;
    }
  }
  for (int idx = 0; idx < solver_matrix_size_; ++idx) {
    match_[idx].first = match_x_[idx];
    match_[idx].second = match_y_[idx];
  }
  return match_;
}

// std::vector<std::pair<int, int>> KMSolver::GetMatch() const {
//   return match_;
// }

}
}