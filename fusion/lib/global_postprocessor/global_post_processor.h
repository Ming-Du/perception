#pragma once

#include <deque>
#include "Eigen/Dense"

#include "common/include/log.h"
#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"

namespace perception {
namespace fusion {

class GlobalPostProcessor {
 public:
  GlobalPostProcessor() = default;
  ~GlobalPostProcessor() = default;

  void ComputeGlobalState(TrackedObjects& objects, localization::Localization& global_odom);

 private:
  double UtmDeviation(double* host_utm_x,
                      double* host_utm_y,
                      const double host_heading,
                      const double distance_x,
                      const double distance_y);
  void ConvertLocalToGlobal(TrackedObject* tracked_object_ptr,
                            const localization::Localization localization);
  //  private:
};

}  // namespace fusion
}  // namespace perception