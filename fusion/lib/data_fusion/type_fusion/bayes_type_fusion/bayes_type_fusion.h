#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "lib/interface/base_type_fusion.h"

namespace perception {
namespace fusion {


class BayesTypeFusion : public BaseTypeFusion {
 public:
  explicit BayesTypeFusion(TrackPtr track);
  ~BayesTypeFusion() {}

  static bool Init();

  // @brief: update track state with measurement
  // @param [in]: measurement
  // @param [in]: target_timestamp
  void UpdateWithMeasurement(const SensorObjectPtr measurement,
                             double target_timestamp) override;

  void UpdateWithoutMeasurement(const std::string &sensor_id,
                                double measurement_timestamp,
                                double target_timestamp,
                                double min_match_dist) override;

  std::string Name() const;

 private:
  static std::string name_;
  std::vector<double> bel_probs_;

};

}  // namespace fusion
}  // namespace perception
