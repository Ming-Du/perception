 #pragma once

#include <string>

#include "base/sensor_data_manager.h"
#include "base/track.h"
#include "lib/interface/base_shape_fusion.h"
#include "common/standard_kalman_filter/kalman_filter.h"
#include "base/configmanager.h"
namespace perception {
namespace fusion {

class PbfShapeFusion : public BaseShapeFusion {
 public:
  explicit PbfShapeFusion(TrackPtr track) : BaseShapeFusion(track) { }
  virtual ~PbfShapeFusion() {}
  PbfShapeFusion(const PbfShapeFusion&) = delete;
  PbfShapeFusion& operator=(const PbfShapeFusion&) = delete;

  bool Init() override;

  void UpdateWithMeasurement(const SensorObjectPtr measurement, double target_timestamp) override;

  void UpdateWithoutMeasurement(const std::string& sensor_id,
                                double measurement_timestamp,
                                double target_timestamp) override;
  void UpdateBusWithMeasurement(const SensorObjectPtr measurement, double target_timestamp);
  void UpdateSweeperWithMeasurement(const SensorObjectPtr measurement, double target_timestamp);

  std::string Name() const override;

  inline TrackPtr GetTrack() { return track_ref_; }

 private:
  void UpdatePolygon(const SensorObjectConstPtr& measurement);
  void UpdateCenter(const SensorObjectConstPtr &measurement);
  void UpdatePolygon();
  bool UpdatePedestrianYaw(const SensorObjectConstPtr& measurement);//单独更新行人的航向
  void StoreHistoryPositions();//存储行人测量值的历史数据，用于计算航向
  void ComputeBox(const SensorObjectConstPtr &measurement,const Eigen::Vector3f &size,PointCloud<PointD> &pts); 
  bool CheckBoxValid(std::vector<PointD>& box);
  std::vector<PointD>  OptBoxbySize(const std::vector<PointD>& polygon_others,
                                                 const int& min_index,
                                                 const Eigen::Vector3f& size);
  void ComputeBox(const SensorObjectConstPtr& measurement,const Eigen::Vector3d& position,const Eigen::Vector3f& size,const float& head,PointCloud<PointD>& pts);
  void UpdateYawByKF(const SensorObjectConstPtr& measurement) ;
  void UpdateSizeKF(const SensorObjectConstPtr& measurement);
  void InitBayes();//初始化贝叶斯
  void InitKalman(const float& srcyaw);//初始化卡尔曼
  bool CheckBagYaw(const float& dstyaw,const float& srcyaw);//检查异常值
  void BayesFilter(fusion::ObjectConstPtr src_obj);//计算贝叶斯概率
  void KalmanFilter(const float& srcyaw);
  int lostAI_age_=0;//记录AI的丢失次数
  int matchAI_times_=0;//记录AI的匹配次数
  int matchTotal_times_=0;//记录当前阶段全部的匹配次数
  bool ComputeTrackConfirmedState(const SensorObjectConstPtr& measurement);//统计track被确认的情况
  int ncoutbad_;//记录异常差值次数，航向
  int kalmantimes_;
  bool init_ = false;
  bool x_drec_;
  double yaw_kf_ = 0;
  double P_ = .0;
  float Q_ = M_PI_2;
  float R_ = M_PI_4;
  double Gain_ = .0;
  std::vector<float> bayes_probs_;
  double bayes_x_prob_ = 0.5;
  double bayes_anti_x_prob_ = 0.5;
  static bool s_use_camera_3d_;
  static float s_camera_radar_time_diff_th_;
  // Modify(@liuxinyu): obu_test
  static float s_obu_radar_time_diff_th_;
  Eigen::Vector3d lastposition_ = {0,0,0};
  std::vector<float>yaws_;//原始的测量值
  std::vector<float>yawsmodify_;//经过正反矫正的测量值
  float lasttrack_yaw_;
  std::vector<Eigen::Vector3d>history_positions_;
  int historypositionnum_= 60;
  int vehicle_;
  //@MODIFY lijian
  void InitKalmanSizeFilter(const SensorObjectConstPtr& measurement);
  perception::fusion::KalmanFilter kalman_filter_;
  bool initsizekm_=false;
  Eigen::MatrixXd transform_matrix_;
  Eigen::MatrixXd q_matrix_;
  Eigen::MatrixXd r_matrix_;
  Eigen::MatrixXd p_matrix_;
  Eigen::VectorXd opt_states_;
  bool isConfirmedDetection_ = false;
  std::map<std::string,PointCloud<PointD>> sensorpolygon_map_;
  bool ComputeNearestPt(const PointCloud<PointD>& pts,const Eigen::Vector3d& originpt,PointD* nesrestpt);
  //坐标转换
  bool ConvertfromUtm2Other(float otheryawutm,Eigen::Vector3d otherpositionutm,PointD srcpt,PointD* dstpt);
  bool ConvertfromOther2Utm(float otheryawutm,Eigen::Vector3d otherpositionutm,PointD srcpt,PointD* dstpt);
  void StoreSensorPolygon(const SensorObjectConstPtr& measurement,PointCloud<PointD> polygon);
};

}  // namespace fusion
}  // namespace perception
