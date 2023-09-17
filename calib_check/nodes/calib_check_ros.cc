// headers in STL
#include "calib_check_ros.h"

#include <chrono>
#include <cmath>
#include <ctime>
// headers in ROS
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

#include "common/include/pb_utils.h"
#include "perception/base/object.h"
#include "perception/base/proto/perception_component.pb.h"
#include "perception/base/timer.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#define TEST_TIME

static const double PI(3.141592653589793238462);

CalibCheckROS::CalibCheckROS()
    : nh_(ros::NodeHandle()),
      private_nh_(ros::NodeHandle("~")),
      imageTransport_(ros::NodeHandle()) {
  // ros related param
  private_nh_.param<bool>("baselink_support", baselink_support_, true);
  Init();
}

bool CalibCheckROS::Init() {
  perception::base::SensorManager* sensor_manager = perception::base::SensorManager::Instance();
  private_nh_.getParam("calib_check_config_path", config_file_);
  private_nh_.getParam("check_lidar", check_lidar_);
  ROS_INFO_STREAM("mid_config: " << config_file_);
  ROS_INFO_STREAM("check_lidar: " << check_lidar_);
  config_file_str_ = config_file_ + "calib_check_config.txt";
  ROS_INFO_STREAM("calib_check_config:  " << config_file_str_);
  //读取配置项
  config_parser_ptr_.reset(new calib_common::ConfigParser(config_file_str_));
  isCalib = config_parser_ptr_->getInt("calib_enbale_flag");
  calib_camera_name_ = config_parser_ptr_->getString("calib_camera_name");
  load_multable_calib_flag_ = config_parser_ptr_->getInt("load_multable_calib_flag");
  is_pub_frustumImg_ = config_parser_ptr_->getBool("is_pub_frustumImg");
  is_pub_camera_frustum_ = config_parser_ptr_->getBool("is_pub_camera_frustum");
  is_dedistortion_ = config_parser_ptr_->getBool("is_dedistortion");
  // rec_topic_name_lidar_cloud_ = config_parser_ptr_->getString("rec_topic_name_lidar_cloud");
  rec_topic_name_front_camera_obstacle_front60_ =
      config_parser_ptr_->getString("rec_topic_name_front_camera_obstacle_front60");
  rec_topic_name_lidar_cloud_ = check_lidar_;
  max_time_tr_ = config_parser_ptr_->getDouble("max_time_tr");
  image_pointcloud_diff_ = config_parser_ptr_->getDouble("image_pointcloud_diff");
  project_point_x_min_ = config_parser_ptr_->getDouble("project_point_x_min");
  project_point_x_max_ = config_parser_ptr_->getDouble("project_point_x_max");
  project_point_y_min_ = config_parser_ptr_->getDouble("project_point_y_min");
  project_point_y_max_ = config_parser_ptr_->getDouble("project_point_y_max");
  leaf_size_ = config_parser_ptr_->getDouble("leaf_size");
  
  //初始化
  T_camera2IMU_.setIdentity();
  T_IMU2camera_.setIdentity();
  T_lidar2IMU_.setIdentity();
  T_CameraIntr_.setIdentity(); 
  //导入各传感器内外参
  sensorInfoMap_ = sensor_manager->GetSensorInfoMap();
  // todo  读入所有传感器的标定参数
  for (auto p : sensorInfoMap_) {
    Eigen::Matrix4d m;
    m.setIdentity();
    if (sensor_manager->IsCamera(p.second.type())) {
      // TODO：p.first是string表示啥？p.second表示传感器所有信息
      // m : camera2world_pose?
      if (p.second.has_intrinsic() && p.second.has_extrinsic()) {
        MulitplyKandExtrinsic(p.first, p.second, m);
        sensorMatrixMap_.insert(std::make_pair(p.second.topic(), m));
        sensorname2topic_map_.insert(std::make_pair(p.first, p.second.topic()));
      }
    } else {
      Eigen::Affine3d other_TtoWorld_base;
      perception::base::SetSensorExtrinsics(p.second.extrinsic(), other_TtoWorld_base);
      // m : lidar2worl_pose?
      m = other_TtoWorld_base.matrix().inverse();
      sensorMatrixMap_.insert(std::make_pair(p.second.topic(), m));
      sensorname2topic_map_.insert(std::make_pair(p.first, p.second.topic()));
    }
    std::cout << "--------" << p.first << "--------"<< std::endl;
    std::cout << m << std::endl;
  }
  //手动调整相机外参
  if (isCalib) {
    LoadCalibrationData();
  }
  return true;
}

void CalibCheckROS::MulitplyKandExtrinsic(std::string sensor_name,
                                          const perception::base::SensorInfo sensor_info,
                                          Eigen::Matrix4d& camera2world_pose) {
  Eigen::Matrix4d T_IMU2camera;
  Eigen::Matrix4d T_CameraIntr;
  Eigen::VectorXd Distcoeff(5);
  T_IMU2camera.setIdentity();
  T_CameraIntr.setIdentity();
  Eigen::Affine3d camera_T_base;
  perception::base::SetSensorExtrinsics(sensor_info.extrinsic(), camera_T_base);

  T_IMU2camera = camera_T_base.matrix().inverse();
  extrinsics_camera_map_.insert(std::make_pair(sensor_info.topic(), T_IMU2camera));
  T_CameraIntr(0, 0) = sensor_info.intrinsic().matrix(0);
  T_CameraIntr(0, 1) = sensor_info.intrinsic().matrix(1);
  T_CameraIntr(0, 2) = sensor_info.intrinsic().matrix(2);
  T_CameraIntr(0, 3) = 0.0;
  T_CameraIntr(1, 0) = sensor_info.intrinsic().matrix(3);
  T_CameraIntr(1, 1) = sensor_info.intrinsic().matrix(4);
  T_CameraIntr(1, 2) = sensor_info.intrinsic().matrix(5);
  T_CameraIntr(1, 3) = 0.0;
  T_CameraIntr(2, 0) = sensor_info.intrinsic().matrix(6);
  T_CameraIntr(2, 1) = sensor_info.intrinsic().matrix(7);
  T_CameraIntr(2, 2) = sensor_info.intrinsic().matrix(8);
  T_CameraIntr.row(3) << 0.0, 0.0, 0.0, 1.0; // 构成4*4
  intrinsics_camera_map_.insert(std::make_pair(sensor_info.topic(), T_CameraIntr));
  camera2world_pose = T_CameraIntr * T_IMU2camera;
  Distcoeff(0) = sensor_info.distcoeff().distort_matrix(0);
  Distcoeff(1) = sensor_info.distcoeff().distort_matrix(1);
  Distcoeff(2) = sensor_info.distcoeff().distort_matrix(2);
  Distcoeff(3) = sensor_info.distcoeff().distort_matrix(3);
  Distcoeff(4) = sensor_info.distcoeff().distort_matrix(4);
  distcoeff_camera_map_.insert(std::make_pair(sensor_info.topic(), Distcoeff));
}

void CalibCheckROS::CreateROSPubSub() {
  sub_points_ = nh_.subscribe<sensor_msgs::PointCloud2>(rec_topic_name_lidar_cloud_, 10,
                                                        &CalibCheckROS::pointsCallback, this);
  sub_camera2d_ = nh_.subscribe(rec_topic_name_front_camera_obstacle_front60_, 1, &CalibCheckROS::CameraObjectsCallback, this);
  image_transport::ImageTransport it_pub(nh_);
  if (is_pub_frustumImg_) {
    pub_frustumImg_ = imageTransport_.advertise("/perception/calib_check/frustum_points_image",
                                                10);  //发布点云投影后的图片
    pub_frustumImg_nvjpeg_ =
        it_pub.advertise("/perception/calib_check/frustum_points_image", 1, false);
  }
  pub_2dbox_image_ = imageTransport_.advertise("/perception/calib_check/2d_box_image",10);
}

bool CalibCheckROS::LidarOnImage(const cv::Mat& img, std::vector<FrustumLidarPoint>& lidarPoints,
                                 std::multimap<int, pcl::PointXYZ>& camerObj_lidarPoint) {
#ifdef TEST_TIME
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();
#endif
  if (lidarPoints.empty()) {
    std::cout << "lidar points size:" << lidarPoints.size() << std::endl;
    return false;
  }
  int width = img.cols;
  int height = img.rows;
  // project lidar points
  // cv::Mat visImg(cameraObjects.height(), cameraObjects.width(), CV_8UC3, cv::Scalar(0, 0, 0));
  // cv::Mat visImg = img.clone();
  Eigen::Vector4d X;
  Eigen::Vector4d Y;
  X.setIdentity();
  Y.setIdentity();

  std::cout << "lidar points size:" << lidarPoints.size() << std::endl;
  float maxVal = 20.0;
  float val = 0.0;
  int red = 0;
  int green = 0;
  int blue = 0;
  for (auto it = lidarPoints.begin(); it != lidarPoints.end(); ++it) {
    if (std::isnan(it->point3D_x) || std::isnan(it->point3D_y)) {
      continue;
    }
    X(0) = it->point3D_x;
    X(1) = it->point3D_y;
    X(2) = it->point3D_z;
    X(3) = 1;
    // todo 到相机坐标系转换并归一化
    X = extrinsics_camera_map_[calib_camera_name_] * X;

    if (X(2) < 0.1) {
      continue;
    }

    Y = intrinsics_camera_map_[calib_camera_name_] * X;
    cv::Point pt;
    pt.x = Y(0) / Y(2);
    pt.y = Y(1) / Y(2);

    if (pt.x > width || pt.x < 0.0 || pt.y > height || pt.y < 0.0) {
      continue;
    }
    if (is_pub_frustumImg_) {
      val = it->point3D_x;
      // red = std::min(255, (int)(255 * abs((val - maxVal) / maxVal)));
      // green = std::min(255, (int)(255 * (1 - abs((val - maxVal) / maxVal))));
      blue = 153;
      green = 247;
      red = 255;
      cv::circle(img, pt, 5, cv::Scalar(0, green, red), -1);
    }
  }

#ifdef TEST_TIME
  end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);
  std::cout << __FUNCTION__ << " elapsed time: " << elapsed_seconds.count() << "s\n";
#endif
  return true;
}

bool CalibCheckROS::Camera2dBoxOnImage(const cv::Mat& image,
                                       const perception::VisualObjects& ori_camera,
                                       const perception::VisualObjects& undisort_camera) {
  if (ori_camera.objs_size() == 0) {
    return false;
  }
  for (size_t i = 0; i < ori_camera.objs_size(); i++) {
    cv::Point2d pt1 = cv::Point2d(ori_camera.objs(i).x(),
                                  ori_camera.objs(i).y());
    cv::Point2d pt2 = cv::Point2d(ori_camera.objs(i).x() + ori_camera.objs(i).width(),
                                  ori_camera.objs(i).y() + ori_camera.objs(i).height());
    cv::rectangle(image, pt1, pt2, cv::Scalar(0, 0, 255), 2);// red - bgr
  }

  for (size_t i = 0; i < undisort_camera.objs_size(); i++) {
    cv::Point2d pt11 = cv::Point2d(undisort_camera.objs(i).x(),
                                  undisort_camera.objs(i).y());
    cv::Point2d pt22 = cv::Point2d(undisort_camera.objs(i).x() + undisort_camera.objs(i).width(),
                                  undisort_camera.objs(i).y() + undisort_camera.objs(i).height());
    cv::rectangle(image, pt11, pt22, cv::Scalar(0, 255, 0), 2); // green
  }
  if (!is_pub_frustumImg_) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub_2dbox_image_.publish(msg);
  }
  return true;
}

// OpenCV camera model.
// CamParam:   fx, fy, cx, cy, k1, k2, p1, p2
void CalibCheckROS::DistortImage(cv::Mat& undistort_img, cv::Mat& distort_img) {
  double fx = intrinsics_camera_map_[calib_camera_name_](0, 0);
  double fy = intrinsics_camera_map_[calib_camera_name_](1, 1);
  double cx = intrinsics_camera_map_[calib_camera_name_](0, 2);
  double cy = intrinsics_camera_map_[calib_camera_name_](1, 2);
  double k1 = distcoeff_camera_map_[calib_camera_name_](0);
  double k2 = distcoeff_camera_map_[calib_camera_name_](1);
  double p1 = distcoeff_camera_map_[calib_camera_name_](2);
  double p2 = distcoeff_camera_map_[calib_camera_name_](3);
  cv::Mat cv_cam_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Mat cv_dist_params_ = (cv::Mat_<float>(4, 1) << k1, k2, p1, p2);
  cv::undistort(undistort_img, distort_img, cv_cam_matrix_, cv_dist_params_, cv::noArray());
}

void CalibCheckROS::DistortPoint(std::vector<cv::Point2f>& src, std::vector<cv::Point2f>& dst) {
  double fx = intrinsics_camera_map_[calib_camera_name_](0, 0);
  double fy = intrinsics_camera_map_[calib_camera_name_](1, 1);
  double cx = intrinsics_camera_map_[calib_camera_name_](0, 2);
  double cy = intrinsics_camera_map_[calib_camera_name_](1, 2);
  double k1 = distcoeff_camera_map_[calib_camera_name_](0);
  double k2 = distcoeff_camera_map_[calib_camera_name_](1);
  double p1 = distcoeff_camera_map_[calib_camera_name_](2);
  double p2 = distcoeff_camera_map_[calib_camera_name_](3);
  cv::Mat cv_cam_matrix_ = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  cv::Mat cv_dist_params_ = (cv::Mat_<float>(4, 1) << k1, k2, p1, p2);
  cv::undistortPoints(src, dst, cv_cam_matrix_, cv_dist_params_, cv::noArray(), cv_cam_matrix_);
}

void CalibCheckROS::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
#ifdef TEST_TIME
  std::chrono::time_point<std::chrono::system_clock> start, end;
  start = std::chrono::system_clock::now();
#endif

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *pcl_pc_ptr);

  // fine the newest camera detection objects
  double lidar_curent_time = pcl_pc_ptr->header.stamp * 1e-6;
  double tempTime = 0.0;
  cv::Mat lasteImg;
  cv::Mat lasteImg_temp;
  float min_timediff = image_pointcloud_diff_;  // origin 1->10
  perception::VisualObjects camera_measurements_ori;
  perception::VisualObjects camera_measurements_undisort;
  double find_camera_2d_time;
  {
    Eigen::VectorXd Distcoeff(5);
    Eigen::Matrix4d Intrinsics;
    Distcoeff = distcoeff_camera_map_[calib_camera_name_];
    Intrinsics = intrinsics_camera_map_[calib_camera_name_];
    int find_camera_2d = -1;
    for (auto it = 0; it < camera2d_data_deque_.size(); it++) {
      double camera_time = camera2d_data_deque_[it].header().stamp().sec() +
                           camera2d_data_deque_[it].header().stamp().sec() * 1e-9;
      double diff_camera_lidar = fabs(camera_time - lidar_curent_time);
      ROS_INFO_STREAM(" Lidar Time: " << std::setprecision(18) 
                                   << lidar_curent_time << " Cam Time: " << diff_camera_lidar);
      ROS_INFO_STREAM("cam_pointcloud_diff: " << fabs(diff_camera_lidar - lidar_curent_time));
      if (diff_camera_lidar < min_timediff) {
        find_camera_2d = it;
      }
    }
    if (find_camera_2d != -1) {
      camera_measurements_ori = camera2d_data_deque_[find_camera_2d];
      find_camera_2d_time = camera_measurements_ori.header().stamp().sec() +
                            camera_measurements_ori.header().stamp().sec() * 1e-9;
      std::cout << std::setprecision(18) << find_camera_2d_time << std::endl;
    }
    for (size_t i = 0; i < camera_measurements_ori.objs_size(); i++) {
      perception::VisualObject camera_object = camera_measurements_ori.objs(i);
      perception::VisualObject* camera_object_undisort = camera_measurements_undisort.add_objs();
      UndisortBox(camera_object, Intrinsics, Distcoeff, camera_object_undisort);
    }
  }
  cv_bridge::CvImagePtr temp_ptr;
  double min_image_time = 10000;
  int find_flag = -1;
  for (auto img_ptr = imgBuffer_map_tmp_[calib_camera_name_].begin();
       img_ptr != imgBuffer_map_tmp_[calib_camera_name_].end(); img_ptr++) {
    tempTime = (*img_ptr)->header.stamp.sec + (*img_ptr)->header.stamp.nsec * 1e-9;
    double system_time = ros::Time::now().toSec();
    ROS_INFO_STREAM("Sys Time: " << std::setprecision(18) << system_time << " Lidar Time: "
                                 << lidar_curent_time << " Img Time: " << tempTime);
    ROS_INFO_STREAM("image_pointcloud_diff: " << fabs(tempTime - lidar_curent_time));
    std::cout << "cam-img time diff: " << std::setprecision(18)  << find_camera_2d_time - tempTime << std::endl;
    if (fabs(tempTime - lidar_curent_time) < min_timediff) {
      min_timediff = fabs(tempTime - lidar_curent_time);
      temp_ptr = *img_ptr;
      find_flag = 1;
    }
    if (tempTime < min_image_time) {
      min_image_time = tempTime;
    }
  }
  std::string lidar_name = "lidar_main";
  for (auto& v:sensorname2topic_map_) { 
    if(v.second == rec_topic_name_lidar_cloud_) {
      lidar_name = v.first;
    }
  }

  if (find_flag == 1) {
    lasteImg_temp = temp_ptr->image.clone();
  } else {
    std::cout << "no image !!!!" << std::endl;
    return;
  }
  
  if (is_dedistortion_) {
    DistortImage(lasteImg_temp,lasteImg);
  } else {
    lasteImg = lasteImg_temp.clone();
  }
  if(!Camera2dBoxOnImage(lasteImg, camera_measurements_ori, camera_measurements_undisort)){
    std::cout << "2d box is null!" << std::endl; 
  }
  pcl::StopWatch filter_timer;
  std::vector<FrustumLidarPoint> frustLidarPoints;
  /* BUS 点云稀疏 不需要降采样*/
  if (lidar_name == "innolidar") {
    // 降采样 体素滤波
    float in_leaf_size=leaf_size_;
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setInputCloud(pcl_pc_ptr);
    sor.setLeafSize((float)in_leaf_size, (float)in_leaf_size, (float)in_leaf_size);
    sor.filter(*pcl_pc_ptr);
    // 滤除无效点云
    std::vector<int> indices2;
    pcl::removeNaNFromPointCloud(*pcl_pc_ptr, *pcl_pc_ptr, indices2);
  }
  for (size_t i = 0; i < pcl_pc_ptr->size(); i++) {
    pcl::PointXYZI point = pcl_pc_ptr->at(i);
    if (point.z < 0.1) {
      continue;
    } 
    if (point.x < project_point_x_min_ || point.x > project_point_x_max_) {
      continue;
    }
    if (point.y < project_point_y_min_ || point.y > project_point_y_max_) {
      continue;
    }
    FrustumLidarPoint tempPoint;
    tempPoint.id = i;
    tempPoint.point3D_x = point.x;
    tempPoint.point3D_y = point.y;
    tempPoint.point3D_z = point.z;
    tempPoint.point3D_r = point.intensity;
    frustLidarPoints.push_back(tempPoint);
  }
  ROS_INFO_STREAM("Filter time=" << filter_timer.getTime() << "ms");

  std::multimap<int, pcl::PointXYZ> camerObj_lidarPoint;
  // std::cout << "cameraObjects.objs_size():" << newest_camera_frame.objs_size() << std::endl;
  bool ret = LidarOnImage(lasteImg, frustLidarPoints, camerObj_lidarPoint);
  if (!ret)  //没有点或没有图像检测结果，直接退出
  {
    return;
  }
  if (is_pub_frustumImg_) {
    sensor_msgs::ImagePtr msg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", lasteImg).toImageMsg();
    pub_frustumImg_.publish(msg);
    pub_frustumImg_nvjpeg_.publish(msg);
  }

#ifdef TEST_TIME
  end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);
  std::cout << __FUNCTION__ << " elapsed time: " << elapsed_seconds.count() << "s\n";
#endif
}

void CalibCheckROS::ModifyCameraLidarExtrinsicMatrix(int c, bool& isCal) {
  switch (c) {
    case 0:
      // std::cout << "no key press" << std::endl;
      break;
    case 119:  //'w'
      rx_ += 0.1;
      UpdateExtrinsicMatrix();
      break;
    case 97:  //'a'
      rz_ += 0.1;
      UpdateExtrinsicMatrix();
      break;
    case 115:  //'s'
      rx_ -= 0.1;
      UpdateExtrinsicMatrix();
      break;
    case 100:  //'d'
      rz_ -= 0.1;
      UpdateExtrinsicMatrix();
      break;
    case 27:  //'Esc'退出
      SaveCalibrationMatrix(config_file_);
      isCal = false;
      break;
    default:
      break;
  }
}

void CalibCheckROS::UpdateExtrinsicMatrix() {
  std::cout << __FUNCTION__ << std::endl;
  T_IMU2camera_(0, 3) = tx_;
  T_IMU2camera_(1, 3) = ty_;
  T_IMU2camera_(2, 3) = tz_;

  double rx = rx_ / 180.0 * PI;
  double ry = ry_ / 180.0 * PI;
  double rz = rz_ / 180.0 * PI;

  Eigen::Quaterniond ag = Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());
  T_IMU2camera_.topLeftCorner(3, 3) = ag.matrix();
  T_IMU2camera_.row(3) << 0, 0, 0, 1;
  extrinsics_camera_map_[calib_camera_name_] = T_IMU2camera_;
}
inline bool FileIsexist(const std::string& name) {
  struct stat buffer;
  return (stat(name.c_str(), &buffer) == 0);
}

bool CalibCheckROS::LoadCalibConfig(std::string config_file) {
  config_file = config_file + calib_camera_name_ + ".xml";
  if (FileIsexist(config_file)) {
    cv::FileStorage fSettings(config_file, cv::FileStorage::READ);
    if (!fSettings.isOpened()) {
      std::cerr << "Failed to open settings file at: " << config_file << std::endl;
      return false;
    } else {
      ROS_INFO_STREAM("Sucessfully load calib config file:  " << config_file);
    }

    cv::Mat ex = cv::Mat_<double>::zeros(4, 4);

    fSettings["outer_matx"] >> ex;

    T_IMU2camera_ << ex.at<double>(0, 0), ex.at<double>(0, 1), ex.at<double>(0, 2),
        ex.at<double>(0, 3), ex.at<double>(1, 0), ex.at<double>(1, 1), ex.at<double>(1, 2),
        ex.at<double>(1, 3), ex.at<double>(2, 0), ex.at<double>(2, 1), ex.at<double>(2, 2),
        ex.at<double>(2, 3), ex.at<double>(3, 0), ex.at<double>(3, 1), ex.at<double>(3, 2),
        ex.at<double>(3, 3);
    extrinsics_camera_map_[calib_camera_name_] = T_IMU2camera_;
    return true;
  } else {
    ROS_INFO_STREAM("Fail load calib config file:  " << config_file << "\n"
                                                     << "please check loaffile!");
    return false;
  }
};

void CalibCheckROS::SaveCalibrationMatrix(std::string config_file) {
  config_file = config_file + calib_camera_name_ + ".xml";
  if (FileIsexist(config_file)) {
    std::cout << "calib config file Isexist   UPDATA!" << std::endl;
  } else {
    std::cout << "creat new config file" << std::endl;
  }
  cv::FileStorage fSettings(config_file, cv::FileStorage::WRITE);

  cv::Mat outer_matx = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
  outer_matx.at<double>(0, 0) = T_IMU2camera_(0, 0);
  outer_matx.at<double>(0, 1) = T_IMU2camera_(0, 1);
  outer_matx.at<double>(0, 2) = T_IMU2camera_(0, 2);
  outer_matx.at<double>(0, 3) = T_IMU2camera_(0, 3);
  outer_matx.at<double>(1, 0) = T_IMU2camera_(1, 0);
  outer_matx.at<double>(1, 1) = T_IMU2camera_(1, 1);
  outer_matx.at<double>(1, 2) = T_IMU2camera_(1, 2);
  outer_matx.at<double>(1, 3) = T_IMU2camera_(1, 3);
  outer_matx.at<double>(2, 0) = T_IMU2camera_(2, 0);
  outer_matx.at<double>(2, 1) = T_IMU2camera_(2, 1);
  outer_matx.at<double>(2, 2) = T_IMU2camera_(2, 2);
  outer_matx.at<double>(2, 3) = T_IMU2camera_(2, 3);
  outer_matx.at<double>(3, 0) = T_IMU2camera_(3, 0);
  outer_matx.at<double>(3, 1) = T_IMU2camera_(3, 1);
  outer_matx.at<double>(3, 2) = T_IMU2camera_(3, 2);
  outer_matx.at<double>(3, 3) = T_IMU2camera_(3, 3);

  cv::Mat inner_matx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
  cv::Mat DistCoeff = (cv::Mat_<double>(1, 5) << 0, 0, 0, 0, 0);
  fSettings << "inner_matx" << inner_matx;
  fSettings << "outer_matx" << outer_matx;
  fSettings << "DistCoeff" << DistCoeff;
  fSettings.release();
}

void CalibCheckROS::LoadCalibrationData() {
  if (load_multable_calib_flag_) {
    if (!LoadCalibConfig(config_file_)) {
      T_IMU2camera_ = extrinsics_camera_map_[calib_camera_name_];
    }
  } else {
    T_IMU2camera_ = extrinsics_camera_map_[calib_camera_name_];
  }
  T_CameraIntr_ = intrinsics_camera_map_[calib_camera_name_];
  // todo T_K是内参
  tx_ = T_IMU2camera_(0, 3);
  ty_ = T_IMU2camera_(1, 3);
  tz_ = T_IMU2camera_(2, 3);

  // xyz - euler
  Eigen::Matrix3d rotation = T_IMU2camera_.topLeftCorner(3, 3);
  // [0:pi]x[-pi:pi]x[-pi:pi]
  Eigen::Vector3d angle = rotation.eulerAngles(0, 1, 2) / PI * 180;

  std::cout << __FUNCTION__ << ", " << __LINE__ << std::endl;
  std::cout << "T_CameraIntr_: " << T_CameraIntr_ << std::endl;
  std::cout << "T_IMU2camera_: " << T_IMU2camera_ << std::endl;
  std::cout << "angle:" << angle << std::endl;

  rx_ = angle(0);
  ry_ = angle(1);
  rz_ = angle(2);

  std::cout << __FUNCTION__ << ", " << __LINE__ << std::endl;
  std::cout << " tx ty tz:" << tx_ << ", " << ty_ << ", " << tz_ << std::endl;
  std::cout << " rx ry rz:" << rx_ << ", " << ry_ << ", " << rz_ << std::endl;
}

void CalibCheckROS::CameraObjectsCallback(const std_msgs::StringConstPtr& msg) {
  perception::VisualObjects camera_measurement;
  camera_measurement.ParseFromString(msg->data);
  if (camera2d_data_deque_.size() > 10) {
    camera2d_data_deque_.pop_front();
  }
  camera2d_data_deque_.emplace_back(camera_measurement);
  while (camera2d_data_deque_.size() > 10) {
    camera2d_data_deque_.pop_front();
  }
}

void CalibCheckROS::UndisortBox(const perception::VisualObject& camera_object, Eigen::Matrix4d& Intrinsics, Eigen::VectorXd& Distcoeff, perception::VisualObject* camera_object_undisort) {
  if (Distcoeff.size() == 0) {
    ROS_WARN_STREAM("Distcoeff Param is NULL");
  }
  // ru: right-up rd: right-down ld: left-down lu: left-up
  // Eigen::Vector2d ru, rd, ld, lu;
  // double u_ru = camera_object.x() + camera_object.width();
  // double v_ru = camera_object.y();
  // double u_rd = camera_object.x() + camera_object.width();
  // double v_rd = camera_object.y() + camera_object.height();
  // double u_ld = camera_object.x();
  // double v_ld = camera_object.y() + camera_object.height();
  // double u_lu = camera_object.x();
  // double v_lu = camera_object.y();
  // ru = GetUndisoredCoord( u_ru,  v_ru, Intrinsics, Distcoeff);
  // rd = GetUndisoredCoord( u_rd,  v_rd, Intrinsics, Distcoeff);
  // ld = GetUndisoredCoord( u_ld,  v_ld, Intrinsics, Distcoeff);
  // lu = GetUndisoredCoord( u_lu,  v_lu, Intrinsics, Distcoeff);
  std::cout << " = = = Not undisorted = = = \n"
            << camera_object.x() << " " << camera_object.y() << " " << camera_object.width() << " "
            << camera_object.height() << std::endl;
  // double width = ru(0) - lu(0);
  // double height = rd(1) - ru(1);

  // std::cout << " = = = undisorted = = = \n"
  //           << lu(0) << " " << lu(1) << " " << width << " " << height << std::endl;
  // camera_object_undisort->set_x(lu(0));
  // camera_object_undisort->set_y(lu(1));
  // camera_object_undisort->set_width(width);
  // camera_object_undisort->set_height(height);

  std::vector<cv::Point2f> src_ru, src_rd, src_ld, src_lu;
  src_ru.push_back(cv::Point2f(camera_object.x() + camera_object.width(), camera_object.y()));
  src_rd.push_back(cv::Point2f(camera_object.x() + camera_object.width(),
                       camera_object.y() + camera_object.height()));
  src_ld.push_back(cv::Point2f(camera_object.x(), camera_object.y() + camera_object.height()));
  src_lu.push_back(cv::Point2f(camera_object.x(), camera_object.y()));
  std::vector<cv::Point2f> ru, rd, ld, lu;
  DistortPoint(src_ru, ru);
  DistortPoint(src_rd, rd);
  DistortPoint(src_ld, ld);
  DistortPoint(src_lu, lu);
  double width = ru[0].x - lu[0].x;
  double height = rd[0].y - ru[0].y;
    std::cout << " = = = undisorted = = = \n"
            << lu[0].x << " " << lu[0].y << " " << width << " " << height << std::endl;
  camera_object_undisort->set_x(lu[0].x );
  camera_object_undisort->set_y(lu[0].y );
  camera_object_undisort->set_width(width);
  camera_object_undisort->set_height(height);

}

Eigen::Vector2d CalibCheckROS::GetUndisoredCoord(double u, double v, Eigen::Matrix4d& Intrinsics, Eigen::VectorXd& Distcoeff) {
  // std::cout << "Intrinsics\n" << Intrinsics.matrix() << std::endl;
  // std::cout << "Distcoeff\n" << Distcoeff.transpose() << std::endl;
  Eigen::Vector2d disorted_coord;
  // std::cout << "u: " << u << "v: " << v << std::endl;
  // 归一化平面坐标
  double x_ = (u - Intrinsics(0, 2)) / Intrinsics(0, 0);
  double y_ = (v - Intrinsics(1, 2)) / Intrinsics(1, 1);
  // std::cout << "x_: " << x_ << "y_: " << y_ << std::endl;
  double R = std::sqrt(x_ * x_ + y_ * y_);
  double x_disorted = x_ * (1 + Distcoeff(0) * R * R + Distcoeff(1) * R * R * R * R) + 2 * Distcoeff(2) * x_ * y_ + Distcoeff(3) * (R * R + 2 * x_ + x_);
  double y_disorted = y_ * (1 + Distcoeff(0) * R * R + Distcoeff(1) * R * R * R * R) + 2 * Distcoeff(2) * (R*R+2*y_ * y_) + 2 * Distcoeff(3) * x_* y_;
  // std::cout << "x_disorted: " << x_disorted << "y_disorted: " << y_disorted << std::endl;
  double u_disorted = Intrinsics(0, 0) * x_disorted + Intrinsics(0, 2);
  double v_disorted = Intrinsics(1, 1) * y_disorted + Intrinsics(1, 2);
  // std::cout <<  "v_disorted: " << u_disorted << "v_disorted: " << v_disorted << std::endl;
  disorted_coord << u_disorted, v_disorted;
  return disorted_coord;
}