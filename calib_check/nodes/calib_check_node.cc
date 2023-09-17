// headers in STL
#include <iostream>

// headers in local files
#include <termios.h>

#include <thread>

#include "calib_check_ros.h"

std::unordered_map<std::string, std::list<cv_bridge::CvImagePtr>> imgBuffer_map_tmp_;
#define CAM_NUM 6

char getch() {
  fd_set set;
  struct timeval timeout;
  int rv;
  char buff = 0;
  int len = 1;
  int filedesc = 0;
  FD_ZERO(&set);
  FD_SET(filedesc, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;

  rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

  struct termios old = {0};
  if (tcgetattr(filedesc, &old) < 0) ROS_ERROR("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(filedesc, TCSANOW, &old) < 0) ROS_ERROR("tcsetattr ICANON");

  if (rv == -1)
    ROS_ERROR("select");
  else if (rv == 0) {
    // ROS_INFO("no_key_pressed");
  }

  else
    read(filedesc, &buff, len);

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(filedesc, TCSADRAIN, &old) < 0) ROS_ERROR("tcsetattr ~ICANON");
  return (buff);
}

void ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImagePtr cam_image_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (!cam_image_ptr->image.empty()) {
      if (imgBuffer_map_tmp_["/sensor/camera/sensing/image_raw_60"].size() >= 10) {
        imgBuffer_map_tmp_["/sensor/camera/sensing/image_raw_60"].front().reset();
        imgBuffer_map_tmp_["/sensor/camera/sensing/image_raw_60"].pop_front();
      }
      imgBuffer_map_tmp_["/sensor/camera/sensing/image_raw_60"].push_back(cam_image_ptr);
    }
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void Image30Callback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_bridge::CvImagePtr cam_image_ptr =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (!cam_image_ptr->image.empty()) {
      if (imgBuffer_map_tmp_["/sensor/camera/sensing/image_raw_30"].size() >= 10) {
        imgBuffer_map_tmp_["/sensor/camera/sensing/image_raw_30"].front().reset();
        imgBuffer_map_tmp_["/sensor/camera/sensing/image_raw_30"].pop_front();
      }
      imgBuffer_map_tmp_["/sensor/camera/sensing/image_raw_30"].push_back(cam_image_ptr);
    }
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "calib_check");
  ROS_INFO("input arg:");

  for (int i = 0; i < argc; i++) {
    ROS_INFO_STREAM(argv[i]);
  }
  ros::NodeHandle nh("~");
  // google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // set ROS LOG output level : Debug	Info Warn Error Fatal Count
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

  std::shared_ptr<CalibCheckROS> calib_check_ptr = std::make_shared<CalibCheckROS>();
  calib_check_ptr->CreateROSPubSub();

  std::string camera_topic[CAM_NUM];
  std::string img_type;
  calib_common::ConfigParserPtr config_parser_ptr;
  std::string config_file;
  nh.getParam("calib_check_config_path", config_file);
  ROS_INFO_STREAM("mid_config: " << config_file);
  std::string config_file_str = config_file + "calib_check_config.txt";
  ROS_INFO_STREAM("calib_check_config:  " << config_file_str);
  config_parser_ptr.reset(new calib_common::ConfigParser(config_file_str)); 
  camera_topic[0] = config_parser_ptr->getString("rec_topic_name_front_image_60");
  camera_topic[1] = config_parser_ptr->getString("rec_topic_name_front_image_30");
  camera_topic[2] = config_parser_ptr->getString("rec_topic_name_front_image_120");
  camera_topic[3] = config_parser_ptr->getString("rec_topic_name_front_image_left120");
  camera_topic[4] = config_parser_ptr->getString("rec_topic_name_front_image_right120");
  camera_topic[5] = config_parser_ptr->getString("rec_topic_name_back_image_back120");
  img_type = config_parser_ptr->getString("img_type");
  void (*ImageCallbackPtr[CAM_NUM])(const sensor_msgs::ImageConstPtr &img_msg)={ImageCallback,Image30Callback};
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Subscriber img_sub[CAM_NUM];
  for (int i = 0; i < 2; i++) {
    img_sub[i] = it.subscribe(camera_topic[i], 1, ImageCallbackPtr[i],
                              image_transport::TransportHints(img_type));
  }

  bool isCal = calib_check_ptr->isCalib;
  int c = 0;
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    if (isCal) {
      c = getch();
      calib_check_ptr->ModifyCameraLidarExtrinsicMatrix(c, isCal);
    }
    rate.sleep();
  }
  return 0;
}


