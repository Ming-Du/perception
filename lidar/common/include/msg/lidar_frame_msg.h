#ifndef RS_SDK_LIDAR_FRAME_MSG_H_
#define RS_SDK_LIDAR_FRAME_MSG_H_

#include "grid_map/grid_map.h"

#include "basic_type/axis_type.h"
#include "basic_type/bird_view_space.h"
#include "basic_type/freespace.h"
#include "basic_type/object.h"
#include "basic_type/pose.h"

#include <condition_variable>
#include <map>
#include <mutex>

#include <boost/circular_buffer.hpp>
#include <eigen3/Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "common/proto/convexhull.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/hadmap.pb.h"
#include "common/proto/localization.pb.h"
#include "common/proto/object.pb.h"

namespace robosense {

struct RefineData {
    std::vector<int> erase_objs;
    std::map<int, std::vector<int>> measure_to_trackers;
    std::map<int, std::vector<int>> tracker_to_measures;
};

struct DenoiseGridData {
    std::vector<int> point_num;
    std::vector<int> pos_x;
    std::vector<int> pos_y;
    std::vector<Eigen::Vector2d> noisemapobj;
    std::vector<Eigen::Vector2d> noisemapnos;
    std::vector<NoiseType> noise_type;
    double xmin;
    double xmax;
    double ymin;
    double ymax;
    double xsize;
    double ysize;
    double unit_size; // 1
    bool enable = false;
};

struct GroundCoeff{
    int start_ring;
    int end_ring;
    Eigen::Vector4f ground_coe; //平面方程系数
    Eigen::Vector3f ground_normal; //平面法向量
};

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
using PointCloudPtr = PointCloud::Ptr;

enum class HackAreaType {
    NORMAL = 0,
    ENABLE_CONE = 1,
    ENABLE_UPHILL = 2,
};

enum class TriggerType {
    NORMAL = 0,
    ENABLE_NOISE = 1,
    ENABLE_YAW = 2,
    ENABLE_TRAFFIC = 3,
    ENABLE_OVERSIZE = 4,
};

class LidarFrameMsg {
public:
    using Ptr = std::shared_ptr<LidarFrameMsg>;

    LidarFrameMsg() {
        axis_pose_map.clear();
        for (auto itr = kAxisTypeSet.begin(); itr != kAxisTypeSet.end(); ++itr) {
            Pose::Ptr tmp_pose_ptr(new Pose);
            axis_pose_map[*itr] = tmp_pose_ptr;
        }
    }

    std::string frame_id; // every frame has it unique frame_id
    double timestamp = 0;
    double sys_timestamp = 0;

    Pose::Ptr global_pose_ptr;
    std::map<AxisType, Pose::Ptr> axis_pose_map;
    AxisType status;
    HackAreaType hackarea_status = HackAreaType::NORMAL;

    bool is_rain_mode = false;
    PointCloud::Ptr scan_ptr;
    bool denoise_enable;
    DenoiseGridData denoise_grid;

    GridMapPtr grid_map_ptr;

    bool msg_pub_flag = false;
    bool is_roifilter_processed=false;
    bool is_pointcloud_map = false;
    bool is_semantic_map = false;
    GroundCoeff leishen_semantic_map;
    std::vector<GroundCoeff> semantic_map;
    std::vector<GroundCoeff> matched_semantic_map;
    std::vector<Eigen::Vector3f> pointcloud_map;
    std::vector<TriggerType> trigger_vec = {4, TriggerType::NORMAL};
    std::vector<std::string> trigger_info;
    // results
    std::vector<Object::Ptr> objects;
    std::vector<Object::Ptr> objects_rule;
    std::vector<Object::Ptr> objects_ai;

    std::vector<Object::Ptr> objects_ai_refine;
    std::vector<Object::Ptr> objects_refine;
    std::vector<Object::Ptr> objects_proposals;

    std::vector<boost::circular_buffer<Object::Ptr>> objects_history;

    std::vector<Object::Ptr> objects_vehicle;

    std::vector<int> valid_indices;
    std::vector<int> roifiltered_indices;

    std::vector<int> noise_indices;
    std::vector<int> out_map_indices;
    RefineData refine_data;
     
    cv::Mat road_map_mat_info;
    Eigen::Transform<double, 3, Eigen::Affine> t_transform;

    // rviz or debug
    std::vector<Object::Ptr> objects_rule_debug;
    std::vector<Object::Ptr> objects_ai_debug;
    std::vector<Object::Ptr> objects_denoise_del;

    std::vector<Eigen::Vector2d> roadmap;
    std::vector<Eigen::Vector2d> binmap;

    std::vector<std::vector<float>> shape_indices;

    void transAxis(const AxisType &s, const std::string &name = "");
    void transGlobal2VehicleMat(Eigen::Matrix4d &transform_mat);
    void transVehicle2GlobalMat(Eigen::Matrix4d &transform_mat);
};

} // namespace robosense

#endif // RS_SDK_LIDAR_FRAME_MSG_H_
