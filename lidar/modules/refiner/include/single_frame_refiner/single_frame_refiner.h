#pragma once

#include "common/include/msg/lidar_frame_msg.h"
#include "common/include/config/rs_yamlReader.h"
#include "common/include/basic_type/rotate_box.h"
#include "common/include/tic_toc.h"
#include <omp.h>
namespace robosense {

class SingleFrameRefiner {
public:
    using Ptr = std::shared_ptr<SingleFrameRefiner>;

    SingleFrameRefiner();

    void init(const RefinerParam& param);
    void perception(const LidarFrameMsg::Ptr &msg_ptr);

private:
    void single_box_refine(const LidarFrameMsg::Ptr &msg_ptr);
    void rbInBoxRefine(Object::Ptr& curr_obj, const LidarFrameMsg::Ptr& msg_ptr);
    void rbOutBoxRefine(Object::Ptr& curr_obj, const LidarFrameMsg::Ptr& msg_ptr, const std::vector<Eigen::Vector3d> &out_point);
    bool point_in_polygon(const Eigen::Vector3d &p, Eigen::MatrixXd &poly);
    void lShapeBoxFitting(Object::Ptr& curr_obj, const  std::vector<Eigen::Vector2d> &bounding_point, double &yaw);
    void getBoundingInfo(Object::Ptr& curr_obj, const LidarFrameMsg::Ptr& msg_ptr, std::vector<Eigen::Vector2d> &bounding_point, double &box_dist, bool modify_cart=false);
    double calc_nearest_criterion(const Eigen::MatrixXd &c1, const Eigen::MatrixXd &c2,double &c1_min,double &c1_max,double &c2_min, double &c2_max );
    void calc_rect_contour();

    inline void calc_cross_point(const double a0, const double a1,const double b0, const double b1,const double c0, const double c1, double &x,double &y) {
        x = (b0 * (-c1) - b1 * (-c0)) / (a0 * b1 - a1 * b0);
        y = (a1 * (-c0) - a0 * (-c1)) / (a0 * b1 - a1 * b0);
    }

private:
    RefinerParam params_;

    const double dist_th = 7; 
    double box_dist =0; 
    const double min_dist_of_nearest_crit_ = 0.01;
    double dtheta_deg_for_search_;
    const int num_threads=2;
    std::vector<double> a_;
    std::vector<double> b_;
    std::vector<double> c_;
    const double sin_theta[90] = {0,
                                  0.0174524, 0.0348995, 0.0523360, 0.0697565, 0.0871557, 0.1045285, 0.1218693, 0.1391731, 0.1564345, 0.1736482,
                                  0.1908090, 0.20791169, 0.22495105, 0.2419219, 0.25881905, 0.27563736, 0.2923717, 0.30901699, 0.32556815, 0.34202014,
                                  0.35836795, 0.37460659, 0.39073113, 0.40673664, 0.42261826, 0.43837115, 0.4539905, 0.46947156, 0.48480962, 0.5,
                                  0.51503807, 0.52991926, 0.54463904, 0.5591929, 0.57357644, 0.58778525, 0.60181502, 0.61566148, 0.62932039, 0.64278761,
                                  0.65605903, 0.66913061, 0.68199836, 0.69465837, 0.70710678, 0.7193398, 0.7313537, 0.74314483, 0.75470958, 0.76604444,
                                  0.77714596, 0.78801075, 0.79863551, 0.80901699, 0.81915204, 0.82903757, 0.838670568, 0.8480481, 0.85716730, 0.8660254,
                                  0.87461971, 0.88294760, 0.89100652, 0.89879405, 0.90630779, 0.91354546, 0.920504853, 0.9271839, 0.93358043, 0.9396926,
                                  0.94551858, 0.95105652, 0.95630476, 0.961261696, 0.96592583, 0.97029573, 0.974370065, 0.9781476, 0.98162718, 0.9848078,
                                  0.98768834, 0.99026807, 0.99254615, 0.994521895, 0.9961947, 0.99756405, 0.998629535, 0.9993908, 0.999847695};
    const double cos_theta[90] = {1,
                                  0.99984770, 0.99939083, 0.99862953, 0.997564050, 0.99619470, 0.99452190, 0.992546152, 0.9902681, 0.987688341, 0.98480775,
                                  0.98162718, 0.97814760, 0.97437006, 0.970295726, 0.96592583, 0.961261696, 0.956304756, 0.95105652, 0.945518576, 0.93969262,
                                  0.93358043, 0.92718385, 0.92050485, 0.913545458, 0.90630779, 0.898794046, 0.891006524, 0.88294759, 0.874619707, 0.86602540,
                                  0.85716730, 0.84804810, 0.83867057, 0.829037573, 0.81915204, 0.809016994, 0.798635510, 0.78801075, 0.777145961, 0.76604444,
                                  0.75470958, 0.74314483, 0.73135370, 0.719339800, 0.70710678, 0.694658370, 0.681998360, 0.66913061, 0.656059029, 0.64278761,
                                  0.62932039, 0.61566148, 0.60181502, 0.587785252, 0.57357644, 0.559192903, 0.544639035, 0.52991926, 0.515038075, 0.5,
                                  0.48480962, 0.46947156, 0.4539905, 0.438371147, 0.42261826, 0.406736643, 0.390731128, 0.37460659, 0.358367950, 0.34202014,
                                  0.32556815, 0.30901699, 0.2923717, 0.275637356, 0.25881905, 0.241921896, 0.224951054, 0.20791169, 0.190808995, 0.17364818,
                                  0.15643447, 0.13917310, 0.12186934, 0.104528463, 0.08715574, 0.069756474, 0.052335956, 0.03489950, 0.017452406};
    std::vector<Eigen::Vector2d> vertex_pts_;
};

} // namespace robosense