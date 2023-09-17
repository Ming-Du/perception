#include "pubs/marker_pubs/arrow_marker_pub.h"
#include "common/include/basic_type/rotate_box.h"

namespace robosense {

std::vector<ROS_VISUALIZATION_MARKER> &ArrowMarkerPub::display(const LidarFrameMsg::Ptr &msg_ptr) {

    const auto &obj_vec = msg_ptr->objects;
    if (params_.max_obj_size < obj_vec.size()) {
        params_.max_obj_size = obj_vec.size();
        params_.marker_list.resize(obj_vec.size());
    }

    for (size_t i = 0; i < params_.marker_list.size(); ++i) {
        auto &tmp_maker = params_.marker_list[i];
        tmp_maker.color = params_.default_color_type;
        tmp_maker.scale = params_.default_scale_type;
        tmp_maker.ns = "arrow";
        tmp_maker.id = i;
        tmp_maker.header.frame_id = options_.frame_id;
    }

    for (size_t i = 0; i < obj_vec.size(); ++i) {
        const auto &tmp_obj = obj_vec[i];
        const auto &center = tmp_obj->center;
        const auto &size = tmp_obj->size;
        auto &tmp_marker = params_.marker_list[i];
        drawArrow(tmp_obj, tmp_marker, 1);
    }

    return params_.marker_list;
}

void ArrowMarkerPub::drawArrow(const Object::Ptr &obj, ROS_VISUALIZATION_MARKER &marker, double alpha) {
    marker.type = ROS_VISUALIZATION_MARKER::ARROW;
    marker.action = ROS_VISUALIZATION_MARKER::ADD;

    const auto &center = obj->center;
    const auto &size = obj->size;
    const auto &direction = obj->direction;
    RotateBox box(center, size, direction);
    double box_size = box.volume();
    if (box_size > 0) {
        marker.color.a = alpha;
        marker.color.r = 255;
        marker.color.g = 0;
        marker.color.b = 0;
        marker.pose.position.x = box.center.x();
        marker.pose.position.y = box.center.y();
        marker.pose.position.z = box.center.z();

        marker.scale.x = 3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.1;
        if ((obj->velocity.norm() > 1.38 && obj->type == ObjectType::UNKNOW) || (obj->velocity.norm() > 5.66 && obj->type != ObjectType::UNKNOW)) {
            box.angle = atan2(obj->velocity(1), obj->velocity(0)); // atan2(obj_ptr->direction(1), obj_ptr->direction(0));
        }
        tf::Quaternion quat = tf::createQuaternionFromYaw(box.angle);
        tf::quaternionTFToMsg(quat, marker.pose.orientation);
    } else {
        marker.color.a = 0;
    }
}

} // namespace robosense
