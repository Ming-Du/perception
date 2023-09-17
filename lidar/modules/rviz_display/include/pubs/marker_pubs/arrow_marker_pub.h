#ifndef RS_RVIZ_DISPLAY_PUBS_MARKER_PUB_ARROW_MAKER_PUB_H_
#define RS_RVIZ_DISPLAY_PUBS_MARKER_PUB_ARROW_MAKER_PUB_H_

#include "common/base_marker_pub.h"


namespace robosense{

class ArrowMarkerPub : public BaseMarkerPub {
public:
    using Ptr = std::shared_ptr<ArrowMarkerPub>;

    ArrowMarkerPub() = default;

    virtual void init(const MarkerPubOptions &options) override {
        if (options.node_ptr == nullptr) {
            return;
        }
        options_ = options;
    }

    virtual std::vector<ROS_VISUALIZATION_MARKER> &display(const LidarFrameMsg::Ptr &msg_ptr) override;

    virtual std::string name() override {
        return "ArrowMarkerPub";
    }

private:
    void drawArrow(const Object::Ptr &obj, ROS_VISUALIZATION_MARKER &marker, double alpha = 1.);
};

}

#endif  // RS_RVIZ_DISPLAY_PUBS_MARKER_PUB_ARROW_MAKER_PUB_H_
