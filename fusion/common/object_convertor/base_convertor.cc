#include "base_convertor.h"

namespace perception {
namespace fusion {

//  BaseConvertor::~BaseConvertor(){};

bool BaseConvertor::ObjectConvertor(const perception::Object& object, fusion::ObjectPtr& fusion_object) {
    fusion_object->id = object.id();
    fusion_object->track_id = object.id();
    fusion_object->direction << cosf(object.angle()), sinf(object.angle()), 0;
    fusion_object->theta = object.angle();
    fusion_object->center_ego << object.center().x(), object.center().y(), object.center().z();
    fusion_object->size << object.size().x(), object.size().y(), object.size().z();
    if (std::fabs(object.confidence()) > 1 || object.confidence() <= 0.0) {
        fusion_object->confidence = 0.1;
    } else {
        fusion_object->confidence = object.confidence();
    }
    fusion_object->tracking_time = object.tracking_time();
    fusion_object->acceleration << object.acceleration().x(), object.acceleration().y(), object.acceleration().z();
    return true;
}


void BaseConvertor::typeApollo2MogoConvertor(const perception::fusion::ObjectType& apollo_type,
                                             const perception::fusion::ObjectSubType& apollo_subtype,
                                             perception::ObjectType& mogo_type) {
    auto type = fusion::kSubType2TypeMap.find(apollo_subtype);
    switch (apollo_type) {
        case perception::fusion::ObjectType::UNKNOWN:
            mogo_type = perception::ObjectType::TYPE_UNKNOWN;
            break;
        case perception::fusion::ObjectType::PEDESTRIAN:
            mogo_type = perception::ObjectType::TYPE_PEDESTRIAN;
            break;
        case perception::fusion::ObjectType::BICYCLE:
            switch (apollo_subtype) {
            case perception::fusion::ObjectSubType::BICYCLE_RIDER:
                mogo_type = perception::ObjectType::TYPE_RIDER;
                break;
            case perception::fusion::ObjectSubType::RIDER:
                mogo_type = perception::ObjectType::TYPE_RIDER;
                break;
            default:
            mogo_type = perception::ObjectType::TYPE_BICYCLE;
            break;
            }
            break;
        case perception::fusion::ObjectType::MOTOR:
            switch (apollo_subtype)
            {
            case perception::fusion::ObjectSubType::MOTOR_RIDER:
                mogo_type = perception::ObjectType::TYPE_RIDER;
                break;
            case perception::fusion::ObjectSubType::RIDER:
                mogo_type = perception::ObjectType::TYPE_RIDER;
                break;
            default:
            mogo_type = perception::ObjectType::TYPE_MOTOR;
            break;
            }
            break;
        case perception::fusion::ObjectType::CAR:
            mogo_type = perception::ObjectType::TYPE_CAR;
            break;
        case perception::fusion::ObjectType::BUS:
            mogo_type = perception::ObjectType::TYPE_BUS;
            break;
        case perception::fusion::ObjectType::TRUCK:
            mogo_type = perception::ObjectType::TYPE_TRUCK;
            break;
        case perception::fusion::ObjectType::VEGETATION:
            mogo_type = perception::ObjectType::TYPE_VEGETATION;
            break;
        case perception::fusion::ObjectType::TRAFFICCONE:
            switch (apollo_subtype) {
                case perception::fusion::ObjectSubType::WARNINGTRIANGLE:
                    mogo_type = perception::ObjectType::TYPE_WARNINGTRIANGLE;
                    break;
                default:
                    mogo_type = perception::ObjectType::TYPE_TRIANGLEROADBLOCK;
                    break;
            }
            break;
        default:
            mogo_type = perception::ObjectType::TYPE_UNKNOWN;
            break;
    }
    return;
}

float BaseConvertor::ConvertToAngle(float angle, double lon, double lat) {
    double angleOrigin = 450.0 - angle;
    double radianOrigin = DegToRad(angleOrigin);
    double radianLat = DegToRad(lat);
    double radianDeal = atan(tan(radianOrigin) * cos(radianLat));

    double ruler = 270.0;
    if (angle >= 0 && angle <= 180.0) {
        ruler = 90.0;
    } else {
        ruler = 270.0;
    }
    float result = (float)(ruler - RadToDeg(radianDeal));
    return result;
}

std::string BaseConvertor::DecIntToHexStr(long long num) {
    std::string str;
    long long Temp = num / 16;
    int left = num % 16;
    if (Temp > 0)
        str += DecIntToHexStr(Temp);
    if (left < 10)
        str += (left + '0');
    else
        str += ('A' + left - 10);
    return str;
}

std::string BaseConvertor::DecStrToHexStr(std::string str) {
    long long Dec = 0;
    for (int i = 0; i < str.size(); ++i)
        Dec = Dec * 10 + str[i] - '0';  //得到相应的整数：ASCII码中：字符0的码值是48；
    return DecIntToHexStr(Dec);
}

double BaseConvertor::DegToRad(double deg) {
    return (deg / 180.0 * M_PI);
}

double BaseConvertor::RadToDeg(double rad) {
    return (rad / M_PI * 180.0);
}
}  // namespace fusion
}  // namespace perception