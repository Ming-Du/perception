#include "event_processor.h"

namespace perception {
namespace fusion {

void EventProcessor::ZombieCarProcessor(TrackedObjects& tracked_objects, double zombie_thr) {
  for (int i = 0; i < tracked_objects.objs_size(); i++) {
    int flag = 0;
    perception::TrackedObject* tracked_object = tracked_objects.mutable_objs(i);
    ::perception::Object* object = tracked_object->mutable_obj();
    double time_offset = object->obu_supplement().status_duration();  // 2min = 120000ms
    if (time_offset > zombie_thr && time_offset < DURATION_DEFAULT) {
      flag = 1;
    } else {
      flag = 0;
    }
    perception::AdditionalAttribute event = event_map.at(flag);
    if (object->add_attribute() == perception::AdditionalAttribute::ATTR_UNKNOWN) {
      object->set_add_attribute(event);
    }
  }
}

int EventProcessor::ZombieCarProcessor(double status_duration, double zombie_thr) {
  int flag = 0;
  // 2min = 120000ms
  if (status_duration > zombie_thr && status_duration < DURATION_DEFAULT) {
    flag = 1;
  } else {
    flag = 0;
  }
  return flag;
}

void EventProcessor::V2nEventProcessor(TrackedObjects& tracked_objects) {
  for (int i = 0; i < tracked_objects.objs_size(); i++) {
    int flag = 0;
    perception::TrackedObject* tracked_object = tracked_objects.mutable_objs(i);
    ::perception::Object* object = tracked_object->mutable_obj();
    bool _has_obu = object->has_obu_supplement();
    if (_has_obu) {
      if (tracked_object->source() == perception::ObjectSource::V2N_RSI &&
          (object->type() == perception::ObjectType::TYPE_ROADWORK_OCCUPY_0501 ||
           object->type() == perception::ObjectType::TYPE_ROADWORK_BREAK_0502)) {
        flag = 2;
      } else if (tracked_object->source() == perception::ObjectSource::V2N_RSM) {
        flag = 3;
      } else if (tracked_object->source() == perception::ObjectSource::V2N_RSI &&
                 object->type() == perception::ObjectType::TYPE_WARNINGTRIANGLE) {
        flag = 4;
      } else {
        flag = 0;
      }
      perception::AdditionalAttribute event = event_map.at(flag);
      if (object->add_attribute() == perception::AdditionalAttribute::ATTR_UNKNOWN) {
        object->set_add_attribute(event);
      }
    }
  }
}

}  // namespace fusion
}  // namespace perception