//
// Created by moriarty on 4/30/21.
//

#ifndef SRC_TRACK_OBJECT_TYPES_H
#define SRC_TRACK_OBJECT_TYPES_H
#include <common/proto/object.pb.h>
#include <map>

namespace perception {
namespace base {
//type, {half_width, half_length}
extern const std::map<perception::ObjectType, std::array<float, 2>> \
 track_object_types;
}//namespace base
}//namespace perception
#endif //SRC_TRACK_OBJECT_TYPES_H
