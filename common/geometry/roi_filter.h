/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <memory>
#include <vector>

#include "perception/base/hdmap_struct.h"
#include "perception/base/object.h"
#include "perception/base/point_cloud.h"


namespace perception {
namespace common {

// @brief: whether a point is in ROI.
bool IsPtInRoi(
    const std::shared_ptr<const perception::base::HdmapStruct> roi,
    const perception::base::PointD pt);

// @brief: whether an object's center is in ROI.
bool IsObjectInRoi(
    const std::shared_ptr<const perception::base::HdmapStruct> roi,
    const std::shared_ptr<const perception::base::Object> obj);

// @brief: whether an object's bbox is in ROI.
bool IsObjectBboxInRoi(
    const std::shared_ptr<const perception::base::HdmapStruct> roi,
    const std::shared_ptr<const perception::base::Object> obj);

// @brief: whether objects' center are in ROI. If return True,
//         you can get objects in ROI by param valid_objects.
bool ObjectInRoiCheck(
    const std::shared_ptr<const perception::base::HdmapStruct> roi,
    const std::vector<std::shared_ptr<perception::base::Object>>& objs,
    std::vector<std::shared_ptr<perception::base::Object>>* valid_objs);

}  // namespace common
}  // namespace perception

