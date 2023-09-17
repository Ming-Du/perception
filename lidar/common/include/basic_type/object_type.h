/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#ifndef RS_SDK_OBJECT_TYPE_H_
#define RS_SDK_OBJECT_TYPE_H_

#include <map>
#include <string>

#include "common/proto/object.pb.h"

namespace robosense {

enum class NoiseType {
    IGNORE = 0,
    NOISE = 1,
    OBJECT = 2,
};

enum class AttentionType {
    NONE = 0,
    ATTENTION = 1,
};
enum class ModeType {
    FOCUS = 0,
    BSD = 1,
};
enum class SourceType {
    NONE = 0,
    SINGLE_RB,
    SINGLE_AI,
    AI_RB
};

enum class FilterObjType {
    VALID = 0,
    SUSTECTED_GHOST,
    SUSTECTED_FLOATING,
    SUSTECTED_FLOWERBED
};

enum class ObjectType {
    UNKNOW = 0,
    PED = 1,
    BIC = 2,
    CAR = 3,
    TRUCK = 4,
    BUS = 5,
};

enum class TrackerState {
    VISIBLE = 0,
    INVISIBLE = 1,
};

enum class DetectState {
    DETECT = 0,
    PREDICT = 1,
};

enum class GroupType {
    SINGLE = 0,
    GROUP = 1,
};

enum class MotionState {
    STATIC = 0,
    SUSPECTED_STATIC = 1,
    WORM = 2,
    NORMAL = 3,
};

enum class NoiseState {
    // CONFIRM = 0,
    // SUSPECTED = 1,
    // FLOWERBEDS = 2,
    NOISE_OBJECT = 0,
    NOISE_NOISE = 1,
    NOISE_SUSPECTED = 2,
    NOISE_FLOWERBEDS = 3,
};

enum class RoadType {
    ROAD = 0,
    ROADSIDE = 1,
    FLOWERBEDSSIDE = 2,
    FENCESIDE = 3,
};

enum class AiRefine_State {
    NO_AIREFINE = 0,
    DISCARD_AIREFINE = 1,
    AIREFINE = 2,
};

/**
 * ObjectType and mapping
 */
const std::map<NoiseType, std::string> kNoiseType2NameMap = {
    {NoiseType::IGNORE, "IGNORE"},
    {NoiseType::NOISE, "NOISE"},
    {NoiseType::OBJECT, "OBJECT"},
};

const std::map<ObjectType, std::string> kObjectType2NameMap = {
    {ObjectType::UNKNOW, "UNKNOW"},
    {ObjectType::PED, "PED"},
    {ObjectType::BIC, "BIC"},
    {ObjectType::CAR, "CAR"},
    {ObjectType::TRUCK, "TRUCK"},
    {ObjectType::BUS, "BUS"},

};

const std::map<std::string, ObjectType> kObjectTypeName2TypeMap = {
    {"UNKNOW", ObjectType::UNKNOW},
    {"PED", ObjectType::PED},
    {"BIC", ObjectType::BIC},
    {"CAR", ObjectType::CAR},
    {"TRUCK", ObjectType::TRUCK},
    {"BUS", ObjectType::BUS},

};

const std::map<DetectState, std::string> kDetectState2NameMap = {
    {DetectState::DETECT, "DET"},
    {DetectState::PREDICT, "PRE"},
};

const std::map<GroupType, std::string> kGroupType2NameMap = {
    {GroupType::SINGLE, "Single"},
    {GroupType::GROUP, "Group"},
};

const std::map<NoiseState, std::string> kNoiseState2NameMap = {
    {NoiseState::NOISE_OBJECT, "NOISE_OBJECT"},
    {NoiseState::NOISE_NOISE, "NOISE_NOISE"},
    {NoiseState::NOISE_SUSPECTED, "NOISE_SUSPECTED"},
    {NoiseState::NOISE_FLOWERBEDS, "NOISE_FLOWERBEDS"},
};
const std::map<RoadType, std::string> kRoadType2NameMap = {
    {RoadType::ROAD, "ROAD"},
    {RoadType::ROADSIDE, "ROADSIDE"},
    {RoadType::FLOWERBEDSSIDE, "FLOWERBEDSSIDE"},
    {RoadType::FENCESIDE, "FENCESIDE"},
};

const std::map<ObjectType, perception::ObjectType> kObjectType2MogoType = {
    {ObjectType::UNKNOW, perception::ObjectType::TYPE_UNKNOWN},
    {ObjectType::PED, perception::ObjectType::TYPE_PEDESTRIAN},
    {ObjectType::BIC, perception::ObjectType::TYPE_BICYCLE},
    {ObjectType::CAR, perception::ObjectType::TYPE_CAR},
    {ObjectType::TRUCK, perception::ObjectType::TYPE_TRUCK},
    {ObjectType::BUS, perception::ObjectType::TYPE_BUS},
};    

const std::map<DetectState, perception::DetectState> kDetectState2MogoState = {
    {DetectState::DETECT, perception::DetectState::STATE_DETECT},
    {DetectState::PREDICT, perception::DetectState::STATE_PREDICT},
};  

const std::map<GroupType, perception::GroupType> kGroupType2MogoType = {
    {GroupType::SINGLE, perception::GroupType::TYPE_SINGLE},
    {GroupType::GROUP, perception::GroupType::TYPE_GROUP},
};  

const std::map<NoiseState, perception::NoiseState> kNoiseState2MogoState = {
    {NoiseState::NOISE_OBJECT, perception::NoiseState::NOISE_OBJECT},
    {NoiseState::NOISE_NOISE, perception::NoiseState::NOISE_NOISE},
    {NoiseState::NOISE_SUSPECTED, perception::NoiseState::NOISE_SUSPECTED},
    {NoiseState::NOISE_FLOWERBEDS, perception::NoiseState::NOISE_FLOWERBEDS},
};  

const std::map<RoadType, perception::RoadType> kRoadType2MogoType = {
    {RoadType::ROAD, perception::RoadType::RoadType_ROAD},
    {RoadType::ROADSIDE, perception::RoadType::RoadType_ROADSIDE},
    {RoadType::FLOWERBEDSSIDE, perception::RoadType::RoadType_FLOWERBEDSSIDE},
    {RoadType::FENCESIDE, perception::RoadType::RoadType_FENCESIDE},
};  


} // namespace robosense

#endif // RS_SDK_OBJECT_TYPE_H_
