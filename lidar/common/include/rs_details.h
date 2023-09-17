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

#ifndef ZHN_SDK_RS_DETAILS_H
#define ZHN_SDK_RS_DETAILS_H
#include <set>
#include <map>
#include <string>

namespace robosense {

//=========================================
//  RsApplication
//=========================================
enum class RsApplication {
    NONE = 0,
    PSERIES = 1,
    V2R = 2,
    SMARTSENSOR = 3,
};

/**
 * RsApplication and mapping
 */
const std::set<RsApplication> kRsApplicationSet = {
RsApplication::NONE,
RsApplication::PSERIES,
RsApplication::V2R,
RsApplication::SMARTSENSOR,
};

const std::map<RsApplication, std::string> kRsApplication2NameMap = {
{RsApplication::NONE, "NONE"},
{RsApplication::PSERIES, "PseriesPerception"},
{RsApplication::V2R, "V2rPerception"},
{RsApplication::SMARTSENSOR, "SmartsensorPerception"},
};

const std::map<std::string, RsApplication> kRsApplicationName2TypeMap = {
{"NONE", RsApplication::NONE},
{"PseriesPerception", RsApplication::PSERIES},
{"V2rPerception", RsApplication::V2R},
{"SmartsensorPerception", RsApplication::SMARTSENSOR},
};
//=========================================

//=========================================
//  RsLidarType
//=========================================
enum class RsLidarType {
    RS16 = 0,
    RS32 = 1,
    RS80 = 2,
    RS128 = 3,
    RSBP = 4,
    RSM1 = 5,
    RSHELIOS = 5,
};
/**
 * RsLidarType and mapping
 */
const std::set<RsLidarType> kRsLidarTypeSet = {
RsLidarType::RS16,
RsLidarType::RS32,
RsLidarType::RS80,
RsLidarType::RS128,
RsLidarType::RSBP,
RsLidarType::RSM1,
RsLidarType::RSHELIOS,
};

const std::map<RsLidarType, std::string> kRsLidarType2NameMap = {
{RsLidarType::RS16, "RS16"},
{RsLidarType::RS32, "RS32"},
{RsLidarType::RS80, "RS80"},
{RsLidarType::RS128, "RS128"},
{RsLidarType::RSBP, "RSBP"},
{RsLidarType::RSM1, "RSM1"},
{RsLidarType::RSHELIOS, "RSHELIOS"},
};

const std::map<std::string, RsLidarType> kRsLidarTypeName2TypeMap = {
{"RS16", RsLidarType::RS16},
{"RS32", RsLidarType::RS32},
{"RS80", RsLidarType::RS80},
{"RS128", RsLidarType::RS128},
{"RSBP", RsLidarType::RSBP},
{"RSM1", RsLidarType::RSM1},
{"RSHELIOS", RsLidarType::RSHELIOS},
};
//=========================================
}

#endif //ZHN_SDK_RS_DETAILS_H
