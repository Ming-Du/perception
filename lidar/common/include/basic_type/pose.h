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
#ifndef RS_SDK_POSE_H_
#define RS_SDK_POSE_H_

#include <iostream>
#include <sstream>
#include <memory>
#include <limits>
#include <sstream>

namespace robosense{

struct Pose{
    using Ptr = std::shared_ptr<Pose>;

    double x = 0.f, y = 0.f, z = 0.f, roll = 0.f, pitch = 0.f, yaw = 0.f;

    std::string infos() {
        std::ostringstream os;
        os << "{ x: " << x << ", y: " << y << ", z: " << z
           << ", roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << " }"<< std::endl;
        return os.str();
    }
};

inline bool operator==(const Pose& lhs,const Pose& rhs){
    if (std::abs(lhs.x - rhs.x) > std::numeric_limits<double>::epsilon()){
        return false;
    }
    if (std::abs(lhs.y - rhs.y) > std::numeric_limits<double>::epsilon()){
        return false;
    }
    if (std::abs(lhs.z - rhs.z) > std::numeric_limits<double>::epsilon()){
        return false;
    }
    if (std::abs(lhs.roll - rhs.roll) > std::numeric_limits<double>::epsilon()){
        return false;
    }
    if (std::abs(lhs.pitch - rhs.pitch) > std::numeric_limits<double>::epsilon()){
        return false;
    }
    if (std::abs(lhs.yaw - rhs.yaw) > std::numeric_limits<double>::epsilon()){
        return false;
    }
    return true;
}

inline std::ostream& operator << (std::ostream& os, const Pose& p){
    os << "(" << "x:" << p.x
       << "," << "y:" << p.y
       << "," << "z:" << p.z
       << "," << "roll:" << p.roll
       << "," << "pitch:" << p.pitch
       << "," << "yaw:" << p.yaw
       << ")";
    return os;
}


}

#endif  // RS_SDK_POSE_H_
