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
#ifndef RS_COMMON_BASIC_TYPE_RANGE_H_
#define RS_COMMON_BASIC_TYPE_RANGE_H_

#include "rs_util.h"
namespace robosense{

//==================================
//Range3D
//==================================
class alignas(16) Range3D {
public:
    Range3D() = default;

    Range3D(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max){
        this->xmin = x_min;
        this->xmax = x_max;
        this->ymin = y_min;
        this->ymax = y_max;
        this->zmin = z_min;
        this->zmax = z_max;
    }

    std::string infos() {
        std::ostringstream os;
        os << "{ x: (" << xmin << ", " << xmax << "); "
           << "y: (" << ymin << ", " << ymax << "); "
           << "z: (" << zmin << ", " << zmax << ") }" << std::endl;
        return os.str();
    }

    Range3D(const Range3D &r){
        *this = r;
    }

    bool isValid() const{
        return (xmin < xmax && ymin < ymax && zmin < zmax);
    }

    bool inRange(const double& x,const double& y,const double& z){
        return (x > xmin) && (x < xmax) && (y > ymin) && (y < ymax) && (z > zmin) && (z < zmax);
    }

    bool inRange2D(const double& x,const double& y){
        return (x > xmin) && (x < xmax) && (y > ymin) && (y < ymax);
    }

    double xmin = 0, xmax = 0, ymin = 0, ymax = 0, zmin = 0, zmax = 0;
};


}



#endif  // RS_COMMON_BASIC_TYPE_RANGE_H_
