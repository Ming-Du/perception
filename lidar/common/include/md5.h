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

#ifndef RS_PERCEPTION_MD5_H_
#define RS_PERCEPTION_MD5_H_

#include <iostream>
#include <cstring>
#include <cstdio>

namespace robosense {

#define RS_MD5_BLOCK_SIZE 64

typedef unsigned int uInt;  // 32-bit
typedef unsigned char uChar;  // 8-bit(Byte)

class RsMD5 {
public:
    RsMD5();
    explicit RsMD5(const std::string&);
    void update(const uChar*, uInt);
    void update(const char*, uInt);
    RsMD5& finalize();
    std::string hexdigest();
    friend std::ostream& operator << (std::ostream&, RsMD5);
private:
    bool finish;  // Flag to check transformation finished
    uInt count[2];  // 64-bit counter for number of bits(low, high)
    uInt state[4];  // Store A.B.C.D numbers while transforming
    uChar result[16];  // The result of MD5
    uChar buffer[RS_MD5_BLOCK_SIZE];  // Bytes that didn't fit in last 64 byte chunk

    void init();
    void transform(const uChar*);
    void decode(uInt*, const uChar*, uInt);
    void encode(uChar*, const uInt*, uInt);
};

std::string md5(const std::string);
inline uInt F(uInt, uInt, uInt);
inline uInt G(uInt, uInt, uInt);
inline uInt H(uInt, uInt, uInt);
inline uInt I(uInt, uInt, uInt);
inline uInt rotate_left(uInt, int);
inline void FF(uInt&, uInt, uInt, uInt, uInt, uInt, uInt);
inline void GG(uInt&, uInt, uInt, uInt, uInt, uInt, uInt);
inline void HH(uInt&, uInt, uInt, uInt, uInt, uInt, uInt);
inline void II(uInt&, uInt, uInt, uInt, uInt, uInt, uInt);

}  // namespace robosense

#endif  // RS_PERCEPTION_INTERNAL_MD5_H_
