#ifndef UTM2LONLAT_H_
#define UTM2LONLAT_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdio>
#include <iostream>

class frame_transform {
public:
    frame_transform(){};
    ~frame_transform(){};

    void SetZone(const double zone);
    void UtmXYToLatlon(const double x, const double y, double &lat, double &lon);
    void LatlonToUtmXY(const double lon_rad, const double lat_rad, double &x, double &y);

private:
    void SetIsSouthhemi(const bool southhemi);
    double UtmCentralMeridian(const int zone);
    double FootpointLatitude(const double y);
    void MapXYToLatlon(const double x, const double y, const double lambda0, double &lat, double &lon);

    double Arclength0fMeridian(const double phi);
    void MaplatlonToXY(const double phi, const double lambda, double lambda0, double &x, double &y);

private:
    int zone_ = 0;
    bool southhemi_ = false;
    const double kSinsRadToDeg = 57.295779513;
    const double kUtmScaleFactor = 0.9996;
    const double kSmA = 6378137.0;
    const double kSmB = 6356752.31425;
    const double RAD_TO_DEG_LOCAL = 180.0 / M_PI;
};
#endif
