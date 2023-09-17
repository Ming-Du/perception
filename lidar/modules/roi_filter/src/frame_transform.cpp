#include "frame_transform.h"
#include <cmath>

void frame_transform::SetZone(const double zone) {
    zone_ = zone;
}
void frame_transform::SetIsSouthhemi(const bool southhemi) {
    southhemi_ = southhemi;
}
double frame_transform::UtmCentralMeridian(const int zone) {
    const double kSinsDegToRad = 0.01745329252;
    return (-183.0 + (zone * 6.0)) * kSinsDegToRad;
}
/*
 * FootpointLatitude
 *
 * Computes the footpoint latitude for use in converting transverse
 * Mercator coordinates to ellipsoidal coordinates.
 *
 * Reference: Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J.,
 *   GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994.
 *
 * Inputs:
 *   y - The UTM northing coordinate, in meters.
 *
 * Returns:
 *   The footpoint latitude, in radians.
 *
 */
double frame_transform::FootpointLatitude(const double y) {
    /* Precalculate n (Eq. 10.18) */
    double n = (kSmA - kSmB) / (kSmA + kSmB);

    /* Precalculate alpha (Eq. 10.22) */
    /* (Same as alpha in Eq. 10.17) */
    double alpha =
        ((kSmA + kSmB) / 2.0) * (1 + (pow(n, 2.0) / 4) + (pow(n, 4.0) / 64));

    /* Precalculate yy (Eq. 10.23) */
    double yy = y / alpha;

    /* Precalculate beta (Eq. 10.22) */
    double beta = (3.0 * n / 2.0) + (-27.0 * pow(n, 3.0) / 32.0) +
                  (269.0 * pow(n, 5.0) / 512.0);

    /* Precalculate gamma (Eq. 10.22) */
    double gamma = (21.0 * pow(n, 2.0) / 16.0) + (-55.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta (Eq. 10.22) */
    double delta = (151.0 * pow(n, 3.0) / 96.0) + (-417.0 * pow(n, 5.0) / 128.0);

    /* Precalculate epsilon (Eq. 10.22) */
    double epsilon = (1097.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series (Eq. 10.21) */
    double result = yy + (beta * sin(2.0 * yy)) + (gamma * sin(4.0 * yy)) +
                    (delta * sin(6.0 * yy)) + (epsilon * sin(8.0 * yy));

    return result;
}

void frame_transform::MapXYToLatlon(const double x, const double y, const double lambda0,
                                    double &lat, double &lon) {

    /* Get the value of phif, the footpoint latitude. */
    double phif = FootpointLatitude(y);

    /* Precalculate ep2 */
    double ep2 = (pow(kSmA, 2.0) - pow(kSmB, 2.0)) / pow(kSmB, 2.0);

    /* Precalculate cos (phif) */
    double cf = cos(phif);

    /* Precalculate nuf2 */
    double nuf2 = ep2 * pow(cf, 2.0);

    /* Precalculate nf and initialize nfpow */
    double nf = pow(kSmA, 2.0) / (kSmB * sqrt(1 + nuf2));
    double nfpow = nf;

    /* Precalculate tf */
    double tf = tan(phif);
    double tf2 = tf * tf;
    double tf4 = tf2 * tf2;

    /* Precalculate fractional coefficients for x**n in the equations
   below to simplify the expressions for latitude and longitude. */
    double x1frac = 1.0 / (nfpow * cf);

    nfpow *= nf; /* now equals nf**2) */
    double x2frac = tf / (2.0 * nfpow);

    nfpow *= nf; /* now equals nf**3) */
    double x3frac = 1.0 / (6.0 * nfpow * cf);

    nfpow *= nf; /* now equals nf**4) */
    double x4frac = tf / (24.0 * nfpow);

    nfpow *= nf; /* now equals nf**5) */
    double x5frac = 1.0 / (120.0 * nfpow * cf);

    nfpow *= nf; /* now equals nf**6) */
    double x6frac = tf / (720.0 * nfpow);

    nfpow *= nf; /* now equals nf**7) */
    double x7frac = 1.0 / (5040.0 * nfpow * cf);

    nfpow *= nf; /* now equals nf**8) */
    double x8frac = tf / (40320.0 * nfpow);

    /* Precalculate polynomial coefficients for x**n.
   -- x**1 does not have a polynomial coefficient. */
    double x2poly = -1.0 - nuf2;

    double x3poly = -1.0 - 2 * tf2 - nuf2;

    double x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2 -
                    3.0 * (nuf2 * nuf2) - 9.0 * tf2 * (nuf2 * nuf2);

    double x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;

    double x6poly =
        -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 + 162.0 * tf2 * nuf2;

    double x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);

    double x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);

    /* Calculate latitude */
    lat =
        phif + x2frac * x2poly * (x * x) + x4frac * x4poly * pow(x, 4.0) +
        x6frac * x6poly * pow(x, 6.0) + x8frac * x8poly * pow(x, 8.0);

    /* Calculate longitude */
    lon = lambda0 + x1frac * x + x3frac * x3poly * pow(x, 3.0) +
          x5frac * x5poly * pow(x, 5.0) +
          x7frac * x7poly * pow(x, 7.0);
    return;
}

void frame_transform::UtmXYToLatlon(const double x, const double y,
                                    double &lat, double &lon) {
    // bool southhemi_=false;
    const double kUtmScaleFactor = 0.9996;
    double xx = x;
    xx -= 500000.0;
    xx /= kUtmScaleFactor;

    /* If in southern hemisphere, adjust y accordingly. */
    double yy = y;
    if (southhemi_) {
        yy -= 10000000.0;
    }
    yy /= kUtmScaleFactor;

    double cmeridian = UtmCentralMeridian(zone_);
    MapXYToLatlon(xx, yy, cmeridian, lat, lon);
    lat *= RAD_TO_DEG_LOCAL;
    lon *= RAD_TO_DEG_LOCAL;
    return;
}
//--lonlat To utm_xy
void frame_transform::LatlonToUtmXY(const double lon_rad, const double lat_rad, double &x, double &y) {
    int zone = 0;
    zone = static_cast<int>((lon_rad * kSinsRadToDeg + 180) / 6) + 1;

    MaplatlonToXY(lat_rad, lon_rad, UtmCentralMeridian(zone), x, y);

    /* Adjust easting and northing for UTM system. */
    x = x * kUtmScaleFactor + 500000.0;
    y = y * kUtmScaleFactor;
    if (y < 0.0) {
        y += 10000000.0;
    }
    return;
}
double frame_transform::Arclength0fMeridian(const double phi) {
    /* Precalculate n */
    double n = (kSmA - kSmB) / (kSmA + kSmB);

    /* Precalculate alpha */
    double alpha = ((kSmA + kSmB) / 2.0) *
                   (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));

    /* Precalculate beta */
    double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) +
                  (-3.0 * pow(n, 5.0) / 32.0);

    /* Precalculate gamma */
    double gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta */
    double delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);

    /* Precalculate epsilon */
    double epsilon = (315.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series and return */
    double result =
        alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) +
                 (delta * sin(6.0 * phi)) + (epsilon * sin(8.0 * phi)));

    return result;
}
void frame_transform::MaplatlonToXY(const double phi, const double lambda, double lambda0, double &x, double &y) {
    /* Precalculate ep2 */
    double ep2 = (pow(kSmA, 2.0) - pow(kSmB, 2.0)) / pow(kSmB, 2.0);

    /* Precalculate nu2 */
    double nu2 = ep2 * pow(cos(phi), 2.0);

    /* Precalculate nn */
    double nn = pow(kSmA, 2.0) / (kSmB * sqrt(1 + nu2));

    /* Precalculate t */
    double t = tan(phi);
    double t2 = t * t;
    // double tmp = (t2 * t2 * t2) - pow(t, 6.0);

    /* Precalculate l */
    double l = lambda - lambda0;

    /* Precalculate coefficients for l**nn in the equations below
   so a normal human being can read the expressions for easting
   and northing
   -- l**1 and l**2 have coefficients of 1.0 */
    double l3coef = 1.0 - t2 + nu2;

    double l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

    double l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;

    double l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;

    double l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

    double l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    /* Calculate easting (x) */
    x = nn * cos(phi) * l +
        (nn / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) +
        (nn / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) +
        (nn / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

    /* Calculate northing (y) */
    y = Arclength0fMeridian(phi) +
        (t / 2.0 * nn * pow(cos(phi), 2.0) * pow(l, 2.0)) +
        (t / 24.0 * nn * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) +
        (t / 720.0 * nn * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) +
        (t / 40320.0 * nn * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
    return;
}