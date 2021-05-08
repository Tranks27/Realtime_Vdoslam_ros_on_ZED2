#ifndef VDO_ROS_GEONAV_CONSTANTS
#define VDO_ROS_GEONAV_CONSTANTS

#include <cmath>

constexpr double RADIANS_PER_DEGREE = M_PI/180.0;
constexpr double DEGREES_PER_RADIAN = 180.0/M_PI;

// Grid granularity for rounding UTM coordinates to generate MapXY.
constexpr double grid_size = 100000.0;    // 100 km grid

// WGS84 Parameters
#define WGS84_A   6378137.0   // major axis
#define WGS84_B   6356752.31424518  // minor axis
#define WGS84_F   0.0033528107    // ellipsoid flattening
#define WGS84_E   0.0818191908    // first eccentricity
#define WGS84_EP  0.0820944379    // second eccentricity

// UTM Parameters
#define UTM_K0    0.9996               // scale factor
#define UTM_FE    500000.0             // false easting
#define UTM_FN_N  0.0                  // false northing, northern hemisphere
#define UTM_FN_S  10000000.0           // false northing, southern hemisphere
#define UTM_E2    (WGS84_E*WGS84_E)    // e^2
#define UTM_E4    (UTM_E2*UTM_E2)      // e^4
#define UTM_E6    (UTM_E4*UTM_E2)      // e^6
#define UTM_EP2   (UTM_E2/(1-UTM_E2))  // e'^2

#endif