/* constants.h */

#ifndef __constants_h__
#define __constants_h__

/*                 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862 */
#define PI_long   (3.1415926535897932384626433832795028841972L)
#define PI_double (3.141592653589793238463)
#define PI_float  (3.141592653589793238463f)

#define DEG2RAD_double (PI_double / 180.0)
#define RAD2DEG_double (180.0 / PI_double)

#define DEG2RAD_float (PI_float / 180.0f)
#define RAD2DEG_float (180.0f / PI_float)

#define DEG2RAD_long (PI_long / 180.0L)
#define RAD2DEG_long (180.0L / PI_long)

#if defined(DEG2RAD) || defined(RAD2DEG)
#error "DEG2RAD or RAD2DEG already defined outside constants.h"
#endif 

#if defined(CONSTANTS_CONFIG_DEG2RAD_IS_LONG)

#define DEG2RAD DEG2RAD_long
#define RAD2DEG RAD2DEG_long

#elif defined(CONSTANTS_CONFIG_DEG2RAD_IS_DOUBLE)

#define DEG2RAD DEG2RAD_double
#define RAD2DEG RAD2DEG_double

#elif defined(CONSTANTS_CONFIG_DEG2RAD_IS_FLOAT)

#define DEG2RAD DEG2RAD_float
#define RAD2DEG RAD2DEG_float

#else

// Here, we intentionally define a DEG2RAD with a symbol that is likely to cause a conflict
// if DEG2RAD is defined elsewhere in the platform, or likely to produce an error if 
// DEG2RAD is used without having set CONSTANTS_CONFIG_DEG2RAD_IS_...

#define DEG2RAD See_DEG2RAD_Error_In_constants_h
#define RAD2DEG See_DEG2RAD_Error_In_constants_h

#endif

// Evaluate the gravity model at the location of the IMU calibration stand:
// 
//      LLA = {43�38'29.5"N, 72�15'15.2"W, 189 meters}
//      
//      GeodeticPosition_t lla;
//      CartesianVector_t g_lb_l;
//  
//      lla.lat =  (43.0 + (38.0 / 60.0) + (29.5 / 3600.0)) * (M_PI / 180.0);
//      lla.lon = -(72.0 + (15.0 / 60.0) + (15.2 / 3600.0)) * (M_PI / 180.0);
//      lla.alt = 189.0;
//      
//      gravityNED(&lla, &g_lb_l);
//      REFERENCE_GRAVITY = getVectorMagnitude(&g_lb_l);
//      
#define REFERENCE_GRAVITY 9.8044128861895583071373039274476468563079833984375

#endif 
