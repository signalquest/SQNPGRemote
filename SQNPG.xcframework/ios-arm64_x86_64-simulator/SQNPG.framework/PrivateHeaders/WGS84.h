/**
 * @brief World Geodetic System 1984 (WGS84) model parameters.
 *
 * @file WGS84.h
 *
 * @copyright Copyright &copy; 2017 %SignalQuest, LLC. All rights reserved.
 *
 * @par Notice
 * This software is the intellectual property of %SignalQuest, LLC. All source
 * code is PROPRIETARY and CONFIDENTIAL and has not been approved for external
 * release to any customer, individual, or other entity. All software that
 * includes this file must be encrypted and code locked when deployed in a
 * customer-facing application.
 */
#ifndef WGS84_H_
#define WGS84_H_

/**
 * @brief WGS84 Magnitude of Earth's equatorial radius \f$ [m] \f$.
 */
#define WGS84_EARTH_RADIUS (6378137.0)

/**
 * @brief WGS84 Magnitude of Earth's polar radius \f$ [m] \f$.
 */
#define WGS84_POLAR_RADIUS (6356752.31425)

/**
 * @brief WGS84 Magnitude of Earth's equatorial radius \f$ [m^{3}\ s^{-2}] \f$.
 */
#define WGS84_GRAVITY_MU (3.986004418e14)

/**
 * @brief WGS84 Second zonal harmonic of gravitation [unitless].
 */
#define WGS84_GRAVITY_J2 (1.082627e-3)

/**
 * @brief WGS84 Third zonal harmonic of gravitation [unitless].
 */
#define WGS84_GRAVITY_J3 (-2.5327e-6)

/**
 * @brief WGS84 flattening parameter of the reference ellipsoid.
 */
#define WGS84_FLATTENING (1.0 / 298.257223563)

/**
 * @brief WGS84 eccentricity parameter of the reference ellipsoid.
 */
#define WGS84_ECCENTRICITY (0.0818191908425)

/**
 * @brief WGS84 Speed of Earth rotation \f$ [rad\ s^{-1}] \f$.
 */
#define WGS84_OMEGA_IE (7.292115e-5)

#endif  // WGS84.h
