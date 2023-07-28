/**
 * @brief Library for navigation transforms and kinematics.
 *
 * @file NavigationTransforms.h
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
#ifndef NAVIGATION_TRANSFORMS_H_
#define NAVIGATION_TRANSFORMS_H_

    #ifndef NAVIGATION_TRANSFORMS_C_CONFIG_API_VERSION
#warning "Add symbol NAVIGATION_TRANSFORMS_C_CONFIG_API_VERSION to platform config file."
#define NAVIGATION_TRANSFORMS_C_CONFIG_API_VERSION (2)
    #endif

/**
 * @brief An attitude matrix.
 */
typedef struct _AttitudeMatrix_s
{
	double matrix[3][3];
}
AttitudeMatrix_t;

/**
 * @brief Data structure for Roll, Pitch, and Yaw attitude angles.
 */
typedef struct _RPYAngles_t
{
    double roll;
    double pitch;
    double yaw;
}
RPYAngles_t;

/**
 * @brief 3-2-1 sequence of Euler Angles (Roll, Pitch, Yaw)
 */
typedef union _Euler321_t
{
    RPYAngles_t angle;
    double array[3];
}
Euler321_t;

/**
 * @brief Geodetic position with respect to the WGS84 reference ellipsoid.
 *
 * - Geodetic latitude [radians]
 * - Longitude [radians]
 * - Height above ellipsoid [meters]
 */
typedef struct _GeodeticPosition_t
{
    double lat;
    double lon;
    double alt;
}
GeodeticPosition_t;

/**
 * @brief A Cartesian vector expressed in X, Y, Z components.
 */
typedef struct _CartesianFrame_t
{
    double x;
    double y;
    double z;
}
CartesianFrame_t;

/**
 * @brief A Cartesian vector expressed as both an array and X, Y, Z components.
 */
typedef union _CartesianVector_t
{
    CartesianFrame_t axis;
    double array[3];
}
CartesianVector_t;

/**
 * @brief A Cartesian vector expressed in X, Y, Z components (single-precision).
 */
typedef struct _CartesianFrame32_t
{
    float x;
    float y;
    float z;
}
CartesianFrame32_t;

/**
 * @brief A Cartesian vector expressed as both an array and X, Y, Z components
 *        (single presision).
 */
typedef union _CartesianVector32_t
{
    CartesianFrame32_t axis;
    float array[3];
}
CartesianVector32_t;

/**
 * @brief Position, Velocity, Attitude (PVA) state of an ECEF navigator.
 */
typedef struct _ECEFState_t
{
    CartesianVector_t r_eb_e;
    CartesianVector_t v_eb_e;
    AttitudeMatrix_t C_b_e;
}
ECEFState_t;

/**
 * @brief Position, Velocity, Attitude (PVA) state of an NED navigator.
 */
typedef struct _NEDState_t
{
    GeodeticPosition_t origin;
    CartesianVector_t r_lb_l;
    CartesianVector_t v_lb_l;
    AttitudeMatrix_t C_l_b;
}
NEDState_t;

/**
 * @brief Method for transposition of an NEDState_t PVA navigation solution.
 *
 * Changes the "object frame" and re-expresses the position, velocity, and
 * attitude in this new frame.
 *
 * @param [in] omega_lb_b - Rotation rate of the input frame, with respect to
 *             tangent plane axes, resolved in input frame axes.
 * @param [in] lever_arm_ob_o - Vector from origin of output frame to origin
 *             of input frame, expressed in output frame axes.
 * @param [in] output_to_input - Rotation matrix from output to input frame.
 * @param [in] input - The reference NEDState_t (PVA state to transfer).
 * @param [out] output - The NEDState_t transposed to the output frame.
 */
void NEDState_transferObjectFrame(const CartesianVector_t * omega_lb_b,
                                  const CartesianVector_t * lever_arm_ob_o,
                                  const AttitudeMatrix_t * output_to_input,
                                  const NEDState_t * input,
                                  NEDState_t * output);

/**
 * @brief Method to obtain the rotation matrix that rotates from ECEF to
 *        local-level North, East, Down axes.
 *
 * @param llh Pointer to the latitude, longitude, altitude reference
 *            position. [rad, rad, m].
 * @param ecef2ned The rotation matrix that rotates from ECEF to NED.
 */
void getResolvingMatrixNED(const GeodeticPosition_t * llh,
                           AttitudeMatrix_t * ecef2ned);

/**
 * @brief Method to obtain the rotation matrix that rotates from local-level
 *        North, East, Down axes to ECEF axes.
 *
 * @param llh Pointer to the latitude, longitude, altitude reference
 *            position. [rad, rad, m].
 * @param ned2ecef The rotation matrix that rotates from NED to ECEF.
 */
void getResolvingMatrixECEF(const GeodeticPosition_t * llh,
                            AttitudeMatrix_t * ned2ecef);

/**
 * @brief Method to convert ECEF coordinates to geodetic latitude, longitude,
 *        and altitude (height above WGS84 reference ellipsoid).
 *
 * This method uses an iterative algorithm to converge the latitude and altitude
 * solution.
 *
 * @param [in] r_eb_e The ECEF position [meters].
 * @param [out] llh The geodetic position in [rad, rad, m].
 * @return The number of iterations required by the coordinate transformation.
 */
int getPosition_ecef2llh(const CartesianVector_t * r_eb_e,
                          GeodeticPosition_t * llh);

/**
 * @brief Method to convert geodetic latitude, longitude, and altitude to ECEF
 *        position.
 *
 * @param [in] llh Geodetic latitude, longitude, and altitude (height above
 *                 WGS84 reference ellipsoid). [rad, rad, m].
 * @param [out] r_eb_e The ECEF position [meters].
 */
void getPosition_llh2ecef(const GeodeticPosition_t * llh,
                          CartesianVector_t * r_eb_e);

/**
 * @brief Method to convert ECEF coordinates to NED tangent plane coordinates.
 *
 * @param [in] r_eb_e The ECEF position [meters].
 * @param [in] origin The tangent plane origin [rad, rad, m].
 * @param [out] r_lb_l The NED tangent plane position [meters].
 */
void getPosition_ecef2topo(const CartesianVector_t * r_eb_e,
                           const GeodeticPosition_t * origin,
                           CartesianVector_t * r_lb_l);

/**
 * @brief Method to convert NED tangent plane coordinates to ECEF coordinates.
 *
 * @param [in] r_lb_l The NED tangent plane position [meters].
 * @param [in] origin The tangent plane origin [rad, rad, m].
 * @param [out] r_eb_e The ECEF position [meters].
 */
void getPosition_topo2ecef(const CartesianVector_t * r_lb_l,
                           const GeodeticPosition_t * origin,
                           CartesianVector_t * r_eb_e);

/**
 * @brief Method to convert geodetic latitude, longitude, and altitude to NED
 *        tangent plane coordinates.
 *
 * @param [in] llh Geodetic latitude, longitude, and altitude (height above
 *                 WGS84 reference ellipsoid). [rad, rad, m].
 * @param [in] origin The tangent plane origin [rad, rad, m].
 * @param [out] r_lb_l The NED tangent plane position [meters].
 */
void getPosition_llh2topo(const GeodeticPosition_t * llh,
                          const GeodeticPosition_t * origin,
                          CartesianVector_t * r_lb_l);

/**
 * @brief Method to convert geodetic latitude, longitude, and altitude to NED
 *        tangent plane coordinates.
 *
 * @param [in] r_lb_l The NED tangent plane position [meters].
 * @param [in] origin The tangent plane origin [rad, rad, m].
 * @param [out] llh Geodetic latitude, longitude, and altitude (height above
 *                  WGS84 reference ellipsoid). [rad, rad, m].
 */
void getPosition_topo2llh(const CartesianVector_t * r_lb_l,
                          const GeodeticPosition_t * origin,
                          GeodeticPosition_t * llh);

/**
 * @brief Method to convert NED Position, Velocity, and Attitude to ECEF PVA.
 *
 * @param [in] topo The NED PVA state.
 * @param [out] ecef The ECEF PVA state.
 */
void topo2ecef(const NEDState_t * topo,
               ECEFState_t * ecef);

/**
 * @brief Method to convert ECEF Position, Velocity, and Attitude to NED PVA.
 *
 * @param [in] ecef The ECEF PVA state.
 * @param [in] lat0 The reference latitude of the topocentric origin [radians].
 * @param [in] lon0 The reference longitude of the topocentric origin [radians].
 * @param [in] alt0 The reference altitude of the topocentric origin [meters].
 * @param [out] topo The NED PVA state.
 */
void ecef2topo(const ECEFState_t * ecef,
               double lat0,
               double lon0,
               double alt0,
               NEDState_t * topo);

/**
 * @brief Method to evaluate the gravity model resolved in NED axes.
 *
 * Uses a Zonal Spherical Harmonic expansion up to third order in the Legendre
 * polynomial. The model is parameterized to avoid numerical singularities at
 * the poles. Additionally, for heights below the surface of the WGS84 reference
 * ellipsoid (i.e. negative altitude), the gravity model returns a scaled value
 * of surface gravity. The gravity model includes the centripetal acceleration
 * of the Earth and corresponds to the direction in which a plum-bob would align
 * when stationary on the Earth.
 *
 * @param [in] llh The geodetic position [rad, rad, m] used to evaluate the
 *                 gravity model.
 * @param [out] g_eb_l The gravity vector resolved in NED components. [m/s^2].
 */
void gravityNED(const GeodeticPosition_t * llh,
                CartesianVector_t * g_eb_l);

/**
 * @brief Method to evaluate the gravity model resolved in ECEF axes.
 *
 * This method is a wrapper around gravityNED, and resolves the output in
 * ECEF axes. The input position, r_eb_e, is converted to the equivalent
 * geodetic position for gravity model evaluation.
 *
 * @param [in] r_eb_e The ECEF position used to evaluate the gravity model.
 *                    [meters].
 * @param [out] g_eb_e The gravity vector resolved in ECEF components. [m/s^2].
 */
void gravityECEF(const CartesianVector_t * r_eb_e,
                 CartesianVector_t * g_eb_e);

/**
 * @brief Convert a rotation vector to a rotation matrix.
 *
 * Approximates the matrix exponential via a truncated power series for sine
 * and cosine via the Rodrigues Formula.
 *
 * @param [in] vector The rotation vector (radians).
 * @param [out] result The equivalent attitude matrix.
 */
void rotationVectorToRotationMatrix(const CartesianVector_t * vector,
                                    AttitudeMatrix_t * result);

/**
 * @brief Method to convert 3-2-1 sequence of Euler angles to a rotation matrix.
 *
 * @param [in] rpy 3-2-1 sequence of Euler angles. [radians].
 * @param [out] rm The equivalent attitude matrix (rotation matrix) rotating
 *                 from NED axes to body axes. Assumes NED-like body axes.
 */
void euler321ToRotationMatrix(const Euler321_t * rpy,
                              AttitudeMatrix_t * rm);

/**
 * @brief Method to convert a rotation matrix to the equivalent set of 3-2-1
 *        Euler angles.
 *
 * The attitude matrix is both unique and globally non-singular. The Euler
 * angles, however, have a "gimbal lock" singularity at a pitch angle of +/-
 * 90 degrees. In this case, the Roll and Yaw axes are aligned and only their
 * linear combination (sum or difference) can be determined from the attitude
 * matrix.
 *
 * This method always clamps the Yaw angle to zero in gimbal lock and puts all
 * of the angle magnitude into the Roll angle output. As a result, the Roll and
 * Yaw angles may exhibit step discontinuities when passing through gimbal lock.
 *
 * The return flag is used to indicate if the system is in gimbal lock.
 *
 * @param [in] rm The input rotation matrix which rotates from NED to body axes.
 *                Assumes NED-like body axes.
 * @param [out] rpy The equivalent set of 3-2-1 Euler angles [radians].
 * @return 1 On gimbal lock (pitch >= 89.7 degrees)
 * @return 0 Otherwise
 */
int rotationMatrixToEuler321(const AttitudeMatrix_t * rm,
                             Euler321_t * rpy);

/**
 * @brief Method to wrap inputs on the interval [0, 2 pi).
 *
 * @param [in] x The angle to wrap [radians].
 * @return The wrapped value [radians].
 */
double wrapTo2Pi(double x);

/**
 * @brief Method to wrap inputs on the interval [-pi, pi).
 *
 * @param [in] x The angle to wrap [radians].
 * @return The wrapped value [radians].
 */
double wrapToPi(double x);

/**
 * @brief Method to wrap inputs on the interval [-pi/2, pi/2].
 *
 * @param [in] x The angle to wrap [radians].
 * @return The wrapped value [radians].
 */
double wrapToPiOn2(double x);

/**
 * @brief Method to compute the L-2 norm of a 3-element Cartesian vector.
 *
 * @param [in] vector The vector whose magnitude we wish to calculate.
 * @return The vector magnitude.
 */
double getVectorMagnitude(const CartesianVector_t * vector);

/**
 * @brief Method to convert geodetic latitude to geocentric latitude.
 *
 * @param [in] lat Geodetic latitude [radians].
 * @param [in] alt Height above the WGS84 reference ellipsoid [meters].
 * @return The geocentric latitude [radians].
 */
double getGeocentricLatitude(double lat, double alt);

/**
 * @brief Method to initialize a geodetic position vector.
 *
 * @param [out] target The llh vector to be initialized.
 * @param [in] latitude Reference latitude [radians].
 * @param [in] longitude Reference longitude [radians].
 * @param [in] altitude Reference altitude (HAE) [meters].
 */
void setGeodeticPosition(GeodeticPosition_t * target,
                         double latitude,
                         double longitude,
                         double altitude);

/**
 * @brief Method to initialize an attitude matrix.
 *
 * @param [out] target The attitude matrix to be initialized.
 * @param [in] c11 Row 1, column 1 coefficient of the attitude matrix.
 * @param [in] c12 Row 1, column 2 coefficient of the attitude matrix.
 * @param [in] c13 Row 1, column 3 coefficient of the attitude matrix.
 * @param [in] c21 Row 2, column 1 coefficient of the attitude matrix.
 * @param [in] c22 Row 2, column 2 coefficient of the attitude matrix.
 * @param [in] c23 Row 2, column 3 coefficient of the attitude matrix.
 * @param [in] c31 Row 3, column 1 coefficient of the attitude matrix.
 * @param [in] c32 Row 3, column 2 coefficient of the attitude matrix.
 * @param [in] c33 Row 3, column 3 coefficient of the attitude matrix.
 */
void setAttitudeMatrix(AttitudeMatrix_t * target,
                       double c11,
                       double c12,
                       double c13,
                       double c21,
                       double c22,
                       double c23,
                       double c31,
                       double c32,
                       double c33);

/**
 * @brief Method to initialize a vector of 3-2-1 Euler angles.
 *
 * @param [out] rpy The vector to be initialized.
 * @param roll Reference roll angle.
 * @param pitch Reference pitch angle.
 * @param yaw Reference yaw angle.
 */
void setEuler321(Euler321_t * rpy,
                 double roll,
                 double pitch,
                 double yaw);

/**
 * @brief Method to initialize a Cartesian vector.
 *
 * @param [out] target The vector to be initialized.
 * @param [in] x Reference x-axis value.
 * @param [in] y Reference y-axis value.
 * @param [in] z Reference z-axis value.
 */
void setVector(CartesianVector_t * target,
               double x,
               double y,
               double z);

/**
 * @brief Method to copy an instance of a rotation matrix.
 *
 * @param [out] target The attitude matrix to be initialized.
 * @param [in] source The reference attitude matrix.
 */
#define copyAttitudeMatrix(target, source) *(target) = *(source)

/**
 * @brief Method to copy the transpose of a rotation matrix.
 *
 * @param [out] target The transposed attitude matrix.
 * @param [in] source The reference attitude matrix.
 */
void copyAttitudeMatrixTranspose(AttitudeMatrix_t * target,
                                 const AttitudeMatrix_t * source);

/**
 * @brief Method to copy an instance of a Cartesian vector.
 *
 * @param [out] target The vector to be initialized.
 * @param [in] source The reference vector.
 */
#define copyVector(target, source) *(target) = *(source);

/**
 * @brief Method to ortho-normalize a rotation matrix.
 *
 * Restores the unitary constraint of a rotation matrix (rows and columns are
 * orthogonal unit vectors, determinant == 1). The unitary constraint will
 * degrade over time due to accumulated numerical roundoff error. This method
 * "squares up" the orthogonality of the rows and columns and normalizes each
 * row and column to restore the unit norm constraint.
 *
 * @param [in,out] matrix The attitude matrix to be ortho-normalized.
 */
void squareUpRotationMatrix(AttitudeMatrix_t * matrix);

/**
 * @brief Method to multiply two rotation matrices
 *
 * @param [in] left The matrix left of the multiplication operator.
 * @param [in] right The matrix right of the multiplication operator.
 * @param [out] result The matrix product.
 */
void multiplyRotationMatrix(const AttitudeMatrix_t * left,
                            const AttitudeMatrix_t * right,
                            AttitudeMatrix_t * result);

/**
 * @brief Helper method to multiply a vector by a rotation matrix.
 */
void rotateVector(const AttitudeMatrix_t * rm,
                  const CartesianVector_t * vector,
                  CartesianVector_t * result);

/**
 * @brief Helper method to compute the dot product between two vectors.
 */
double dotVector(const CartesianVector_t * x,
                 const CartesianVector_t * y);

/**
 * @brief Helper method to compute the cross product between two vectors.
 */
void crossVector(const CartesianVector_t * x,
                 const CartesianVector_t * y,
                 CartesianVector_t * result);

/**
 * @brief Helper method to compute the sum of two vectors.
 */
void addVector(const CartesianVector_t * x,
               const CartesianVector_t * y,
               CartesianVector_t * result);

/**
 * @brief Helper method to compute the difference between two vectors.
 */
void diffVector(const CartesianVector_t * x,
                const CartesianVector_t * y,
                CartesianVector_t * result);

/**
 * @brief Helper method to multiply a vector by a scalar.
 */
void multVector(double scalar,
                const CartesianVector_t * vector,
                CartesianVector_t * result);

/**
 * @brief Helper method to convert a vector to a skew-symmetric matrix.
 */
void skewVector(const CartesianVector_t * vector,
                AttitudeMatrix_t * result);

//------------------------------------------------------------------------------
// These methods are deprecated and will be removed in a future release.
//------------------------------------------------------------------------------

    #if (NAVIGATION_TRANSFORMS_C_CONFIG_API_VERSION < 2)

/**
 * @note DEPRECATED - To be removed in a future release.
 *       Replaced by getResolvingMatrixNED()
 *
 * @brief Method to obtain the rotation matrix that rotates from ECEF to
 *        local-level North, East, Down axes.
 *
 * @param llh Pointer to the latitude, longitude, altitude reference
 *            position. [rad, rad, m].
 * @param ecef2ned The rotation matrix that rotates from ECEF to NED.
 */
void getNEDResolvingMatrix(const GeodeticPosition_t * llh,
                           AttitudeMatrix_t * ecef2ned);

/**
 * @note DEPRECATED - To be removed in a future release.
 *       Replaced by getPosition_ecef2llh()
 *
 * @brief Method to convert ECEF coordinates to geodetic latitude, longitude,
 *        and altitude (height above WGS84 reference ellipsoid).
 *
 * This method uses an iterative algorithm to converge the latitude and altitude
 * solution.
 *
 * @param [in] r_eb_e The ECEF position [meters].
 * @param [out] llh The geodetic position in [rad, rad, m].
 * @return The number of iterations required by the coordinate transformation.
 */
int getGeodeticPosition(const CartesianVector_t * r_eb_e,
                        GeodeticPosition_t * llh);

/**
 * @note DEPRECATED - To be removed in a future release.
 *       Replaced by getPosition_llh2ecef
 *
 * @brief Method to convert geodetic latitude, longitude, and altitude to ECEF
 *        position.
 *
 * @param [in] llh Geodetic latitude, longitude, and altitude (height above
 *                 WGS84 reference ellipsoid). [rad, rad, m].
 * @param [out] r_eb_e The ECEF position [meters].
 */
void getECEFPosition(const GeodeticPosition_t * llh,
                     CartesianVector_t * r_eb_e);

/**
 * @note DEPRECATED - To be removed in a future release.
 *       Replaced by getPosition_topo2llh
 *
 * @brief Method to obtain the geodetic position from a reference llh origin and
 *        Cartesian displacement resolved in North, East, Down components.
 *
 * Note that the fields v\_lb\_l and C\_l\_b in the NEDSTate\_t data structure
 * are not touched by this function.
 *
 * @param [in] topo The NED state containing the llh origin [rad, rad, m] and
 *                  relative displacement in tangent frame axes [meters].
 * @param [out] llh The geodetic position [rad, rad, m] corresponding to the
 *                  position of topo.r_lb_l.
 */
void getTopoPosition(const NEDState_t * topo,
                     GeodeticPosition_t * llh);

    #endif

#endif  // NAVIGATION_TRANSFORMS_H_
