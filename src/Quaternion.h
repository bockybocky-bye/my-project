#ifndef QUATERNION_H
#define QUATERNION_H

#include <math.h>
#include <stdint.h>

/**
 * Normalize quaternion to unit length.
 * q = q / ||q||
 */
void quat_normalize(float q[4]);

/**
 * Build Ω(ω) matrix (4x4) used in quaternion kinematics:
 * q̇ = 0.5 * Ω(ω) * q
 *
 * w : angular rate [rad/s] in body frame (wx, wy, wz)
 * Om: 4x4 matrix output
 */
void quat_omega_matrix(const float w[3], float Om[4][4]);

/**
 * Quaternion to rotation matrix (body -> navigation).
 *
 * q : [w, x, y, z]
 * R : 3x3 rotation matrix
 */
void quat_to_R(const float q[4], float R[3][3]);

/**
 * Convert Euler angles (roll, pitch, yaw) to quaternion.
 *
 * roll, pitch, yaw : [rad]
 * q                : [w, x, y, z]
 */
void euler_to_quat(float roll, float pitch, float yaw, float q[4]);

/**
 * Integrate quaternion using angular rate.
 *
 * q  : in/out quaternion [w, x, y, z]
 * wx,wy,wz : angular rate [rad/s] in body frame
 * dt : sample time [s]
 */
void quat_update(float q[4], float wx, float wy, float wz, float dt);

/**
 * Predict gravity direction measured in body frame from quaternion.
 *
 * q      : attitude quaternion
 * a_pred : predicted specific force (gravity direction) in body frame
 */
void predict_gravity(const float q[4], float a_pred[3]);

/**
 * Correct quaternion using small rotation vector dtheta.
 *
 * dtheta : small angle error [rad] (3x1)
 * q      : in/out quaternion (q_new = δq ⊗ q_old)
 */
void quat_correct(float q[4], const float dtheta[3]);

/**
 * Convert quaternion to Euler angles roll, pitch, yaw [rad].
 *
 * q     : [w, x, y, z]
 * roll  : output pointer
 * pitch : output pointer
 * yaw   : output pointer
 */
void quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw);

#endif // QUATERNION_H
