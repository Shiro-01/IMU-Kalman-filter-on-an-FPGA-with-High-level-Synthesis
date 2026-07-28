// #include "kalman_filter.h"
// #include <math.h>

// AhrsEkf::AhrsEkf() {
//     x_.setZero();
//     P_.setIdentity();
//     P_ *= 1.0f;  // initial uncertainty — tune based on how confident your init() estimate is

//     Q_.setZero();
//     setProcessNoise(1e-4f, 1e-6f); // sane defaults; override via setProcessNoise before init()
// }

// void AhrsEkf::setProcessNoise(float q_angle, float q_bias) {
//     Q_.setZero();
//     Q_(0,0) = Q_(1,1) = Q_(2,2) = q_angle;
//     Q_(3,3) = Q_(4,4) = Q_(5,5) = q_bias;
// }

// void AhrsEkf::setAccelNoise(float r) { r_accel_ = r; }
// void AhrsEkf::setMagNoise(float r)   { r_mag_ = r; }

// void AhrsEkf::init(float roll0, float pitch0, float yaw0) {
//     x_(0) = roll0;
//     x_(1) = pitch0;
//     x_(2) = yaw0;
//     x_(3) = x_(4) = x_(5) = 0.0f;
// }

// void AhrsEkf::wrapAngle(float& angle) {
//     while (angle > M_PI)  angle -= 2.0f * M_PI;
//     while (angle < -M_PI) angle += 2.0f * M_PI;
// }

// void AhrsEkf::predict(float gx, float gy, float gz, float dt) {
//     float phi   = x_(0);
//     float theta = x_(1);

//     // Bias-corrected body rates
//     float wx = gx - x_(3);
//     float wy = gy - x_(4);
//     float wz = gz - x_(5);

//     float sphi = sinf(phi),   cphi = cosf(phi);
//     float ttheta = tanf(theta), ctheta = cosf(theta);

//     // NOTE: ctheta -> 0 near pitch = ±90 deg. This is the gimbal-lock singularity —
//     // clamp or guard against it rather than letting it blow up.
//     if (fabsf(ctheta) < 1e-3f) ctheta = (ctheta >= 0) ? 1e-3f : -1e-3f;

//     float phi_dot   = wx + wy * sphi * ttheta + wz * cphi * ttheta;
//     float theta_dot = wy * cphi - wz * sphi;
//     float psi_dot   = (wy * sphi + wz * cphi) / ctheta;

//     // Euler-integrate state
//     x_(0) += phi_dot   * dt;
//     x_(1) += theta_dot * dt;
//     x_(2) += psi_dot   * dt;
//     wrapAngle(x_(2));
//     // x_(3..5) (biases) unchanged in predict — random walk, driven only by Q

//     // --- Jacobian F = d(f)/d(x), linearized about current state ---
//     Eigen::Matrix<float, N, N> F;
//     F.setIdentity();

//     float d_phidot_dphi   = wy * cphi * ttheta - wz * sphi * ttheta;
//     float d_phidot_dtheta = (wy * sphi + wz * cphi) * (1.0f + ttheta * ttheta); // d(tan)/dtheta = sec^2
//     float d_thetadot_dphi = -wy * sphi - wz * cphi;
//     float d_psidot_dphi   = (wy * cphi - wz * sphi) / ctheta;
//     float d_psidot_dtheta = (wy * sphi + wz * cphi) * sphi / (ctheta * ctheta); // approx, small-angle friendly

//     F(0,0) += d_phidot_dphi * dt;
//     F(0,1)  = d_phidot_dtheta * dt;
//     F(0,3)  = -dt;                       // d(phi_dot)/d(bgx) = -1
//     F(0,4)  = -sphi * ttheta * dt;
//     F(0,5)  = -cphi * ttheta * dt;

//     F(1,0)  = d_thetadot_dphi * dt;
//     F(1,4)  = -cphi * dt;
//     F(1,5)  = sphi * dt;

//     F(2,0)  = d_psidot_dphi * dt;
//     F(2,1)  = d_psidot_dtheta * dt;
//     F(2,4)  = -sphi / ctheta * dt;
//     F(2,5)  = -cphi / ctheta * dt;

//     // Covariance propagation
//     P_ = F * P_ * F.transpose() + Q_ * dt;
// }

// void AhrsEkf::updateAccel(float ax, float ay, float az) {
//     // Normalize — we only care about direction, not magnitude (rejects linear-accel disturbance somewhat,
//     // though a proper disturbance rejection scheme would gate this update when |a| deviates far from g)
//     float norm = sqrtf(ax*ax + ay*ay + az*az);
//     if (norm < 1e-6f) return;
//     ax /= norm; ay /= norm; az /= norm;

//     float phi = x_(0), theta = x_(1);
//     float sphi = sinf(phi), cphi = cosf(phi);
//     float stheta = sinf(theta), ctheta = cosf(theta);

//     // Predicted gravity direction in body frame from current roll/pitch estimate
//     Eigen::Matrix<float, 2, 1> z_pred;
//     z_pred(0) = stheta;             // expected normalized ax
//     z_pred(1) = -sphi * ctheta;     // expected normalized ay

//     Eigen::Matrix<float, 2, 1> z_meas;
//     z_meas(0) = ax;
//     z_meas(1) = ay;

//     Eigen::Matrix<float, 2, 1> y = z_meas - z_pred; // innovation

//     Eigen::Matrix<float, 2, N> H;
//     H.setZero();
//     H(0,1) = ctheta;                  // d(z_pred0)/d(theta)
//     H(1,0) = -cphi * ctheta;          // d(z_pred1)/d(phi)
//     H(1,1) = sphi * stheta;           // d(z_pred1)/d(theta)

//     Eigen::Matrix<float, 2, 2> R = Eigen::Matrix<float, 2, 2>::Identity() * r_accel_;
//     Eigen::Matrix<float, 2, 2> S = H * P_ * H.transpose() + R;
//     Eigen::Matrix<float, N, 2> K = P_ * H.transpose() * S.inverse();

//     x_ += K * y;
//     wrapAngle(x_(2));
//     P_ = (Eigen::Matrix<float, N, N>::Identity() - K * H) * P_;
// }

// void AhrsEkf::updateMag(float mx, float my, float mz) {
//     float norm = sqrtf(mx*mx + my*my + mz*mz);
//     if (norm < 1e-6f) return;
//     mx /= norm; my /= norm; mz /= norm;

//     float phi = x_(0), theta = x_(1), psi = x_(2);
//     float sphi = sinf(phi), cphi = cosf(phi);
//     float stheta = sinf(theta), ctheta = cosf(theta);

//     // Tilt-compensate the mag reading into the horizontal plane
//     float mx_h =  mx * ctheta + mz * stheta;
//     float my_h =  mx * sphi * stheta + my * cphi - mz * sphi * ctheta;

//     float heading_meas = atan2f(-my_h, mx_h);

//     float y = heading_meas - psi;
//     wrapAngle(y);

//     Eigen::Matrix<float, 1, N> H;
//     H.setZero();
//     H(0,2) = 1.0f;  // measurement is (approximately) direct observation of yaw

//     float R = r_mag_;
//     float S = (H * P_ * H.transpose())(0,0) + R;
//     Eigen::Matrix<float, N, 1> K = P_ * H.transpose() / S;

//     x_ += K * y;
//     wrapAngle(x_(2));
//     P_ = (Eigen::Matrix<float, N, N>::Identity() - K * H) * P_;
// }