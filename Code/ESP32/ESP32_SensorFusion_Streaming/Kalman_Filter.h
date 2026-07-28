// #ifndef KALMAN_FILTER_H
// #define KALMAN_FILTER_H

// // Constrain Eigen to no dynamic allocation — catches accidental heap use at compile/runtime
// #define EIGEN_NO_MALLOC
// #define EIGEN_MPL2_ONLY
// #include <Eigen/Dense> //  depending on your Eigen-for-Arduino setup

// class AhrsEkf {
// public:
//     AhrsEkf();

//     // Call once at startup with an initial guess (e.g. from a static accel/mag reading)
//     void init(float roll0, float pitch0, float yaw0);

//     // Predict step — call every loop with gyro reading (rad/s) and elapsed time (s)
//     void predict(float gx, float gy, float gz, float dt);

//     // Correction steps — call whenever new measurements are available
//     void updateAccel(float ax, float ay, float az);
//     void updateMag(float mx, float my, float mz);

//     // Accessors
//     float roll()  const { return x_(0); }
//     float pitch() const { return x_(1); }
//     float yaw()   const { return x_(2); }
//     float biasGx() const { return x_(3); }
//     float biasGy() const { return x_(4); }
//     float biasGz() const { return x_(5); }

//     // Tuning — call before init() to override defaults
//     void setProcessNoise(float q_angle, float q_bias);
//     void setAccelNoise(float r_accel);
//     void setMagNoise(float r_mag);

// private:
//     static constexpr int N = 6;  // state size: roll, pitch, yaw, bgx, bgy, bgz

//     Eigen::Matrix<float, N, 1> x_;   // state vector
//     Eigen::Matrix<float, N, N> P_;   // state covariance
//     Eigen::Matrix<float, N, N> Q_;   // process noise covariance

//     float r_accel_ = 0.05f;  // measurement noise variance, tune against your Allan Variance work
//     float r_mag_   = 0.1f;

//     static constexpr float GRAVITY = 9.80665f;

//     void wrapAngle(float& angle); // keep yaw in [-pi, pi]
// };

// #endif