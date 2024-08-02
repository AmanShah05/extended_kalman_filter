#ifndef EKF_H_
#define EKF_H_

#include <cmath>
#include "fcu_states.h"

/*

References:
Paper: Multiplicative vs. Additive Filtering for Spacecraft Attitude Determination.
https://ntrs.nasa.gov/api/citations/20040037784/downloads/20040037784.pdf

Paper: A covariance correction step for Kalman filtering with an attitude.
https://arc.aiaa.org/doi/abs/10.2514/1.G000848

Paper: Modelling and Control of Quadrotor
https://lup.lub.lu.se/student-papers/search/publication/8905295

Paper: Time Derivative of Rotation Matrices: A Tutorial
https://arxiv.org/pdf/1609.06088
// The time derivative of a rotation matrix is a skew-symmetric matrix
// and the rotation matrix of itself.
//

*/

#define MAX_COVARIANCE 100.0
#define MIN_COVARIANCE (1e-6f)
#define EPS 1e-6f
#define MAX_ATTITUDE_ERROR 10.0f

// Indexes to access the quad's state, stored as a column vector
/*
 * The internally-estimated states of the kalman filter
 * - X, Y, Z: position in the global rame
 * - PX, PY, PZ: velocity in the global frame
 * - D0, D1, D2: attitude error
*/
enum EKF_STATES {
    EKF_STATE_X,
    EKF_STATE_Y,
    EKF_STATE_Z,
    EKF_STATE_PX,
    EKF_STATE_PY,
    EKF_STATE_PZ,
    EKF_STATE_D0,
    EKF_STATE_D1,
    EKF_STATE_D2,
    EKF_STATE_DIM,
};

typedef struct GpsEkf {
    bool is_gps_new_data;
    bool is_gps_connected;
    bool is_gps_vel_valid;
    bool is_gps_hdt_valid;
    Vector3f gps_velocity;
    Vector3f gps_velocity_std;
    float heading;
    float heading_std;
    float pitch;
    float pitch_std;
} GpsEkfData;


typedef struct OptiflowEkf {
    Vector2f forward;
    float forward_dt;
    bool forward_new_data;
    Vector2f downward;
    float downward_dt;
    bool downward_new_data;
} OptiflowEkfData;


typedef struct LidarEkf {
   float velocity;
   float velocity_std;
} LidarEkfData;


// todo move to command_apis
// These need some statistical analysis to determine the states
typedef struct {
    float std_initial_position_xy = 100;
    float std_initial_position_z = 1;
    float std_initial_velocity = 0.01;
    float std_initial_attitude_rollpitch = 0.01;
    float std_initial_attitude_yaw = 0.01;

    // Todo add the process noise to updating covariance matrix
    float proc_noise_acc_xy = 0.5;
    float proc_noise_acc_z = 1;
    float proc_noise_vel = 0;
    float proc_noise_pos = 0;
    float proc_noise_att = 0;
    float meas_noise_baro = 2.0f;           // meters
    float meas_noise_gyro_rollpitch = 0.1f; // radians per second
    float meas_noise_gyro_yaw = 0.1f;       // radians per second
    float meas_optiflow_x = 0.3f;
    float meas_optiflow_y = 0.3f;

    float initial_x = 0.0f;
    float initial_y = 0.0f;
    float initial_z = 0.0f;
    float initial_yaw = 0.0f;
    float initial_baro_ref_height = 0.0f;
} EKFParameters;

/*
 *
 * Overview of Extended Kalman Filter (EKF) for Attitude Estimation
 *
 * This module implements an attitude estimation algorithm for drone using
 * a Kalman filter approach with a focus on efficient and accurate attitude
 * error handling. The algorithm leverages concepts from the Multiplicative
 * Extended Kalman Filter (MEKF) and a covariance correction step to maintain
 * robust attitude estimation even during large rotations.
 *
 * Algorithm Steps
 *
 * 1. Initialization:
 *  - Initialize the state vector with the drone's initial attitude (as a quaternion)
 *    and other relevant states (e.g., angular velocity, biases).
 *  - Initialize the covariance matrix representing the uncertainty in the initial state.
 *
 * 2. Prediction Step:
 *  - Directly update the drone's attitude based on gyroscope measurements using quaternion
 *    multiplication.
 *  - Compute the attitude error using Rodrigues parameters, which are derived from the
 *    gyroscope's angular velocity.
 *  - Rotate the covariance matrix to reflect the updated attitude using a second-order
 *    taylor series approximation.
 *
 * 3. Measurement Update:
 *  - Incorporate new sensor measurements (e.g., accelerometer, optical-flow, barometer, gps) to
 *    update the state vector.
 *  - Compute the Kalman gain and update the state estimate.
 *  - Update the covariance matrix to reflect the reduced uncertainty after incorporating the
 *    measurements.
 *
 * 4. Attitude and Covariance Reset:
 *  - Apply the attitude error reset by multiplying the reference quaternion with the incremental
 *    rotation quaternion.
 *  - Reset the attitude error to zero.
 *  - Rotate the covariance matrix to maintain consistency with the updated reference attitude.
 *
*/

class ExtendedKalmanFilter {
    public:
        ExtendedKalmanFilter(const EKFParameters& ekf_params);
        void update(const FCU_STATES::STATES fcu_state,
                    const Vector3f accel,
                    const Vector3f gyro,
                    const OptiflowEkfData optiflow,
                    const GpsEkfData gps_data,
                    const float altimeter);
        Vector3f get_ekf_position() const;
        Vector3f get_ekf_velocity() const;
        Vector3f get_ekf_attitude() const;
    private:
        void predict();
        void update_measurement(VectorXf& h, float residual, float std_dev);
        void gps_velocity_measurement();
        void barometer_measurement();
        void optiflow_forward_measurement();
        void optiflow_downward_measurement();
        void downward_lidar_measurement();
        void forward_lidar_measurement();
        void update_attitude();
        void initialize_ekf();
        bool check_ekf_stability();

        FCU_STATES::STATES fcu_state;
        Vector3f accel;
        Vector3f gyro;
        OptiflowEkfData optiflow;
        GpsEkfData gps_data;
        float altimeter;
        EKFParameters ekf_params;

        Vector4f q; // attitude quaternion
        VectorXf S; // state vector
        MatrixXf P; // covariance matrix
        Matrix3f R; // attitude rotation matrix
        Vector4f initial_quaternion;
};

#endif // EKF_H_