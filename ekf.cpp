#include "ekf.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(const EKFParameters& params) :
    ekf_params(params),
    q(),
    S(EKF_STATE_DIM),
    P(EKF_STATE_DIM, EKF_STATE_DIM),
    R(),
    initial_quaternion()
{
    initialize_ekf();
}

void ExtendedKalmanFilter::update(const FCU_STATES::STATES fcu_state,
                                  const Vector3f accel,
                                  const Vector3f gyro,
                                  const OptiflowEkfData optiflow,
                                  const GpsEkfData gps_data,
                                  const float altimeter)
{
    // Store the sensor data
    this->fcu_state = fcu_state;
    this->accel = accel;
    this->gyro = gyro;
    this->optiflow = optiflow;
    this->gps_data = gps_data;
    this->altimeter = altimeter;

    // State prediction
    predict();

    // Measurement updates
    if (gps_data.is_gps_new_data
            && gps_data.is_gps_connected
            && gps_data.is_gps_vel_valid
            && gps_data.is_gps_hdt_valid) {
        gps_velocity_measurement();
    }

    barometer_measurement();

    if (optiflow.forward_new_data) {
        optiflow_forward_measurement();
    }

    if (optiflow.downward_new_data) {
        optiflow_downward_measurement();
    }

    // Todo: add lidar sensors maybe?
    downward_lidar_measurement();
    forward_lidar_measurement();

    update_attitude();

    // Check ekf stability reset if unstable
    if (!check_ekf_stability()) {
        initialize_ekf();
    }
}

Vector3f ExtendedKalmanFilter::get_ekf_position() const
{
    Vector3f position;
    position(0) = S(EKF_STATE_X);
    position(1) = S(EKF_STATE_Y);
    position(2) = S(EKF_STATE_Z);
    return position;
}

Vector3f ExtendedKalmanFilter::get_ekf_velocity() const
{
    Vector3f velocity;
    velocity(0) = S(EKF_STATE_PX);
    velocity(1) = S(EKF_STATE_PY);
    velocity(2) = S(EKF_STATE_PZ);
    return velocity;
}

Vector3f ExtendedKalmanFilter::get_ekf_attitude() const
{
    Vector3f angles;
    float w = q.w();
    float x = q.x();
    float y = q.y();
    float z = q.z();

    // roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    angles(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = std::sqrt(1 + 2 * (w * y - x * z));
    float cosp = std::sqrt(1 - 2 * (w * y - x * z));
    angles(1) = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    angles(2) = std::atan2(siny_cosp, cosy_cosp);

    // Todo: ask Gino what the convention should be for euler angles

    return angles;
}

void ExtendedKalmanFilter::predict()
{
    const float g = 9.81;
    const float dt = 1.0 / FCU_LOOP_FREQ;

    MatrixXf A(EKF_STATE_DIM, EKF_STATE_DIM);

    // Initialize as the identity matrix
    A(EKF_STATE_X, EKF_STATE_X) = 1.0;
    A(EKF_STATE_Y, EKF_STATE_Y) = 1.0;
    A(EKF_STATE_Z, EKF_STATE_Z) = 1.0;
    A(EKF_STATE_PX, EKF_STATE_PX) = 1.0;
    A(EKF_STATE_PY, EKF_STATE_PY) = 1.0;
    A(EKF_STATE_PZ, EKF_STATE_PZ) = 1.0;
    A(EKF_STATE_D0, EKF_STATE_D0) = 1.0;
    A(EKF_STATE_D1, EKF_STATE_D1) = 1.0;
    A(EKF_STATE_D2, EKF_STATE_D2) = 1.0;

     // position from body-frame velocity
    A(EKF_STATE_X, EKF_STATE_PX) = R(0,0)*dt;
    A(EKF_STATE_Y, EKF_STATE_PX) = R(1,0)*dt;
    A(EKF_STATE_Z, EKF_STATE_PX) = R(2,0)*dt;

    A(EKF_STATE_X, EKF_STATE_PY) = R(0,1)*dt;
    A(EKF_STATE_Y, EKF_STATE_PY) = R(1,1)*dt;
    A(EKF_STATE_Z, EKF_STATE_PY) = R(2,1)*dt;

    A(EKF_STATE_X, EKF_STATE_PZ) = R(0,2)*dt;
    A(EKF_STATE_Y, EKF_STATE_PZ) = R(1,2)*dt;
    A(EKF_STATE_Z, EKF_STATE_PZ) = R(2,2)*dt;

    // position from attitude error
    A(EKF_STATE_X, EKF_STATE_D0) = (S(EKF_STATE_PY)*R(0,2) - S(EKF_STATE_PZ)*R(0,1))*dt;
    A(EKF_STATE_Y, EKF_STATE_D0) = (S(EKF_STATE_PY)*R(1,2) - S(EKF_STATE_PZ)*R(1,1))*dt;
    A(EKF_STATE_Z, EKF_STATE_D0) = (S(EKF_STATE_PY)*R(2,2) - S(EKF_STATE_PZ)*R(2,1))*dt;

    A(EKF_STATE_X, EKF_STATE_D1) = (- S(EKF_STATE_PX)*R(0,2) + S(EKF_STATE_PZ)*R(0,0))*dt;
    A(EKF_STATE_Y, EKF_STATE_D1) = (- S(EKF_STATE_PX)*R(1,2) + S(EKF_STATE_PZ)*R(1,0))*dt;
    A(EKF_STATE_Z, EKF_STATE_D1) = (- S(EKF_STATE_PX)*R(2,2) + S(EKF_STATE_PZ)*R(2,0))*dt;

    A(EKF_STATE_X, EKF_STATE_D2) = (S(EKF_STATE_PX)*R(0,1) - S(EKF_STATE_PY)*R(0,0))*dt;
    A(EKF_STATE_Y, EKF_STATE_D2) = (S(EKF_STATE_PX)*R(1,1) - S(EKF_STATE_PY)*R(1,0))*dt;
    A(EKF_STATE_Z, EKF_STATE_D2) = (S(EKF_STATE_PX)*R(2,1) - S(EKF_STATE_PY)*R(2,0))*dt;

    // body-frame velocity
    A(EKF_STATE_PX, EKF_STATE_PX) = 1.0f;
    A(EKF_STATE_PY, EKF_STATE_PX) =-gyro.z()*dt;
    A(EKF_STATE_PZ, EKF_STATE_PX) = gyro.y()*dt;

    A(EKF_STATE_PX, EKF_STATE_PY) = gyro.z()*dt;
    A(EKF_STATE_PY, EKF_STATE_PY) = 1.0f;
    A(EKF_STATE_PZ, EKF_STATE_PY) =-gyro.x()*dt;

    A(EKF_STATE_PX, EKF_STATE_PZ) =-gyro.y()*dt;
    A(EKF_STATE_PY, EKF_STATE_PZ) = gyro.x()*dt;
    A(EKF_STATE_PZ, EKF_STATE_PZ) = 1.0f;

    // body-frame velocity from attitude error
    A(EKF_STATE_PX, EKF_STATE_D0) =  0;
    A(EKF_STATE_PY, EKF_STATE_D0) = -g * R(2,2)*dt;
    A(EKF_STATE_PZ, EKF_STATE_D0) =  g * R(2,1)*dt;

    A(EKF_STATE_PX, EKF_STATE_D1) =  g * R(2,2)*dt;
    A(EKF_STATE_PY, EKF_STATE_D1) =  0;
    A(EKF_STATE_PZ, EKF_STATE_D1) = -g * R(2,0)*dt;

    A(EKF_STATE_PX, EKF_STATE_D2) = -g * R(2,1)*dt;
    A(EKF_STATE_PY, EKF_STATE_D2) = g * R(2,0)*dt;
    A(EKF_STATE_PZ, EKF_STATE_D2) =  0;

    // Calculate Rodrigues' Paremeter
    float d0 = gyro.x()*dt/2.0f;
    float d1 = gyro.y()*dt/2.0f;
    float d2 = gyro.z()*dt/2.0f;

    // Update attitude body attitude error and rotate covariance second-order appriximation
    // Instead of updating the attitude-error and the incoperating new error into current attitude
    // Which requires rotation of attitude-error covariance to direclty update the attitude-error
    A(EKF_STATE_D0, EKF_STATE_D0) = 1.0f - d1*d1/2.0f - d2*d2/2.0f;
    A(EKF_STATE_D0, EKF_STATE_D1) = d2 + d0*d1/2.0f;
    A(EKF_STATE_D0, EKF_STATE_D2) = -d1 + d0*d2/2.0f;

    A(EKF_STATE_D1, EKF_STATE_D0) = -d2 + d0*d1/2.0f;
    A(EKF_STATE_D1, EKF_STATE_D1) = 1.0f - d0*d0/2.0f - d2*d2/2.0f;
    A(EKF_STATE_D1, EKF_STATE_D2) = d0 + d1*d2/2.0f;

    A(EKF_STATE_D2, EKF_STATE_D0) = d1 + d0*d2/2.0f;
    A(EKF_STATE_D2, EKF_STATE_D1) = -d0 + d1*d2/2.0f;
    A(EKF_STATE_D2, EKF_STATE_D2) = 1.0 - d0*d0/2.0f - d1*d1/2.0f;

    // Update the covariance matrix
    P = A * P * A.transpose();  // Todo: add proccess noise

    // Prediction step
    // Thrust is not used a control put given the complexity of the
    // drone, the accelerometer is used to directly measure drone thrust
    // this also accounts for behavior against the wall.
    float dx, dy, dz;
    float prev_px = S(EKF_STATE_PX);
    float prev_py = S(EKF_STATE_PY);
    float prev_pz = S(EKF_STATE_PZ);
    bool drone_is_ground = fcu_state <= FCU_STATES::PREFLIGHT
                                || fcu_state == FCU_STATES::POSTFLIGHT
                                || fcu_state == FCU_STATES::DEMO;
    // Avoid positional drift by only using acceleration in z-axis.
    if (drone_is_ground) {

        dx = S(EKF_STATE_PX) * dt;
        dy = S(EKF_STATE_PY) * dt;
        dz = S(EKF_STATE_PZ) * dt + accel.z() * dt * dt / 2.0f;

        // Positional update inertial frame
        S(EKF_STATE_X) += R(0, 0)*dx + R(0, 1)*dy + R(0, 2)*dz;
        S(EKF_STATE_Y) += R(1, 0)*dx + R(1, 1)*dy + R(1, 2)*dz;
        S(EKF_STATE_Z) += R(2, 0)*dx + R(2, 1)*dy + R(2, 2)*dz - g*dt*dt/2.0f;

        // Velocity update inertial frame
        S(EKF_STATE_PX) += dt * (gyro.z() * prev_py - gyro.y() * prev_pz - g * R(2,0));
        S(EKF_STATE_PY) += dt * (-gyro.z() * prev_px + gyro.x() * prev_pz - g * R(2, 1));
        S(EKF_STATE_PZ) += dt * (accel.z() + gyro.y() * prev_px - gyro.x() * prev_py - g * R(2,2));

    } else {

        dx = S(EKF_STATE_PX) * dt + accel.x() * dt * dt / 2.0f;
        dy = S(EKF_STATE_PY) * dt + accel.y() * dt * dt / 2.0f;
        dz = S(EKF_STATE_PZ) * dt + accel.z() * dt * dt / 2.0f;

        // Positional update inertial frame
        S(EKF_STATE_X) += R(0, 0)*dx + R(0, 1)*dy + R(0, 2)*dz;
        S(EKF_STATE_Y) += R(1, 0)*dx + R(1, 1)*dy + R(1, 2)*dz;
        S(EKF_STATE_Z) += R(2, 0)*dx + R(2, 1)*dy + R(2, 2)*dz - g*dt*dt/2.0f;

        // Velocity update inertial frame
        S(EKF_STATE_PX) += dt * (accel.x() + gyro.z() * prev_py - gyro.y() * prev_pz - g * R(2, 0));
        S(EKF_STATE_PY) += dt * (accel.y() - gyro.z() * prev_px + gyro.x() * prev_pz - g * R(2, 1));
        S(EKF_STATE_PZ) += dt * (accel.z() + gyro.y() * prev_px - gyro.x() * prev_py - g * R(2, 2));
    }

    // update the drones attitude using the gyro while maintaining the quaternion
    // representation's integrity.
    // integrated gyroscope angular velocity over the sample period.
    float wx = gyro.x() * dt;
    float wy = gyro.y() * dt;
    float wz = gyro.z() * dt;

    // Compute the quaternion values in [w, x, y, z] order
    float angle = sqrt(wx * wx + wy * wy + wz * wz) + EPS;
    float cosine = cos(angle / 2.0f);
    float sine = sin(angle / 2.0f);
    float delta_q[4] = {cosine, wx * sine / angle, wy * sine / angle, wz * sine / angle};

    // rotate the quad's attitude by the delta quaternion vector
    float tmpq0 = delta_q[0]*q[0] - delta_q[1]*q[1] - delta_q[2]*q[2] - delta_q[3]*q[3];
    float tmpq1 = delta_q[1]*q[0] + delta_q[0]*q[1] + delta_q[3]*q[2] - delta_q[2]*q[3];
    float tmpq2 = delta_q[2]*q[0] - delta_q[3]*q[1] + delta_q[0]*q[2] + delta_q[1]*q[3];
    float tmpq3 = delta_q[3]*q[0] + delta_q[2]*q[1] - delta_q[1]*q[2] + delta_q[0]*q[3];

    // normalize and store the new attitude quaternion
    float norm = sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3);
    q[0] = tmpq0 / norm;
    q[1] = tmpq1 / norm;
    q[2] = tmpq2 / norm;
    q[3] = tmpq3 / norm;
}

// This function updates the measurement one at time, compare to other ekf
// where the measurment vector contains various sensor states.
// This allows me to fuse data from various sensors such as optical-flow, barometer,
// GPS all using the same function.
void ExtendedKalmanFilter::update_measurement(VectorXf& h, float residual, float std_dev)
{
    if (h.size() != EKF_STATE_DIM) {
        return;
    }

    VectorXf K = VectorXf::Zero(EKF_STATE_DIM);

    // Innovation covariance
    RowVectorXf PH_T(EKF_STATE_DIM);
    PH_T = P * h.transpose();

    // Sensor Noise
    float N = std_dev * std_dev;
    float innovation = N;
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        innovation += PH_T(i) * h(i);           // Update Innovation
    }

    // Calculate the kalman gain and preform state update
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        K(i) = PH_T(i) / innovation;
        S(i) = S(i) + K(i) * residual;
    }

    // Update the covariance matrix
    P = (MatrixXf::Identity(EKF_STATE_DIM, EKF_STATE_DIM) - K * h.transpose()) * P;


    /*
     *
     * Calculate Measurement Noise Contribution: For each pair of indices (i, j),
     * it calculates a value v that represents the contribution of measurement noise
     * to the covariance. This is done by multiplying the Kalman gain K(i) by some
     * measurement noise R and then by K(j). The Kalman gain is a factor that reflects
     * the amount of trust in the measurements over the predictions.
     *
     * Update Covariance Matrix with Noise: The calculated noise contribution v is
     * then added to the average of the symmetric elements P(i, j) and P(j, i),
     * effectively updating the covariance matrix with the measurement noise. This
     * is done by the line calculating p.
     *
    */

    for (int i=0; i<EKF_STATE_DIM; i++) {
        for (int j=i; j<EKF_STATE_DIM; j++) {
            float v = K(i) * N * K(j);
            float sym = 0.5f*P(i, j) + 0.5f*P(j, i) + v; // add measurement noise
            if (std::isnan(sym) || sym > MAX_COVARIANCE) {
                P(i, j) = P(j, i) = MAX_COVARIANCE;
            } else if ( i==j && sym < MIN_COVARIANCE ) {
                P(i, j) = P(j, i) = MIN_COVARIANCE;
            } else {
                P(i, j) = P(j, i) = sym;
            }
        }
    }
}

void ExtendedKalmanFilter::gps_velocity_measurement()
{
    // Ask gino if ground couse work during angled flight
    // GPS velocity has course and GPS heading has heading and pitch angle
    Vector3f gps_vel_ned = gps_data.gps_velocity;

    Vector3f gps_vel_ned_std = gps_data.gps_velocity_std;

    float heading = gps_data.heading * M_PI / 180.0f;
    float pitch = gps_data.pitch * M_PI / 180.0f;

    // Rotate the heading from NED to body frame
    // todo this incorrect, need to rotate the heading to the body frame
    Matrix3f R_heading = Matrix3f::Zero();
    R_heading(0, 0) = cos(heading);
    R_heading(0, 1) = sin(heading);
    R_heading(0, 2) = 0;
    R_heading(1, 0) = -sin(heading);
    R_heading(1, 1) = cos(heading);
    R_heading(1, 2) = 0;
    R_heading(2, 0) = 0;
    R_heading(2, 1) = 0;
    R_heading(2, 2) = 1;

    // Rotate the pitch from NED to body frame
    Matrix3f R_pitch = Matrix3f::Zero();
    R_pitch(0, 0) = cos(pitch);
    R_pitch(0, 1) = 0;
    R_pitch(0, 2) = -sin(pitch);
    R_pitch(1, 0) = 0;
    R_pitch(1, 1) = 1;
    R_pitch(1, 2) = 0;
    R_pitch(2, 0) = sin(pitch);
    R_pitch(2, 1) = 0;
    R_pitch(2, 2) = cos(pitch);

    // Convert the GPS velocity from NED to body frame
    Vector3f gps_vel = R_pitch * R_heading * gps_vel_ned;
    // Rotate to inertial frame
    gps_vel = R * gps_vel;

    // Convert the GPS velocity standard deviation from NED to body frame
    Vector3f gps_vel_std = R_pitch * R_heading * gps_vel_ned_std;
    gps_vel_std = R * gps_vel_std;

    // Measurement Jaccobian
    // The derviative of the jaccobian is the equal rotation matrix
    MatrixXf H = MatrixXf::Zero(3, EKF_STATE_DIM);
    H.block<3, 3>(0, EKF_STATE_PX) = R_pitch * R_heading;

    // Apply update to each measurement
    for (int i = 0; i<3; i++) {
        VectorXf h = H.row(i);
        update_measurement(h, S(EKF_STATE_PX + i) - gps_vel(i), gps_vel_std(i));
    }
}

// Currently, the Optiflow system on the OBC acts as a separate state estimator
// and is prone to drifting as the drone yaws. This is because the assumption
// that point trackers are always on a flat, perpendicular plane in front of the
// camera is not valid. This assumption leads to inaccuracies, especially during
// yaw movements, causing the states in the Kalman filter to diverge. As a result,
// the covariance error grows, leading to the Kalman filter relying less on OptiFlow
// data to correct its predicted state.
//
// Instead we will be using the raw pixel flow data to estimate the drone velocity
// in the x and y directions. This will be done by integrating the pixel flow data
// over time to get the drone's velocity and angular velocity of the drone. The velocity
// will then be used to update the state vector in the Kalman filter.
void ExtendedKalmanFilter::optiflow_forward_measurement()
{
    // forward optical flow can only be used when there is valid forward lidar
    // data that will provide depth estimate the forward surface.

}

void ExtendedKalmanFilter::optiflow_downward_measurement()
{
    const float FLOW_RESOLUTION = 1.0f;
    const float Npix = 1024;                            // Number of pixels assumed same in [x, y] direction
    const float thetapix = 0.717f;                      // apeture angle of roughly 45 degrees
    float dx = S[EKF_STATE_PX];
    float dy = S[EKF_STATE_PY];
    float h = S[EKF_STATE_Z];
    float omega_x = gyro.x();
    float omega_y = gyro.y();
    float dt = optiflow.downward_dt;
    float pixel_flow_x = optiflow.downward.x();
    float pixel_flow_y = optiflow.downward.y();

    // Predict X Pixel flow
    VectorXf Hx = VectorXf::Zero(EKF_STATE_DIM);
    float pred_Nx = (dt * Npix / thetapix) * ((dx * R(2, 2) / h) - omega_y);
    float meas_Nx = pixel_flow_x * FLOW_RESOLUTION;

    // derive the Jaccobian flow model
    Hx[EKF_STATE_Z] = (Npix * dt / thetapix) * ((R(2, 2) * dx) / (-h * h));
    Hx[EKF_STATE_PX] = (Npix * dt / thetapix) * (R(2, 2) / h);

    // Update X flow measurement
    update_measurement(Hx, (meas_Nx - pred_Nx), ekf_params.meas_optiflow_x*FLOW_RESOLUTION);

    // Predict Y Pixel flow
    VectorXf Hy = VectorXf::Zero(EKF_STATE_DIM);
    float pred_Ny = (dt * Npix / thetapix) * ((dy * R(2, 2) / h) - omega_x);
    float meas_Ny = pixel_flow_y * FLOW_RESOLUTION;

    // derive the Jaccobian flow model
    Hy[EKF_STATE_Z] = (Npix * dt / thetapix) * ((R(2, 2) * dy) / (-h * h));
    Hy[EKF_STATE_PY] = (Npix * dt / thetapix) * (R(2, 2) / h);

    // Update X flow measurement
    update_measurement(Hy, (meas_Ny - pred_Ny), ekf_params.meas_optiflow_y*FLOW_RESOLUTION);

}

void ExtendedKalmanFilter::barometer_measurement()
{
    bool drone_is_ground = fcu_state <= FCU_STATES::PREFLIGHT
                                || fcu_state == FCU_STATES::POSTFLIGHT
                                || fcu_state == FCU_STATES::DEMO;

    // Jaccobian Matrix
    VectorXf H = VectorXf::Zero(EKF_STATE_DIM);
    H(EKF_STATE_Z) = 1.0f;

    // if drone is on the ground calculate the an average reference
    static int average_cnt = 0;
    if(drone_is_ground) {
        ekf_params.initial_baro_ref_height += altimeter;
        ekf_params.initial_baro_ref_height = ekf_params.initial_baro_ref_height / (++average_cnt);
        update_measurement(H, 0.0f, ekf_params.meas_noise_baro);
    } else {
        // Only update baro if the baro is valid and not zero
        if (ekf_params.initial_baro_ref_height > 0.1f) {
            float height = ekf_params.initial_baro_ref_height - altimeter;
            float residual = S(EKF_STATE_Z) - height;
            update_measurement(H, residual, ekf_params.meas_noise_baro);
        }
    }
}

void ExtendedKalmanFilter::forward_forward_measurement()
{
    VectorXf H = VectorXf::Zero(EKF_STATE_DIM);
    H(EKF_STATE_PX) = ;
    H(EKF_STATE_PY) = 1.0f;
    H(EKF_STATE_PZ) = 1.0f;
}

void ExtendedKalmanFilter::downward_lidar_measurement()
{

}

void ExtendedKalmanFilter::update_attitude()
{
    // Incorporate the new attitude into the state vector
    float v0 = S(EKF_STATE_D0);
    float v1 = S(EKF_STATE_D1);
    float v2 = S(EKF_STATE_D2);

    // avoid signularity and quaternion instability by using attitude error less than 10
    if ((fabs(v0) < MAX_ATTITUDE_ERROR && fabs(v0) > EPS)
        && (fabs(v1) < MAX_ATTITUDE_ERROR && fabs(v1) > EPS)
        && (fabs(v2) < MAX_ATTITUDE_ERROR && fabs(v2) > EPS)) {

        // Convert state attitude into quaternion
        float angle = sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
        float cosine = cos(angle / 2.0f);
        float sine = sin(angle / 2.0f);
        float delta_q[4] = {cosine, v0 * sine / angle, v1 * sine / angle, v2 * sine / angle};

        // rotate the quad's attitude by the delta quaternion vector
        float tmpq0 = delta_q[0] * q(0) - delta_q[1] * q(1) - delta_q[2] * q(2) - delta_q[3] * q(3);
        float tmpq1 = delta_q[1] * q(0) + delta_q[0] * q(1) + delta_q[3] * q(2) - delta_q[2] * q(3);
        float tmpq2 = delta_q[2] * q(0) - delta_q[3] * q(1) + delta_q[0] * q(2) + delta_q[1] * q(3);
        float tmpq3 = delta_q[3] * q(0) + delta_q[2] * q(1) - delta_q[1] * q(2) + delta_q[0] * q(3);

        // normalize and store the new attitude quaternion
        float norm = sqrt(tmpq0*tmpq0 + tmpq1*tmpq1 + tmpq2*tmpq2 + tmpq3*tmpq3);
        q(0) = tmpq0 / norm;
        q(1) = tmpq1 / norm;
        q(2) = tmpq2 / norm;
        q(3) = tmpq3 / norm;

        // Rotate the covariance matrix using second appriximation
        float d0 = v0/2;
        float d1 = v1/2;
        float d2 = v2/2;

        MatrixXf A(EKF_STATE_DIM, EKF_STATE_DIM);

        A(EKF_STATE_X, EKF_STATE_X) = 1;
        A(EKF_STATE_Y, EKF_STATE_Y) = 1;
        A(EKF_STATE_Z, EKF_STATE_Z) = 1;

        A(EKF_STATE_PX, EKF_STATE_PX) = 1;
        A(EKF_STATE_PY, EKF_STATE_PY) = 1;
        A(EKF_STATE_PZ, EKF_STATE_PZ) = 1;

        A(EKF_STATE_D0, EKF_STATE_D0) =  1 - d1*d1/2 - d2*d2/2;
        A(EKF_STATE_D0, EKF_STATE_D1) =  d2 + d0*d1/2;
        A(EKF_STATE_D0, EKF_STATE_D2) = -d1 + d0*d2/2;

        A(EKF_STATE_D1, EKF_STATE_D0) = -d2 + d0*d1/2;
        A(EKF_STATE_D1, EKF_STATE_D1) =  1 - d0*d0/2 - d2*d2/2;
        A(EKF_STATE_D1, EKF_STATE_D2) =  d0 + d1*d2/2;

        A(EKF_STATE_D2, EKF_STATE_D0) =  d1 + d0*d2/2;
        A(EKF_STATE_D2, EKF_STATE_D1) = -d0 + d1*d2/2;
        A(EKF_STATE_D2, EKF_STATE_D2) = 1 - d0*d0/2 - d1*d1/2;

        // Update the covariance matrix
        P = A * P * A.transpose();
    }

    // Update rotation matrix
    R(0, 0) = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    R(0, 1) = 2 * q[1] * q[2] - 2 * q[0] * q[3];
    R(0, 2) = 2 * q[1] * q[3] + 2 * q[0] * q[2];
    R(1, 0) = 2 * q[1] * q[2] + 2 * q[0] * q[3];
    R(1, 1) = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    R(1, 2) = 2 * q[2] * q[3] - 2 * q[0] * q[1];
    R(2, 0) = 2 * q[1] * q[3] - 2 * q[0] * q[2];
    R(2, 1) = 2 * q[2] * q[3] + 2 * q[0] * q[1];
    R(2, 2) = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    // reset the attitude error
    S(EKF_STATE_D0) = 0.0;
    S(EKF_STATE_D1) = 0.0;
    S(EKF_STATE_D2) = 0.0;
}

void ExtendedKalmanFilter::initialize_ekf()
{
    // Initialize the internal state vector
    S[EKF_STATE_X] = ekf_params.initial_x;
    S[EKF_STATE_Y] = ekf_params.initial_y;
    S[EKF_STATE_Z] = ekf_params.initial_z;
    S[EKF_STATE_PX] = 0.0;
    S[EKF_STATE_PY] = 0.0;
    S[EKF_STATE_PZ] = 0.0;
    S[EKF_STATE_D0] = 0.0;
    S[EKF_STATE_D1] = 0.0;
    S[EKF_STATE_D2] = 0.0;

    // reset the attitude quaternion
    initial_quaternion(0) = cos(ekf_params.initial_yaw / 2.0f);
    initial_quaternion(1) = 0.0;
    initial_quaternion(2) = 0.0;
    initial_quaternion(3) = sin(ekf_params.initial_yaw / 2.0f);

    // copy the initial quaternion to the attitude quaternion
    q = initial_quaternion;

    // set the intial roation matrix to the identity
    R = Matrix3f::Identity();

    // set the covariance matrix to zeros
    P = MatrixXf::Zero(EKF_STATE_DIM, EKF_STATE_DIM);

    // update the diagonals of the covariance matrix
    P(EKF_STATE_X, EKF_STATE_X) = ekf_params.std_initial_position_xy * ekf_params.std_initial_position_xy;
    P(EKF_STATE_Y, EKF_STATE_Y) = ekf_params.std_initial_position_xy * ekf_params.std_initial_position_xy;
    P(EKF_STATE_Z, EKF_STATE_Z) = ekf_params.std_initial_position_z * ekf_params.std_initial_position_z;
    P(EKF_STATE_PX, EKF_STATE_PX) = ekf_params.std_initial_velocity * ekf_params.std_initial_velocity;
    P(EKF_STATE_PY, EKF_STATE_PY) = ekf_params.std_initial_velocity * ekf_params.std_initial_velocity;
    P(EKF_STATE_PZ, EKF_STATE_PZ) = ekf_params.std_initial_velocity * ekf_params.std_initial_velocity;
    P(EKF_STATE_D0, EKF_STATE_D0) = ekf_params.std_initial_attitude_rollpitch * ekf_params.std_initial_attitude_rollpitch;
    P(EKF_STATE_D1, EKF_STATE_D1) = ekf_params.std_initial_attitude_rollpitch * ekf_params.std_initial_attitude_rollpitch;
    P(EKF_STATE_D2, EKF_STATE_D2) = ekf_params.std_initial_attitude_yaw * ekf_params.std_initial_attitude_yaw;
}


/*
 *
 * To access the stability of the EKF, look at how the covariance matrix P
 * evolves over time. Stability can be checked by ensuring the covariance
 * matrix is:
 *
 * 1) Check Positive Definiteness of P: The error covariance matrix P remains
 *    positive definite. This is crucial for the EKF to provide meaningful estimates.
 *
 * 2) Eigenvalue Analysis: The eigenvalues of the state transition matrix (or the Jacobian
 *    of the process model). If all eigenvalues are within magnitude less than 1, the
 *    system is stable.
 *
 * 3) Boundedness of Covariance Matrix: Check if the elements of the covariance matrix P
 *    are bounded over time.
 *
*/
bool ExtendedKalmanFilter::check_ekf_stability()
{
    // Check the boundedness of the covariance matrix of covariance matrix P
    const float upper_bound = 1e3;
    const float lower_bound = 1e-3;
    float max_covariance = P.maxCoeff();
    float min_covariance = P.minCoeff();

    if (max_covariance > upper_bound || min_covariance < lower_bound) {
        // If the covariance matrix is unbounded, the system is unstable.
        return false;
    }
    // If all checks passed, the EKF is considered stable.
    return true;
}