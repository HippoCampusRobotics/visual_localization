from __future__ import print_function

import threading
from dataclasses import astuple, dataclass, field

import numpy as np


@dataclass
class InitialState:
    x: float = field(default_factory=lambda: 0.0)
    y: float = field(default_factory=lambda: 0.0)
    z: float = field(default_factory=lambda: 0.0)
    roll: float = field(default_factory=lambda: 0.0)
    pitch: float = field(default_factory=lambda: 0.0)
    yaw: float = field(default_factory=lambda: 0.0)
    dx: float = field(default_factory=lambda: 0.0)
    dy: float = field(default_factory=lambda: 0.0)
    dz: float = field(default_factory=lambda: 0.0)
    droll: float = field(default_factory=lambda: 0.0)
    dpitch: float = field(default_factory=lambda: 0.0)
    dyaw: float = field(default_factory=lambda: 0.0)


@dataclass
class InitialStateCovariance:
    x: float = field(default_factory=lambda: 0.0)
    y: float = field(default_factory=lambda: 0.0)
    z: float = field(default_factory=lambda: 0.0)
    roll: float = field(default_factory=lambda: 0.0)
    pitch: float = field(default_factory=lambda: 0.0)
    yaw: float = field(default_factory=lambda: 0.0)
    dx: float = field(default_factory=lambda: 0.0)
    dy: float = field(default_factory=lambda: 0.0)
    dz: float = field(default_factory=lambda: 0.0)
    droll: float = field(default_factory=lambda: 0.0)
    dpitch: float = field(default_factory=lambda: 0.0)
    dyaw: float = field(default_factory=lambda: 0.0)


@dataclass
class ProcessNoise:
    x: float = field(default_factory=lambda: 0.0)
    y: float = field(default_factory=lambda: 0.0)
    z: float = field(default_factory=lambda: 0.0)
    roll: float = field(default_factory=lambda: 0.0)
    pitch: float = field(default_factory=lambda: 0.0)
    yaw: float = field(default_factory=lambda: 0.0)
    dx: float = field(default_factory=lambda: 0.0)
    dy: float = field(default_factory=lambda: 0.0)
    dz: float = field(default_factory=lambda: 0.0)
    droll: float = field(default_factory=lambda: 0.0)
    dpitch: float = field(default_factory=lambda: 0.0)
    dyaw: float = field(default_factory=lambda: 0.0)


@dataclass
class MeasurementNoise:
    distance: float = field(default_factory=lambda: 0.0)
    yaw: float = field(default_factory=lambda: 0.0)


@dataclass
class MeasurementNoiseOrientation:
    roll: float = field(default_factory=lambda: 0.0)
    pitch: float = field(default_factory=lambda: 0.0)


@dataclass
class EkfParams:
    initial_state: InitialState = field(default_factory=lambda: InitialState())
    initial_state_covariance: InitialStateCovariance = field(
        default_factory=lambda: InitialStateCovariance()
    )
    process_noise: ProcessNoise = field(default_factory=lambda: ProcessNoise())
    measurement_noise: MeasurementNoise = field(
        default_factory=lambda: MeasurementNoise()
    )
    measurement_noise_orientation: MeasurementNoiseOrientation = field(
        default_factory=lambda: MeasurementNoiseOrientation()
    )
    dim_meas: int = field(default_factory=lambda: 0)
    dim_state: int = field(default_factory=lambda: 0)


class ExtendedKalmanFilter(object):
    def __init__(self, ekf_params: EkfParams):
        self.ekf_params = ekf_params

        self._x_est_0 = np.array(
            astuple(self.ekf_params.initial_state)
        ).reshape((-1, 1))
        self._p0_mat = self.ekf_params.initial_state_covariance
        self._p0_mat = np.array(
            np.diag(
                [
                    self.ekf_params.initial_state_covariance.x**2,
                    self.ekf_params.initial_state_covariance.y**2,
                    self.ekf_params.initial_state_covariance.z**2,
                    self.ekf_params.initial_state_covariance.roll**2,
                    self.ekf_params.initial_state_covariance.pitch**2,
                    self.ekf_params.initial_state_covariance.yaw**2,
                    self.ekf_params.initial_state_covariance.dx**2,
                    self.ekf_params.initial_state_covariance.dy**2,
                    self.ekf_params.initial_state_covariance.dz**2,
                    self.ekf_params.initial_state_covariance.droll**2,
                    self.ekf_params.initial_state_covariance.dpitch**2,
                    self.ekf_params.initial_state_covariance.dyaw**2,
                ]
            )
        )

        self._x_est = self._x_est_0
        self._x_est_last = self._x_est
        self._p_mat = self._p0_mat

        self._last_time_stamp_update = 0
        self._last_time_stamp_prediction = 0
        self.lock = threading.Lock()

        self.process_model = ProcessModel(self.ekf_params)
        self.measurement_model = MeasurementModelDistances(self.ekf_params)

    def get_x_est(self):
        return np.copy(self._x_est)

    def get_x_est_0(self):
        return np.copy(self._x_est_0)

    def get_p_mat(self):
        return np.copy(self._p_mat)

    def get_x_est_last(self):
        return np.copy(self._x_est_last)

    def reset(self, x_est_0=None, p0_mat=None):
        if x_est_0:
            self._x_est = x_est_0
        else:
            self._x_est = self._x_est_0
        if p0_mat:
            self._p_mat = p0_mat
        else:
            self._p_mat = self._p0_mat

    def predict(self, dt):
        self._x_est_last = self._x_est
        self._x_est = self.process_model.f(self.get_x_est(), dt)
        a_mat = self.process_model.f_jacobian(self.get_x_est(), dt)
        self._p_mat = (
            np.matmul(np.matmul(a_mat, self.get_p_mat()), a_mat.transpose())
            + self.process_model.V
        )

        # reset EKF if unrealistic values
        if np.absolute(self.get_x_est()[0]) > 10 or np.absolute(
            self.get_x_est()[1] > 10
        ):
            print('Resetting EKF: x or y value too far outside tank')
            self.reset()
        elif not np.all(np.isfinite(self.get_x_est())):
            print('Resetting EKF: unrealistically high value')
            print('x: ', self.get_x_est())
            self.reset()

        return True

    def update_vision_data(self, measurements, detected_tags):
        # measurement is: dist, yaw to each tag

        self._x_est_last = self._x_est
        z_est_vision = self.measurement_model.h_vision_data(
            self.get_x_est(), detected_tags
        )
        h_mat_vision = self.measurement_model.h_jacobian_vision_data(
            self.get_x_est(), detected_tags
        )
        w_mat_vision = self.measurement_model.vision_dynamic_meas_model(
            self.get_x_est(), measurements, detected_tags
        )

        y_vision = measurements - z_est_vision
        # wrap yaw innovation (every second entry) so it is between -pi and pi
        y_vision[1::2] = np.arctan2(
            np.sin(y_vision[1::2]), np.cos(y_vision[1::2])
        )

        self._x_est, self._p_mat = self._update(
            self.get_x_est(),
            self.get_p_mat(),
            y_vision,
            h_mat_vision,
            w_mat_vision,
        )

        return True

    def update_orientation_data(self, measurements):
        # measurement is: roll, pitch from /mavros/local_position/pose

        z_est_orient = self.measurement_model.h_orientation_data(
            self.get_x_est()
        )
        h_mat_orient = self.measurement_model.h_jacobian_orientation_data()

        y_orient = measurements - z_est_orient
        # wrap angle innovations between -pi and pi
        y_orient = np.arctan2(np.sin(y_orient), np.cos(y_orient))

        self._x_est, self._p_mat = self._update(
            self.get_x_est(),
            self.get_p_mat(),
            y_orient,
            h_mat_orient,
            self.measurement_model.w_mat_orientation,
        )

        return True

    def update_imu_data(self, measurements, w_mat_imu):
        # measurement is either: body rates + lin. acceleration
        #               or: only body rates

        # check which measurements we're using:
        if measurements.shape[0] == 3:
            # only using angular velocities
            using_lin_acc = False
        elif measurements.shape[0] == 6:
            using_lin_acc = True
        else:
            print('IMU measurement has unexpected size!')
            using_lin_acc = False

        z_est_imu = self.measurement_model.h_imu_data(
            self.get_x_est(), using_lin_acc
        )
        h_mat_imu = self.measurement_model.h_jacobian_imu_data(using_lin_acc)
        y_imu = measurements - z_est_imu
        self._x_est, self._p_mat = self._update(
            self.get_x_est(), self.get_p_mat(), y_imu, h_mat_imu, w_mat_imu
        )

        return True

    def _update(self, x_est, p_mat, y, h_mat, w_mat):
        """helper function for general update"""

        # compute K gain
        tmp = np.matmul(np.matmul(h_mat, p_mat), h_mat.transpose()) + w_mat
        k_mat = np.matmul(
            np.matmul(p_mat, h_mat.transpose()), np.linalg.inv(tmp)
        )

        # update state
        x_est = x_est + np.matmul(k_mat, y)

        # update covariance
        p_tmp = np.eye(self.ekf_params.dim_state) - np.matmul(k_mat, h_mat)
        p_mat = np.matmul(p_tmp, p_mat)
        # print('P_m diag: ', np.diag(self.get_p_mat()))
        return x_est, p_mat


"""
Different process models for the EKF for the visual localization
    - simple model: no prediction, new state = old state
    - ...

EKF state: [x, y, z, roll, pitch, yaw, dx, dy, dz, droll, dpitch, dyaw]
(linear and angular velocities in body frame)
"""


class ProcessModel(object):
    """Simple process model: no prediction"""

    def __init__(self, ekf_params: EkfParams):
        self.ekf_params = ekf_params
        self.V = np.array(
            np.diag(
                [
                    self.ekf_params.process_noise.x**2,
                    self.ekf_params.process_noise.y**2,
                    self.ekf_params.process_noise.z**2,
                    self.ekf_params.process_noise.roll**2,
                    self.ekf_params.process_noise.pitch**2,
                    self.ekf_params.process_noise.yaw**2,
                    self.ekf_params.process_noise.dx**2,
                    self.ekf_params.process_noise.dy**2,
                    self.ekf_params.process_noise.dz**2,
                    self.ekf_params.process_noise.droll**2,
                    self.ekf_params.process_noise.dpitch**2,
                    self.ekf_params.process_noise.dyaw**2,
                ]
            )
        )

    def f(self, x_est, dt):
        x_next = np.copy(x_est)
        return x_next

    def f_jacobian(self, x_est, dt):
        A = np.eye(self.ekf_params.dim_state)
        return A  # dim [dim_state X dim_state]


class ProcessModelVelocities(ProcessModel):
    """Simple point mass model,
    for derivation see scripts/matlab/point_mass_model
    """

    def __init__(self, dim_state, V):
        super(ProcessModelVelocities, self).__init__(dim_state, V)

    def f(self, x_est, dt):
        x_next = np.copy(x_est)
        x = x_next[0]
        y = x_next[1]
        z = x_next[2]
        R = x_next[3]
        P = x_next[4]
        Y = x_next[5]
        dx = x_next[6]
        dy = x_next[7]
        dz = x_next[8]
        # rr = x_next[9]
        # pr = x_next[10]
        # yr = x_next[11]

        # see matlab script
        x_next[:3] = np.array(
            [
                x
                + dt
                * (
                    dz
                    * (
                        np.sin(R) * np.sin(Y)
                        + np.cos(R) * np.cos(Y) * np.sin(P)
                    )
                    - dy
                    * (
                        np.cos(R) * np.sin(Y)
                        - np.cos(Y) * np.sin(P) * np.sin(R)
                    )
                    + dx * np.cos(P) * np.cos(Y)
                ),
                y
                + dt
                * (
                    dy
                    * (
                        np.cos(R) * np.cos(Y)
                        + np.sin(P) * np.sin(R) * np.sin(Y)
                    )
                    - dz
                    * (
                        np.cos(Y) * np.sin(R)
                        - np.cos(R) * np.sin(P) * np.sin(Y)
                    )
                    + dx * np.cos(P) * np.sin(Y)
                ),
                z
                + dt
                * (
                    dz * np.cos(P) * np.cos(R)
                    - dx * np.sin(P)
                    + dy * np.cos(P) * np.sin(R)
                ),
            ]
        )

        return x_next

    def f_jacobian(self, x_est, dt):
        # x = x_est[0]
        # y = x_est[1]
        # z = x_est[2]
        R = x_est[3]
        P = x_est[4]
        Y = x_est[5]
        dx = x_est[6]
        dy = x_est[7]
        dz = x_est[8]
        # dr = x_est[9]
        # dp = x_est[10]
        # dy = x_est[11]

        A = np.array(
            [
                [
                    1,
                    0,
                    0,
                    dt
                    * (
                        dy
                        * (
                            np.sin(R) * np.sin(Y)
                            + np.cos(R) * np.cos(Y) * np.sin(P)
                        )
                        + dz
                        * (
                            np.cos(R) * np.sin(Y)
                            - np.cos(Y) * np.sin(P) * np.sin(R)
                        )
                    ),
                    dt
                    * np.cos(Y)
                    * (
                        dz * np.cos(P) * np.cos(R)
                        - dx * np.sin(P)
                        + dy * np.cos(P) * np.sin(R)
                    ),
                    -dt
                    * (
                        dy
                        * (
                            np.cos(R) * np.cos(Y)
                            + np.sin(P) * np.sin(R) * np.sin(Y)
                        )
                        - dz
                        * (
                            np.cos(Y) * np.sin(R)
                            - np.cos(R) * np.sin(P) * np.sin(Y)
                        )
                        + dx * np.cos(P) * np.sin(Y)
                    ),
                    dt * np.cos(P) * np.cos(Y),
                    dt * np.cos(Y) * np.sin(P) * np.sin(R)
                    - dt * np.cos(R) * np.sin(Y),
                    dt
                    * (
                        np.sin(R) * np.sin(Y)
                        + np.cos(R) * np.cos(Y) * np.sin(P)
                    ),
                    0,
                    0,
                    0,
                ],
                [
                    0,
                    1,
                    0,
                    -dt
                    * (
                        dy
                        * (
                            np.cos(Y) * np.sin(R)
                            - np.cos(R) * np.sin(P) * np.sin(Y)
                        )
                        + dz
                        * (
                            np.cos(R) * np.cos(Y)
                            + np.sin(P) * np.sin(R) * np.sin(Y)
                        )
                    ),
                    dt
                    * np.sin(Y)
                    * (
                        dz * np.cos(P) * np.cos(R)
                        - dx * np.sin(P)
                        + dy * np.cos(P) * np.sin(R)
                    ),
                    dt
                    * (
                        dz
                        * (
                            np.sin(R) * np.sin(Y)
                            + np.cos(R) * np.cos(Y) * np.sin(P)
                        )
                        - dy
                        * (
                            np.cos(R) * np.sin(Y)
                            - np.cos(Y) * np.sin(P) * np.sin(R)
                        )
                        + dx * np.cos(P) * np.cos(Y)
                    ),
                    dt * np.cos(P) * np.sin(Y),
                    dt
                    * (
                        np.cos(R) * np.cos(Y)
                        + np.sin(P) * np.sin(R) * np.sin(Y)
                    ),
                    dt * np.cos(R) * np.sin(P) * np.sin(Y)
                    - dt * np.cos(Y) * np.sin(R),
                    0,
                    0,
                    0,
                ],
                [
                    0,
                    0,
                    1,
                    dt * np.cos(P) * (dy * np.cos(R) - dz * np.sin(R)),
                    -dt
                    * (
                        dx * np.cos(P)
                        + dz * np.cos(R) * np.sin(P)
                        + dy * np.sin(P) * np.sin(R)
                    ),
                    0,
                    -dt * np.sin(P),
                    dt * np.cos(P) * np.sin(R),
                    dt * np.cos(P) * np.cos(R),
                    0,
                    0,
                    0,
                ],
                [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
            ],
            dtype=float,
        )

        return A  # dim [dim_state X dim_state]


"""
Different measurement models for the EKF for the visual localization
    - Distances: using distance and yaw angle to tag as measurement
    - ...

EKF state: [x, y, z, roll, pitch, yaw, dx, dy, dz, droll, dpitch, dyaw]
(linear and angular velocities in body frame)
"""


class MeasurementModelDistances(object):
    def __init__(self, ekf_params: EkfParams):
        self.ekf_params = ekf_params
        self._w_mat_vision_static = np.array(
            np.diag(
                [
                    self.ekf_params.measurement_noise.distance**2,
                    self.ekf_params.measurement_noise.yaw**2,
                ]
            )
        )
        self.w_mat_orientation = np.array(
            np.diag(
                [
                    self.ekf_params.measurement_noise_orientation.roll**2,
                    self.ekf_params.measurement_noise_orientation.pitch**2,
                ]
            )
        )

    def h_vision_data(self, x_est, detected_tags):
        # measurement is: distance and yaw-angle to each tag
        num_tags = detected_tags.shape[0]
        z_est = np.zeros((num_tags * self.ekf_params.dim_meas, 1))

        for i, tag in enumerate(detected_tags):
            tag_pos = tag[1:4]
            dist = self.get_dist(x_est, tag_pos)
            yaw = x_est[5]

            z_est[i * self.ekf_params.dim_meas, 0] = dist
            z_est[i * self.ekf_params.dim_meas + 1, 0] = yaw

        return z_est  # dim [num_tags*dim_meas X 1]

    def h_jacobian_vision_data(self, x_est, detected_tags):
        num_tags = detected_tags.shape[0]
        h_mat = np.zeros(
            (num_tags * self.ekf_params.dim_meas, self.ekf_params.dim_state)
        )

        for i, tag in enumerate(detected_tags):
            tag_pos = tag[1:4]
            dist = self.get_dist(x_est, tag_pos)

            # dh /dx = 1/2 * (dist ^2)^(-1/2) * (2 * (x1 - t1) * 1)
            h_jac_x = (x_est[0] - tag_pos[0]) / dist
            # dh /dy
            h_jac_y = (x_est[1] - tag_pos[1]) / dist
            # dh /dz
            h_jac_z = (x_est[2] - tag_pos[2]) / dist
            # dh /dyaw
            h_jac_yaw = 1.0

            h_mat[self.ekf_params.dim_meas * i, 0:3] = [
                h_jac_x,
                h_jac_y,
                h_jac_z,
            ]
            h_mat[self.ekf_params.dim_meas * i + 1, 5] = h_jac_yaw
            # all other derivatives are zero

        return h_mat  # dim [num_tags*dim_meas X dim_state]

    def h_orientation_data(self, x_est):
        # measurement is: roll, pitch from /mavros/local_position/pose
        z_est = np.array([x_est[3], x_est[4]]).reshape((-1, 1))
        return z_est  # dim: [2 X 1]

    def h_jacobian_orientation_data(self):
        h_mat = np.zeros((2, self.ekf_params.dim_state))
        # all derivatives zero except for roll, pitch:
        h_mat[0, 3] = 1.0  # dh /droll
        h_mat[1, 4] = 1.0  # dh/ dpitch
        return h_mat  # dim [2 X dim_state]

    def h_imu_data(self, x_est, using_lin_acc=False):
        if not using_lin_acc:
            # measurement is: roll rate, pitch rate, yaw rate
            z_est = np.array([x_est[9], x_est[10], x_est[11]]).reshape((-1, 1))
        else:
            # measurement is: angular velocities and linear accelerations
            print(
                'Using linear acceleration measurements from IMU!'
                + 'Not implemented yet'
            )
        return z_est

    def h_jacobian_imu_data(self, using_lin_acc=False):
        if not using_lin_acc:
            h_mat = np.zeros((3, self.ekf_params.dim_state))
            # all derivatives zero except for body rates
            h_mat[0, 9] = 1.0
            h_mat[1, 10] = 1.0
            h_mat[2, 11] = 1.0
        else:
            print(
                'Using linear acceleration measurements from IMU! '
                + 'Not implemented yet'
            )

        return h_mat  # dim [3 X dim_state]

    def vision_dynamic_meas_model(self, x_est, measurements, detected_tags):
        # currently not using measured tag-position in camera coordinates,
        # but known tag position from calibration, since currently not possible
        # to (nicely) pass full tag-pose measurement to method

        num_tags = detected_tags.shape[0]
        # initialize dynamic W
        w_mat_dyn = np.zeros(
            (
                num_tags * self.ekf_params.dim_meas,
                num_tags * self.ekf_params.dim_meas,
            )
        )

        for i, _tag in enumerate(detected_tags):
            # debugging: not dynamic
            # noise for distance measurement
            w_mat_dyn[
                self.ekf_params.dim_meas * i, self.ekf_params.dim_meas * i
            ] = self._w_mat_vision_static[0, 0]
            # noise for yaw measurement
            w_mat_dyn[
                self.ekf_params.dim_meas * i + 1,
                self.ekf_params.dim_meas * i + 1,
            ] = self._w_mat_vision_static[1, 1]

        return w_mat_dyn  # dim [num_tags*dim_meas X num_tag*dim_meas]

    def get_dist(self, x_est, tag_pos):
        # dist = sqrt((x - x_tag) ^ 2 + (y - y_tag) ^ 2 + (z - z_tag) ^ 2)
        dist = np.sqrt(
            (x_est[0] - tag_pos[0]) ** 2
            + (x_est[1] - tag_pos[1]) ** 2
            + (x_est[2] - tag_pos[2]) ** 2
        )
        return dist
