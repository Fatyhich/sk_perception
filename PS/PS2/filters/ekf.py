"""
This file implements the Extended Kalman Filter.
"""

import numpy as np

from filters.localization_filter import LocalizationFilter
from tools.task import get_motion_noise_covariance
from tools.task import get_observation as get_expected_observation
from tools.task import get_prediction
from tools.task import wrap_angle


class EKF(LocalizationFilter):
    def jacobian_state(self, action: np.ndarray):
        drot1, dtran, _ = action
        wraped_arg = wrap_angle(self.mu[2] + drot1)

        return np.array([
            [1, 0, -dtran * np.sin(wraped_arg)],
            [0, 1, dtran * np.cos(wraped_arg)],
            [0, 0, 1]
        ])
    
    def jacobian_action(self, action: np.ndarray):
        drot1, dtran, drot2 = action
        wraped_arg = wrap_angle(self.mu[2] + drot1)

        return np.array([
            [-(dtran*(np.sin(wraped_arg))), np.cos(wraped_arg), 0],
            [dtran*(np.cos(wraped_arg)), np.sin(wraped_arg), 0],
            [1, 0, 1]
        ])
    
    def jacobian_obs(self, state: np.ndarray, lm_id: int):
        from field_map import FieldMap
        field_map = FieldMap()
        
        dx = field_map.landmarks_poses_x[lm_id] - state[0]
        dy = field_map.landmarks_poses_y[lm_id] - state[1]
        norm_coeff = 1/(dx**2 + dy**2)

        return np.array([
            [dy*norm_coeff, -dx*norm_coeff, -1]
        ])

    def predict(self, u):
        # TODO Implement here the EKF, perdiction part. HINT: use the auxiliary functions imported above from tools.task
        G = self.jacobian_state(u)
        V = self.jacobian_action(u)
        M = get_motion_noise_covariance(u, self._alphas)

        self._state_bar.mu = get_prediction(self._state.mu.flatten(), u)[np.newaxis].T
        self._state_bar.Sigma = G @ self._state.Sigma @ G.T + V @ M @ V.T

    def update(self, z):
        Q = 0.1225

        for i in range(0, len(z), 2):
            bearing = z[i]
            landmark_id = int(z[i + 1])

            z_bar = get_expected_observation(self._state_bar.mu.flatten(), landmark_id)
            H = self.jacobian_obs(self._state_bar.mu.flatten(), landmark_id)
            S = H @ self._state_bar.Sigma @ H.T + Q
            K = self._state_bar.Sigma @ H.T @ np.linalg.inv(S)
            angle_diff = np.array([[wrap_angle(bearing - z_bar[0])]])
            self._state.mu = self._state_bar.mu + K @ angle_diff
            self._state.Sigma = (np.eye(3) - K @ H) @ self._state_bar.Sigma
