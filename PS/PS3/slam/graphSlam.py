"""
Gonzalo Ferrer
g.ferrer@skoltech.ru
28-Feb-2021
"""

import numpy as np
import mrob
from scipy.linalg import inv
from slam.slamBase import SlamBase
from tools.task import get_motion_noise_covariance

class GraphSLAM(SlamBase):
    def __init__(self, initial_state, alphas,
                 state_dim=3, obs_dim=2, landmark_dim=2,
                 action_dim=3, *args, **kwargs):
        super(GraphSLAM, self).__init__(*args, **kwargs)
        self.state_dim = state_dim
        self.landmark_dim = landmark_dim
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.alphas = alphas
        self.Q = kwargs.get('Q')
        
        self.graph = mrob.FGraph()

        x0 = initial_state.mu
        self.x0_node_id = self.graph.add_node_pose_2d(x0)

        W = inv(initial_state.Sigma)
        self.graph.add_factor_1pose_2d(x0, self.x0_node_id, W)

        print("Initial Graph State:")
        self.graph.print(True)
        

    def predict(self, u):
        # Print state before adding
        print("Graph state BEFORE adding odometry factor:")
        estimated_state_before = self.graph.get_estimated_state()
        print(estimated_state_before)

        # Add new nod
        new_node_id = self.graph.add_node_pose_2d(np.zeros(3))

        # Information matrix
        M_t = get_motion_noise_covariance(u, self.alphas)
        W_u = np.linalg.inv(M_t)

        # Add odometry factor
        self.graph.add_factor_2poses_2d_odom(u, self.x0_node_id, new_node_id, W_u)
        self.x0_node_id = new_node_id

        # Print state after adding
        print("Graph state after adding odometry factor:")
        estimated_state = self.graph.get_estimated_state()
        print(estimated_state)

    def update(self, z):
        if not hasattr(self, 'landmark_node_ids'):
            self.landmark_node_ids = {}

        # Information matrix
        W_z = np.linalg.inv(self.Q)

        for i in range(z.shape[0]):
            if np.isnan(z[i, 0]):
                continue
            
            landmark_id = int(z[i, 2])

            # New landmark
            if landmark_id not in self.landmark_node_ids:
                node_id = self.graph.add_node_landmark_2d(np.zeros(2))
                self.landmark_node_ids[landmark_id] = node_id

                self.graph.add_factor_1pose_1landmark_2d(
                    z[i, :2],  #range & bearing
                    self.x0_node_id,  # ID current pose
                    node_id,  # ID landmark node
                    W_z,  # Information matrix
                    initializeLandmark=True  # Automatic landmark initialization
                )
            else:
                # Landmark already observed
                self.graph.add_factor_1pose_1landmark_2d(
                    z[i, :2],
                    self.x0_node_id,
                    self.landmark_node_ids[landmark_id],
                    W_z,
                    initializeLandmark=False
                )

        print("\nGraph after adding landmark observations:")
        self.graph.print(True)

        estimated_state = self.graph.get_estimated_state()
        print("\nEstimated state (poses and landmarks):")
        print(estimated_state)
        # print("++++++++++++++=============++++++++++++++")

    def solve(self):
        # Print state before optimization
        # print("\nGraph state BEFORE optimization:")
        # self.graph.print(True)
        
        # Perform one iteration of the Gauss-Newton algorithm
        chi2_error = self.graph.solve(mrob.GN)
        
        # Print graph state after optimization
        # print("\nGraph state AFTER optimization:")
        # self.graph.print(True)
        
        # Print estimated state after optimization
        estimated_state = self.graph.get_estimated_state()
        print("\nEstimated state after optimization (poses and landmarks):")
        print(estimated_state)
        
        # Print chi-square error
        print(f"\nChi-square error after optimization: {chi2_error}")
        
        return chi2_error
    
    def manual_solve(self):
        estimated_state_before = self.graph.get_estimated_state()
        print("\nEstimated state before hand optimization:")
        print(estimated_state_before)

        x0 = np.concatenate([state.flatten() for state in estimated_state_before])

        A = self.graph.get_adjacency_matrix()
        W = self.graph.get_W_matrix()

        information_matrix_manual = A.T @ W @ A
        information_matrix_mrob = self.graph.get_information_matrix()

        matrix_diff_norm = (information_matrix_manual -
                            information_matrix_mrob).toarray()
        matrix_diff_norm = np.linalg.norm(matrix_diff_norm)

        print("\nManual calculation results:")
        print(f"Matrix difference norm: {matrix_diff_norm}")

        if matrix_diff_norm < 1e-10:
            print("The manually calculated information matrix matches the one from mrob!")
        else:
            print("The matrices don't match exactly. There might be numerical differences.")

        chi2_error = self.graph.solve(mrob.GN)

        estimated_state_after = self.graph.get_estimated_state()
        print("\nEstimated state after optimization:")
        print(estimated_state_after)

        print(f"\nChi-square error after optimization: {chi2_error}")

        return chi2_error
    
    def solve_normal_equation(self):
        estimated_state_before = self.graph.get_estimated_state()
        x0 = np.concatenate([state.flatten() for state in estimated_state_before])

        A = self.graph.get_adjacency_matrix()
        W = self.graph.get_W_matrix()

        b = self.graph.get_vector_b()
        information_matrix = A.T @ W @ A

        information_matrix_dense = information_matrix.toarray()

        try:
            dx = np.linalg.solve(information_matrix_dense, b)
        except np.linalg.LinAlgError:
            # If singularity
            dx = np.linalg.pinv(information_matrix_dense) @ b

        x_manual = x0 + dx
        chi2_error = self.graph.solve(mrob.GN)

        estimated_state_after = self.graph.get_estimated_state()
        x_mrob = np.concatenate([state.flatten() for state in estimated_state_after])

        solution_diff_norm = np.linalg.norm(x_manual - x_mrob)

        print("\nNormal Equation Solution Results:")
        print(f"Manual solution: {x_manual}")
        print(f"MROB solution: {x_mrob}")
        print(f"Solution difference norm: {solution_diff_norm}")

        if solution_diff_norm < 1e-10:
            print("The manually calculated solution matches the one from mrob!")
        else:
            print("The solutions don't match exactly. There might be numerical differences.")

        print(f"\nChi-square error after optimization: {chi2_error}")

        return chi2_error
    
