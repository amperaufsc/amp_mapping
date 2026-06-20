import numpy as np

class Error_Space_Kalman_Filter:
    def __init__(self,imu_update_time=0.01, dvl_update_time=0.1, slam_update_time=0.1, accelerometer_random_walk_bias = [1e-6,1e-6,1e-6],
                 gyroscope_random_walk_bias = [1e-6,1e-6,1e-6], accelerometer_noise = [1e-6,1e-6,1e-6], gyroscope_noise = [1e-6,1e-6,1e-6],
                 corr_noise = [1e-6,1e-6,1e-6], dvl_noise = [1e-6,1e-6,1e-6], slam_noise = [1e-6,1e-6,1e-6,1e-6,1e-6,1e-6], corr_t = np.ones(3)):
        
        # Parâmetros configuráveis (Nontunable)
        self.imu_update_time = imu_update_time
        self.dvl_update_time = dvl_update_time
        self.slam_update_time = slam_update_time
        self.accelerometer_random_walk_bias = np.array(accelerometer_random_walk_bias).reshape(-1,1)
        self.gyroscope_random_walk_bias = np.array(gyroscope_random_walk_bias).reshape(-1,1)
        self.accelerometer_noise = np.array(accelerometer_noise).reshape(-1,1)
        self.gyroscope_noise = np.array(gyroscope_noise).reshape(-1,1)
        self.corr_noise = np.array(corr_noise).reshape(-1,1)
        self.dvl_noise = np.array(dvl_noise).reshape(-1,1)
        self.slam_noise = np.array(slam_noise).reshape(-1,1)
        self.corr_t = corr_t.reshape(-1,1)

        # Estados internos (private)
        # Todos os vetores estão como coluna (Nx1)
        self.error_x = np.zeros((18,1), dtype=float)
        self.covariance_error_x = np.eye(18) * 1e-8  
        self.imu_counter = 0
        self.dvl_counter = 0
        self.slam_counter = 0
        self.position = np.zeros((3,1))
        self.velocity = np.zeros((3,1))
        self.orientation = np.zeros((3,1))
        self.gravity = np.array([[0], [0], [9.81]])
        self.C_n_b = np.eye(3)
        self.alfa = np.zeros((3,1))
        self.beta = np.zeros((3,1))
        self.last_gyro = np.zeros((3,1))
        self.last_alfa_delta = np.zeros((3,1))
        self.last_orientation = np.zeros((3,1))

        # SetupImpl
        self.slam_covariance = np.diag(self.slam_noise.reshape(-1) / self.slam_update_time)
        self.dvl_covariance = np.diag(self.dvl_noise.reshape(-1) / self.dvl_update_time)  # (3x3)
        diag_elements = np.concatenate([np.ones(3) * 10e-4,
                                        self.accelerometer_noise.flatten(),
                                        self.gyroscope_noise.flatten(),
                                        self.accelerometer_random_walk_bias.flatten(),
                                        self.gyroscope_random_walk_bias.flatten(),
                                        self.corr_noise.flatten()])
        self.proccess_covariance = np.diag(diag_elements)

    def ekf_principal(self, imu, dvl, slam): #
        [pos, vel, ori] = self.mechanization(imu[0:3],imu[3:6])
        print('_________________')
        print('pos:',pos)
        print('vel:',vel)
        print('ori:',ori)
        self.update_imu(imu)
        self.slam_counter +=1
        self.dvl_counter +=1

        self.orientation = ori
        self.position = pos
        self.velocity = vel

        if self.dvl_counter>=(self.dvl_update_time/self.imu_update_time):
            print('==========================')
            print('DDDDDDDDDDVVVVVVVLLLLLLLLL')
            self.update_dvl(dvl)
            self.dvl_counter = 0
            self.error_velocity = self.error_x[3:6]
            self.velocity = self.velocity + self.error_velocity
        
        if self.slam_counter >= (self.slam_update_time/self.imu_update_time):
                print('==========================')
                print('SSSSSSLLLLLLLAAAAAAMMMMMMM')
                self.update_slam(slam)
                self.slam_counter = 0
                self.error_position = self.error_x[0:3]
                self.error_orientation = self.error_x[6:9]
                
                self.position = self.position + self.error_position

                self.alfa = np.zeros((3,1))
                self.beta = np.zeros((3,1))
                self.last_alfa_delta = np.zeros((3,1))
                self.last_orientation = ori + self.error_orientation
        
        return [self.position, self.velocity, self.orientation]  #

    def update_imu(self,imu): #
        A_k = self.get_proccess_jacobian_matrix(self.error_x, imu, self.imu_update_time, self.corr_t)
        self.error_x = A_k @ self.error_x
        B_k = self.get_proccess_noise_jacobian_matrix(self.error_x, self.imu_update_time)
        self.covariance_error_x = A_k @ self.covariance_error_x @ A_k.T + B_k @ self.proccess_covariance @ B_k.T
        return 
    

    def update_dvl(self, dvl): #
        error_lambda = self.error_x[6:9]
        R = self.C_n_b #(3x3)

        predicted_measurement = self.dvl_measurement(self.error_x, self.velocity, R) #(3x1)
        dvl_measurement = self.velocity - R @ dvl # (3x1) - (3x3)@(3x1) = (3x1)
        self.update_ekf(dvl_measurement, self.dvl_covariance, self.get_dvl_jacobian(self.error_x,self.velocity), predicted_measurement, self.get_dvl_noise_jacobian(self.error_x))
        #                  (3,1),                (3x3),              (3X18)                                          (3x1)                         (3x3)
        return
        
        
    def update_slam(self, slam): # slam = (3x1)
        predicted_measurement = self.slam_measurement(self.error_x)
        slam_measurement = np.vstack((self.position - slam[0:3],
                                      self.orientation - slam[3:6]))
        self.update_ekf(slam_measurement, self.slam_covariance, self.get_slam_jacobian(self.error_x), predicted_measurement, np.eye(6))
        return


    def update_ekf(self, measurement, measurement_covariance, jacobian_matrix, predicted_measurement, noise_jacobian_matrix): # (3,1), (3x3), (3X18), (3x1), (3x3)
        P_ = self.covariance_error_x  # (18x18)
        H_k1 = jacobian_matrix        # (3,18)
        S_k1 = measurement_covariance # (3x3)
        M_k1 = noise_jacobian_matrix  # (3x3)

        S = H_k1 @ P_ @ H_k1.T + M_k1 @ S_k1 @ M_k1.T
        K_k1 = P_ @ H_k1.T @ np.linalg.inv(S)

        error_z = measurement - predicted_measurement
        self.error_x = self.error_x + K_k1 @ (error_z - H_k1 @ self.error_x);
        self.covariance_error_x = (np.eye(18) - K_k1 @ H_k1) @ P_
        return


    def mechanization(self, accel, gyro): 
            velocity_dot = self.C_n_b @ accel + self.gravity # (3x3)@(3x1) + (3x1) = (3x1)
            alfa_delta = (gyro + self.last_gyro)/2*self.imu_update_time
            d = self.alfa + 1/6*self.last_alfa_delta
            beta_delta = 0.5 * np.cross(d.ravel(), alfa_delta.ravel()).reshape(3,1)
            
            self.alfa = self.alfa + alfa_delta
            self.beta = self.beta + beta_delta

            self.last_gyro = gyro
            self.last_alfa = self.alfa
            self.last_beta = self.beta
            self.last_alfa_delta = alfa_delta

            position = self.position + self.velocity*self.imu_update_time
            velocity =  self.velocity + velocity_dot*self.imu_update_time
            orientation =  self.alfa + self.beta + self.last_orientation

            self.C_n_b = self.euler_to_rot(self.orientation) 
            return (position, velocity, orientation)


    def get_rotation_matrix(self, error_lambda): #
        matrix_error_lambda = self.get_skew_symmetric_matrix(error_lambda)
        return np.eye(3) + matrix_error_lambda
    

    def dvl_measurement(self, error_state, v_nom, R): # (18x1), (3x1), (3x3)
        matrix_vnom = self.get_skew_symmetric_matrix(v_nom)   #(3x3)    
        # -(3x1) - (3x3)@(3x1) + (3x3)@(3x1) = (3x1)
        return (- error_state[3:6] - matrix_vnom @ error_state[6:9] + R @ error_state[15:18])


    def slam_measurement(self, error_state): #YEP - VECTOR (6x1)
        return -np.vstack((error_state[0:3], error_state[6:9]))


    def get_slam_jacobian(self, x0): #YEP - VECTOR (6X18)
        jacobian = np.array([
                            [-1,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, -1,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0,  0, -1, 0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0,  0,  0, 0, 0, 0, -1,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0,  0,  0, 0, 0, 0,  0, -1,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0,  0,  0, 0, 0, 0,  0,  0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        return jacobian


    def get_dvl_jacobian(self, x0, v_nom): #YEP - VECTOR (3X18)
        v1 = x0[3][0]
        v2 = x0[4][0]
        v3 = x0[5][0]
        lambda1 = x0[6][0]
        lambda2 = x0[7][0]
        lambda3 = x0[8][0]
        bv1 = x0[15][0]
        bv2 = x0[16][0]
        bv3 = x0[17][0]
        v_nom1 = v_nom[0][0]
        v_nom2 = v_nom[1][0]
        v_nom3 = v_nom[2][0]

        jacobian = np.array([
                            [0, 0, 0, -1,  0,  0,          bv2, bv3 - v_nom1,      -v_nom2, 0, 0, 0, 0, 0, 0,        1,  lambda1, lambda2],
                            [0, 0, 0,  0, -1,  0, v_nom1 - bv1,            0, bv3 - v_nom3, 0, 0, 0, 0, 0, 0, -lambda1,        1, lambda3],
                            [0, 0, 0,  0,  0, -1,       v_nom2, v_nom3 - bv1,         -bv2, 0, 0, 0, 0, 0, 0, -lambda2, -lambda3,       1]])
        return jacobian


    def get_dvl_noise_jacobian(self, x0): #YEP - VECTOR (3X3)
        lambda1 = x0[6][0]
        lambda2 = x0[7][0]
        lambda3 = x0[8][0]

        jacobian = np.array([
                            [       1,  lambda1, lambda2],
                            [-lambda1,        1, lambda3],
                            [-lambda2, -lambda3,       1]])
        return jacobian

    def get_rodrigues_rotation_matrix(self, attitude_error): #YEP - vECTOR (3X3)
        norm_aterror = np.linalg.norm(attitude_error)
        if norm_aterror < 1e-8:
            return np.eye(3)
        
        skew = self.get_skew_symmetric_matrix(attitude_error)

        sin_component = np.sin(norm_aterror)/norm_aterror
        cos_component = (1-np.cos(norm_aterror))/(norm_aterror**2)
        R = np.eye(3) + sin_component*skew+cos_component*(skew @ skew)
        return R


    def get_proccess_jacobian_matrix(self,x0,u,dt,T): # YEP - VECTOR (18X18)
        a1 =u[0][0]
        a2 =u[1][0]
        a3 =u[2][0]
        v1 = x0[3][0]
        v2 = x0[4][0]
        v3 = x0[5][0]
        lambda1 = x0[6][0]
        lambda2 = x0[7][0]
        lambda3 = x0[8][0]
        bw1 = x0[12][0]
        bw2 = x0[13][0]
        bw3 = x0[14][0]
        t1 = T[0][0]
        t2 = T[1][0]
        t3 = T[2][0]

        jacobian = np.array([
                            [1, 0, 0,         dt,           0,           0,                                        0,                                         0,                                         0, 0, 0, 0,          0,           0,           0,         0,         0,         0],
                            [0, 1, 0,          0,          dt,           0,                                        0,                                         0,                                         0, 0, 0, 0,          0,           0,           0,         0,         0,         0],
                            [0, 0, 1,          0,           0,          dt,                                        0,                                         0,                                         0, 0, 0, 0,          0,           0,           0,         0,         0,         0],
                            [0, 0, 0,     1 - dt, -dt*lambda1, -dt*lambda2,       -dt*(v2 - a1*lambda3 + a2*lambda2), -dt*(a1 + v3 + a2*lambda1 + 2*a3*lambda2),      -dt*(a2 - a1*lambda1 + 2*a3*lambda3), 0, 0, 0,          0,           0,           0,         0,         0,         0],
                            [0, 0, 0, dt*lambda1,      1 - dt, -dt*lambda3, dt*(a1 + v1 + 2*a2*lambda1 + a3*lambda2),              dt*(a1*lambda3 + a3*lambda1), -dt*(a3 + v3 - a1*lambda2 - 2*a2*lambda3), 0, 0, 0,          0,           0,           0,         0,         0,         0],
                            [0, 0, 0, dt*lambda2,  dt*lambda3,      1 - dt,      dt*(a2 - 2*a1*lambda1 + a3*lambda3),  dt*(a3 + v1 - 2*a1*lambda2 - a2*lambda3),         dt*(v2 - a2*lambda2 + a3*lambda1), 0, 0, 0,          0,           0,           0,         0,         0,         0],
                            [0, 0, 0,          0,           0,           0,                               1 - bw2*dt,                                   -bw3*dt,                                         0, 0, 0, 0,        -dt, -dt*lambda1, -dt*lambda2,         0,         0,         0],
                            [0, 0, 0,          0,           0,           0,                                   bw1*dt,                                         1,                                   -bw3*dt, 0, 0, 0, dt*lambda1,         -dt, -dt*lambda3,         0,         0,         0],
                            [0, 0, 0,          0,           0,           0,                                        0,                                    bw1*dt,                                bw2*dt + 1, 0, 0, 0, dt*lambda2,  dt*lambda3,         -dt,         0,         0,         0],
                            [0, 0, 0,          0,           0,           0,                                        0,                                         0,                                         0, 1, 0, 0,          0,           0,           0,         0,         0,         0],
                            [0, 0, 0,          0,           0,           0,                                        0,                                         0,                                         0, 0, 1, 0,          0,           0,           0,         0,         0,         0],
                            [0, 0, 0,          0,           0,           0,                                        0,                                         0,                                         0, 0, 0, 1,          0,           0,           0,         0,         0,         0],
                            [0, 0, 0,          0,           0,           0,                                        0,                                         0,                                         0, 0, 0, 0,          1,           0,           0,         0,         0,         0],
                            [0, 0, 0,          0,           0,           0,                                        0,                                         0,                                         0, 0, 0, 0,          0,           1,           0,         0,         0,         0],
                            [0, 0, 0,          0,           0,           0,                                        0,                                         0,                                         0, 0, 0, 0,          0,           0,           1,         0,         0,         0],
                            [0, 0, 0,          0,           0,           0,                                        0,                                         0,                                         0, 0, 0, 0,          0,           0,           0, 1 - dt/t1,         0,         0],
                            [0, 0, 0,          0,           0,           0,                                        0,                                         0,                                         0, 0, 0, 0,          0,           0,           0,         0, 1 - dt/t2,         0],
                            [0, 0, 0,          0,           0,           0,                                        0,                                         0,                                         0, 0, 0, 0,          0,           0,           0,         0,         0, 1 - dt/t3]])
        return jacobian


    def get_proccess_noise_jacobian_matrix(self,x0,dt): # YEP - VECTOR (18x18)
        lambda1 = x0[6][0]
        lambda2 = x0[7][0]
        lambda3 = x0[8][0]

        jacobian = np.array([
                            [dt + 1,      0,      0,           0,           0,          0,           0,           0,          0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
                            [     0, dt + 1,      0,           0,           0,          0,           0,           0,          0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
                            [     0,      0, dt + 1,           0,           0,          0,           0,           0,          0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
                            [     0,      0,      0,      dt + 1,  dt*lambda1, dt*lambda2,           0,           0,          0,      0,      0,      0,      0,      0,      0,      0,      0,      0],           
                            [     0,      0,      0, -dt*lambda1,      dt + 1, dt*lambda3,           0,           0,          0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
                            [     0,      0,      0, -dt*lambda2, -dt*lambda3,     dt + 1,           0,           0,          0,      0,      0,      0,      0,      0,      0,      0,      0,      0],
                            [     0,      0,      0,           0,           0,          0,      dt + 1,  dt*lambda1, dt*lambda2,      0,      0,      0,      0,      0,      0,      0,      0,      0],
                            [     0,      0,      0,           0,           0,          0, -dt*lambda1,      dt + 1, dt*lambda3,      0,      0,      0,      0,      0,      0,      0,      0,      0],
                            [     0,      0,      0,           0,           0,          0, -dt*lambda2, -dt*lambda3,     dt + 1,      0,      0,      0,      0,      0,      0,      0,      0,      0],
                            [     0,      0,      0,           0,           0,          0,           0,           0,          0, 1 - dt,      0,      0,      0,      0,      0,      0,      0,      0],
                            [     0,      0,      0,           0,           0,          0,           0,           0,          0,      0, 1 - dt,      0,      0,      0,      0,      0,      0,      0],
                            [     0,      0,      0,           0,           0,          0,           0,           0,          0,      0,      0, 1 - dt,      0,      0,      0,      0,      0,      0],
                            [     0,      0,      0,           0,           0,          0,           0,           0,          0,      0,      0,      0, 1 - dt,      0,      0,      0,      0,      0],
                            [     0,      0,      0,           0,           0,          0,           0,           0,          0,      0,      0,      0,      0, 1 - dt,      0,      0,      0,      0],
                            [     0,      0,      0,           0,           0,          0,           0,           0,          0,      0,      0,      0,      0,      0, 1 - dt,      0,      0,      0],
                            [     0,      0,      0,           0,           0,          0,           0,           0,          0,      0,      0,      0,      0,      0,      0, dt + 1,      0,      0],
                            [     0,      0,      0,           0,           0,          0,           0,           0,          0,      0,      0,      0,      0,      0,      0,      0, dt + 1,      0],
                            [     0,      0,      0,           0,           0,          0,           0,           0,          0,      0,      0,      0,      0,      0,      0,      0,      0, dt + 1]])
        return jacobian


    def get_skew_symmetric_matrix(self, vector): # YEP - VECTOR (3x3)
        # função que acha a maatriz antissimétrica
        vector_x = vector[0][0]
        vector_y = vector[1][0]
        vector_z = vector[2][0]
        return np.array([[ 0,  -vector_z,  vector_y],
                         [vector_z,  0,  -vector_x],
                         [-vector_y, vector_x,  0]])
    
    def euler_to_rot(self, euler):
        phi, theta, psi = np.asarray(euler).flatten()
        # Rotação em X (roll)
        Rx = np.array([
            [1,           0,            0],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi),  np.cos(phi)]])
        # Rotação em Y (pitch)
        Ry = np.array([
            [ np.cos(theta), 0, np.sin(theta)],
            [             0, 1,             0],
            [-np.sin(theta), 0, np.cos(theta)]])
        # Rotação em Z (yaw)
        Rz = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi),  np.cos(psi), 0],
            [          0,            0, 1]])
        return Rz @ Ry @ Rx
