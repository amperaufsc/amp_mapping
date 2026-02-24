import numpy as np
#ESSA É APENAS A BASE E NÃO ESTÁ 100% COERENTE COM O ESPERADO

class Error_Space_Kalman_Filter:
    def __init__(self, imu_update_time=0.01, dvl_update_time=0.1, slam_update_time=0.1, accelerometer_random_walk_bias = [1e-6,1e-6,1e-6], 
                 gyroscope_random_walk_bias = [1e-6,1e-6,1e-6], accelerometer_noise = [1e-6,1e-6,1e-6], gyroscope_noise = [1e-6,1e-6,1e-6],
                 corr_noise = [1e-6,1e-6,1e-6], dvl_noise = [1e-6,1e-6,1e-6], slam_noise = [1e-6,1e-6,1e-6,1e-6,1e-6,1e-6], corr_t = np.ones(3)):
        
        # Parâmetros configuráveis (Nontunable)
        self.imu_update_time = imu_update_time
        self.dvl_update_time = dvl_update_time
        self.slam_update_time = slam_update_time
        self.accelerometer_random_walk_bias = np.array(accelerometer_random_walk_bias)
        self.gyroscope_random_walk_bias = np.array(gyroscope_random_walk_bias)
        self.accelerometer_noise = np.array(accelerometer_noise)
        self.gyroscope_noise = np.array(gyroscope_noise)
        self.corr_noise = np.array(corr_noise)
        self.dvl_noise = np.array(dvl_noise)
        self.slam_noise = np.array(slam_noise)
        self.corr_t = corr_t

        # Estados internos (private)
        self.error_x = np.zeros((18, 1))
        self.covariance_error_x = np.eye(18) * 1e-8  
        self.imu_counter = 0
        self.dvl_counter = 0
        self.slam_counter = 0
        self.position = np.zeros((3, 1))
        self.velocity = np.zeros((3, 1))
        self.orientation = np.zeros((3, 1))
        self.gravity = np.array([[0], [0], [9.81]])
        self.C_n_b = np.eye(3)
        self.alfa = np.zeros((3,1))
        self.beta = np.zeros((3,1))
        self.last_gyro = np.zeros((3,1))
        self.last_alfa_delta = np.zeros((3,1))
        self.last_orientation = np.zeros((3,1))

        # SetupImpl
        self.slam_covariance = np.diag(self.slam_noise / self.slam_update_time)
        self.dvl_covariance = np.diag(self.dvl_noise / self.dvl_update_time)
        diag_elements = np.concatenate([np.ones(3) * 10e-4,
                                        self.accelerometer_noise.flatten(),
                                        self.gyroscope_noise.flatten(),
                                        self.accelerometer_random_walk_bias.flatten(),
                                        self.gyroscope_random_walk_bias.flatten(),
                                        self.corr_noise.flatten()])
        self.proccess_covariance = np.diag(diag_elements)

    def ekf_principal(self, imu, dvl, slam): #YEP
        [pos, vel, ori] = self.mechanization(imu[0:3],imu[3:6])
        self.update_imu(imu)
        self.slam_counter +=1
        self.dvl_counter +=1

        self.orientation = ori
        self.position = pos
        self.velocity = vel

        if self.dvl_counter>=(self.dvl_update_time/self.imu_update_time):
            self.updateDVL(dvl)
            self.dvl_counter = 0
            self.error_velocity = self.error_x[3:6]
            self.velocity = self.velocity + self.error_velocity
        
        if self.slam_counter >= (self.slam_update_time/self.imu_update_time):
                self.updateSLAM(slam)
                self.slam_counter = 0
                self.error_position = self.error_x[0:3]
                self.error_orientation = self.error_x[6:9]
                
                self.position = self.position + self.error_position

                self.alfa = np.zeros((3,1))
                self.beta = np.zeros((3,1))
                self.last_alfa_delta = np.zeros((3,1))
                self.last_orientation = ori + self.error_orientation
        
        return [self.position, self.velocity, self.orientation, self.error_x]

    def update_imu(self,imu): #YEP
        A_k = self.get_proccess_jacobian_matrix(self.error_x, imu, self.imu_update_time, self.corr_t)
        self.error_x = A_k @ self.error_x
        B_k = self.get_proccess_noise_jacobian_matrix(self.error_x, self.imu_update_time)
        self.covariance_error_x = A_k @ self.covariance_error_x @ A_k.T + B_k @ self.proccess_covariance @ B_k.T
        return 
    

    def update_dvl(self, dvl): #YEP
        error_lambda = self.error_x[6:9]
        R = self.C_n_b
        predicted_measurement = self.dvl_measurement(self.error_x, self.velocity, R)
        dvl_measurement = self.velocity - R @ dvl
        self.update_ekf(dvl_measurement, self.dvl_covariance, self.get_dvl_jacobian(self.error_x,self.velocity), predicted_measurement, self.get_dvl_noise_jacobian(self.error_x))
        return
        
        
    def update_slam(self, slam): #YEP
        predicted_measurement = self.slam_measurement(self.error_x)
        slam_measurement = np.vstack((self.position - slam[0:3].reshape(3,1),
                                      self.orientation - slam[3:6].reshape(3,1)))
        self.update_ekf(slam_measurement, self.slam_covariance, self.get_slam_jacobian(self.error_x), predicted_measurement, np.eye(6))
        return


    def update_ekf(self, measurement, measurement_covariance, jacobian_matrix, predicted_measurement, noise_jacobian_matrix):  #YEP
        P_ = self.covariance_error_x
        H_k1 = jacobian_matrix
        S_k1 = measurement_covariance
        M_k1 = noise_jacobian_matrix

        S = H_k1 @ P_ @ H_k1.T + M_k1 @ S_k1 @ M_k1.T
        K_k1 = P_ @ H_k1.T @ np.linalg.inv(S)

        error_z = measurement - predicted_measurement
        self.error_x = self.error_x + K_k1 @ (error_z - H_k1 @ self.error_x);
        self.covariance_error_x = (np.eye(18) - K_k1 @ H_k1) @ P_
        return


    def mechanization(self, accel, gyro): 
            velocity_dot = self.C_n_b @ accel + self.gravity
            alfa_delta = (gyro+ self.last_gyro)/2*self.imu_update_time
            beta_delta = 0.5 * np.cross((self.alfa + 1/6*self.last_alfa_delta), alfa_delta)
            
            self.alfa = self.alfa + alfa_delta
            self.beta = self.beta + beta_delta

            self.last_gyro = gyro
            self.last_alfa = self.alfa
            self.last_beta = self.beta
            self.last_alfa_delta = alfa_delta

            position = self.position + self.velocity*self.imu_update_time
            velocity =  self.velocity + velocity_dot*self.imu_update_time
            orientation =  self.alfa + self.beta + self.last_orientation

            self.C_n_b = self.euler_to_rot(self.orientation) #verificar isso aq usa o valor recem calculado? 
            return (position, velocity, orientation)


    def get_rotation_matrix(self, error_lambda): #YEP
        matrix_error_lambda = self.get_skew_symmetric_matrix(error_lambda)
        return np.eye(3) + matrix_error_lambda
    

    def dvl_measurement(self, error_state, v_nom, R): #YEP
        matrix_vnom = self.get_skew_symmetric_matrix(v_nom)
        return (- error_state[3:6] - matrix_vnom @ error_state[6:9] + R @ error_state[15:18])


    def slam_measurement(self, error_state): #YEP
        return -np.vstack((error_state[0:3], error_state[6:9])).reshape(-1,1)


    def get_slam_jacobian(self, x0): #YEP
        jacobian = np.array([
                            [-1,  0,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0, -1,  0, 0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0,  0, -1, 0, 0, 0,  0,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0,  0,  0, 0, 0, 0, -1,  0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0,  0,  0, 0, 0, 0,  0, -1,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                            [ 0,  0,  0, 0, 0, 0,  0,  0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        return jacobian


    def get_dvl_jacobian(self, x0, v_nom): #YEP
        v1 = x0[3]
        v2 = x0[4]
        v3 = x0[5]
        lambda1 = x0[6]
        lambda2 = x0[7]
        lambda3 = x0[8]
        bv1 = x0[15]
        bv2 = x0[16]
        bv3 = x0[17]
        v_nom1 = v_nom[0]
        v_nom2 = v_nom[1]
        v_nom3 = v_nom[2]

        jacobian = np.array([
                            [0, 0, 0, -1,  0,  0,          bv2, bv3 - v_nom1,      -v_nom2, 0, 0, 0, 0, 0, 0,        1,  lambda1, lambda2],
                            [0, 0, 0,  0, -1,  0, v_nom1 - bv1,            0, bv3 - v_nom3, 0, 0, 0, 0, 0, 0, -lambda1,        1, lambda3],
                            [0, 0, 0,  0,  0, -1,       v_nom2, v_nom3 - bv1,         -bv2, 0, 0, 0, 0, 0, 0, -lambda2, -lambda3,       1]])
        return jacobian


    def get_dvl_noise_jacobian(self, x0): #YEP
        lambda1 = x0[6]
        lambda2 = x0[7]
        lambda3 = x0[8]

        jacobian = np.array([
                            [       1,  lambda1, lambda2],
                            [-lambda1,        1, lambda3],
                            [-lambda2, -lambda3,       1]])
        return jacobian

    def get_rodrigues_rotation_matrix(self, attitude_error): #YEP
        norm_aterror = np.linalg.norm(attitude_error)
        if norm_aterror < 1e-8:
            return np.eye(3)
        
        skew = self.get_skew_symmetric_matrix(attitude_error)

        sin_component = np.sin(norm_aterror)/norm_aterror
        cos_component = (1-np.cos(norm_aterror))/(norm_aterror**2)
        R = np.eye(3) + sin_component*skew+cos_component*(skew @ skew)
        return R


    def get_proccess_jacobian_matrix(self,x0,u,dt,T): # YEP
        a1 =u[0]
        a2 =u[1]
        a3 =u[2]
        v1 = x0[3]
        v2 = x0[4]
        v3 = x0[5]
        lambda1 = x0[6]
        lambda2 = x0[7]
        lambda3 = x0[8]
        bw1 = x0[12]
        bw2 = x0[13]
        bw3 = x0[14]
        t1 = T[0]
        t2 = T[1]
        t3 = T[2]

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


    def get_proccess_noise_jacobian_matrix(self,x0,dt): # YEP
        lambda1 = x0[6]
        lambda2 = x0[7]
        lambda3 = x0[8]

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


    def get_skew_symmetric_matrix(self, vector): # YEP
        # função que acha a maatriz antissimétrica
        vector_x, vector_y, vector_z = vector
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
