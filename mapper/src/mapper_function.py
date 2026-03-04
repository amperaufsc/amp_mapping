import time

from typing import List
import numpy as np
from map import Map
from obstacle import Obstacle
from src.vehicle_state import VehicleState
import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

pkg_share = Path(get_package_share_directory("path_planning"))
with open(pkg_share / 'config'/ 'mapper_parameters.yaml') as f:
    data = yaml.safe_load(f)
parameters = data["AMP"]["mapper_node"]["ros__parameters"]

class LuisLopesMappingMethod():
    def __init__(self, initial_state: VehicleState, gamma = parameters["gamma"],
                  saturation_error = parameters["saturation_error"], last_time_limit = parameters["last_time_limit"]) -> None:
        
        self.last_state = initial_state
        self.last_observations = []
        self.gamma = gamma
        self.landmarks = None
        self.saturation_error = saturation_error
        self.waiting_list = []
        self.iteration_number = 0
        self.last_time_limit = last_time_limit

    def covariance_matrix(self, landmark: Obstacle):
        landmark_covariance_matrix = np.array([[landmark.deviation**2,0], 
                                               [0,landmark.deviation**2]])
        return landmark_covariance_matrix

    def innovation_covariance_matrix(self, landmark:Obstacle):

        landmark_covariance_matrix = self.covariance_matrix(landmark)
        R_t = landmark_covariance_matrix
        G_theta = self.g_theta(landmark)
        Q_t = R_t + G_theta @ landmark_covariance_matrix @ G_theta.T
        return Q_t
    
    def g_theta(self, landmark:Obstacle):
        aux = np.sqrt(2)*landmark.deviation*landmark.deviation

        G_theta = np.array([[aux                , aux],
                            [-landmark.deviation, landmark.deviation]])
        return G_theta


    def individual_compatibility(self, observation: Obstacle, landmark: Obstacle):
        observation_position = np.array([observation.x, observation.y])
        landmark_position = np.array([landmark.x, landmark.y])

        Q_t = self.innovation_covariance_matrix(landmark)

        delta_position = observation_position-landmark_position

        individual_compatibility = 1/2*(delta_position.T) @ np.linalg.inv(Q_t)@ delta_position

        return individual_compatibility
    
    def likelihood(self, observation: Obstacle, landmark: Obstacle):
        Q_t = self.innovation_covariance_matrix(landmark)
        Q_det = np.linalg.det(Q_t)
        individual_compatibility = self.individual_compatibility(observation, landmark)
        likelihood = (1/(2*np.pi*np.sqrt(Q_det)))*np.exp(-individual_compatibility)

        return likelihood

    def maximum_likelihood(self, observation: Obstacle, landmarks:List[Obstacle]) -> int:
        likelihood_list = [self.likelihood(observation, landmark) for n, landmark in enumerate(landmarks)]
        return np.argmax(likelihood_list)
    
    def extended_kalman_filter(self, observation:Obstacle, landmark:Obstacle) -> Obstacle:
        E_last = self.covariance_matrix(landmark)
        G_theta = self.g_theta(landmark)
        Q_t = self.innovation_covariance_matrix(landmark)

        I = np.identity(2)


        K_t = np.linalg.multi_dot([E_last, G_theta, Q_t])
        
        landmark_position_last = np.array([landmark.x, landmark.y])
        observation_position = np.array([observation.x, observation.y])

        landmark_position = landmark_position_last + np.matmul(K_t, 
                                                               (observation_position - landmark_position_last))
        
        E_t = np.matmul((I - np.matmul(K_t, G_theta)), E_last)

        if E_t[0][0] <= self.saturation_error:
            E_t = np.array([[self.saturation_error,0], [0, self.saturation_error]])

        landmark.update(landmark_position[0], landmark_position[1], E_t[0][0])

    def filter_list_by_obstacle(self, obstacle_list: List[Obstacle], obstacle) -> List[Obstacle]:
        filtered_list = [obs for obs in obstacle_list if self.individual_compatibility(obstacle,obs) < self.gamma]
        return filtered_list
    
    def update_waiting_list(self):
        self.waiting_list = [wait for wait in self.waiting_list 
                             if time.perf_counter()-wait.last_observed_timestamp < self.last_time_limit]

    def update_batch_and_wait(self,observations:List[Obstacle]) -> List[Obstacle]:
        batch = list()
        for observation in observations:
            filtered_wait = self.filter_list_by_obstacle(self.waiting_list, observation)
            if not bool(filtered_wait):
                self.waiting_list.append(observation)
            else:
                batch.append(filtered_wait[0])
        return observations
    
    def calculate_obstacles_in_global_frame(self, state:VehicleState, observations:List[Obstacle]):
        observations_in_global_frame = list()
        for observation in observations:
            observation_vector = np.array([observation.x, observation.y, 1]).T
            transformation = np.array([[np.cos(state.yaw), -np.sin(state.yaw), state.x_position],
                                       [np.sin(state.yaw), np.cos(state.yaw),  state.y_position],
                                       [0,                 0,                  1               ]])
            new_position = np.matmul(transformation,observation_vector)
            observations_in_global_frame.append(Obstacle(new_position[0], new_position[1], observation.confidence, 
                                                         observation.label, observation.deviation, observation.count,
                                                         observation.id, observation.last_observed_timestamp))
        return observations_in_global_frame
    
    def get_data_association(self, new_observations:List[Obstacle], state:VehicleState, landmarks: List[Obstacle]) -> List[int]:
        detections = new_observations.copy()
        observations = self.calculate_obstacles_in_global_frame(state, detections)
        last_batch = observations.copy()
        data_association = list()

        if len(last_batch) > 0 :
            for i,observation in enumerate(last_batch):

                passed = False
                for j,landmark in enumerate(landmarks):
                    ic = self.individual_compatibility(observation, landmark)
                    if ic <= self.gamma:
                        passed = True
                        break

                if passed:
                    data_association.append([i,landmarks[self.maximum_likelihood(observation, landmarks)].id])
        a = [self.individual_compatibility(observations[d[0]], landmarks[d[1]]) for d in data_association]
        return data_association

    def update_map(self, new_observations:List[Obstacle], state:VehicleState, map:Map) -> Map:

        observations = self.calculate_obstacles_in_global_frame(state, new_observations)
        last_batch = self.update_batch_and_wait(observations)

        for observation in last_batch:
            final_batch = map.map.copy()

            for i,landmark in enumerate(map.map):
                ic = self.individual_compatibility(observation, landmark)
                if ic >= self.gamma:
                    final_batch.remove(landmark)

            if bool(final_batch):
                observation.id = final_batch[self.maximum_likelihood(observation, final_batch)].id
                self.extended_kalman_filter(observation, map.map[observation.id])
            else:
                map.add_new_obstacle(observation)
                
        return map