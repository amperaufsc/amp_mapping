class State:
    def __init__(self, x_position, y_position, yaw, x_velocity, y_velocity, z_angular_velocity, timestamp,position_deviation = 1e-15,velocity_deviation =1e-15,yaw_deviation = 1e-15):

        self.x_position = x_position
        self.y_position = y_position
        self.pos_deviation = position_deviation
        self.yaw = yaw
        self.x_velocity = x_velocity
        self.y_velocity = y_velocity
        self.vel_deviation = velocity_deviation
        self.z_angular_velocity = z_angular_velocity
        self.position_deviation = position_deviation
        self.yaw_deviation = yaw_deviation
        self.timestamp = timestamp

    def get_state_as_array(self):
        return [self.x_position, self.y_position, self.yaw, self.x_velocity, self.y_velocity, self.z_angular_velocity, self.timestamp, self.pos_deviation, self.vel_deviation]