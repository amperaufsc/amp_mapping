class VehicleState:
    def __init__(self, x_position, y_position, yaw, x_velocity, y_velocity, z_angular_velocity):
        self.x_position = x_position
        self.y_position = y_position
        self.yaw = yaw
        self.x_velocity = x_velocity
        self.y_velocity = y_velocity
        self.z_angular_velocity = z_angular_velocity

    def get_state_as_array(self):
        return [self.x_position, self.y_position, self.yaw, self.x_velocity, self.y_velocity, self.z_angular_velocity]