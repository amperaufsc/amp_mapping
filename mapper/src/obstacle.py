class Obstacle:

    def __init__(self, x, y, confidence, label,deviation, count=1, id=0,time = 1e-15):
        self.x = x
        self.y = y
        self.confidence = 0.7
        self.label = label
        self.count = count
        self.deviation = 0.3
        self.id = id
        self.last_observed_timestamp = time

    def get_obstacle_as_array(self):
        """
        Get obstacle data
        :return: x, y, confidence, label
        """
        return [self.x, self.y, self.confidence, self.label]



    def get_obstacle_with_extra_parameters_as_array(self):
        """
        Get obstacle data
        :return: x, y, confidence, label
        """
        return [self.x, self.y, self.confidence, self.label, self.count, self.deviation]
    
    def update(self, x, y, deviation):
        self.x = x
        self.y = y
        self.deviation = deviation