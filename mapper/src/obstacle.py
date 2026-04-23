import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

pkg_share = Path(get_package_share_directory("mapper"))
with open(pkg_share / 'config'/ 'mapper_parameters.yaml') as f:
    data = yaml.safe_load(f)
parameters = data["AMP"]["mapper_node"]["ros__parameters"]

class Obstacle:

    def __init__(self, x, y, confidence = parameters["confidence"], label = 0, deviation = parameters["deviation"], count=1, id=0,time = 1e-15):
        self.x = x
        self.y = y
        self.confidence = confidence
        self.label = label
        self.count = count
        self.deviation = deviation
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