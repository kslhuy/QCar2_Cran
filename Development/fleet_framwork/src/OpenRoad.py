import numpy as np
from hal.utilities.path_planning import RoadMap

class OpenRoad(RoadMap):
    """A RoadMap implementation for OpenRoad"""

    def __init__(self):
        """Initialize a new OpenRoad instance.
        """
        super().__init__()
        # useful constants
        self.scale = 1
        self.xOffset = 0
        self.yOffset = 0

        pi = np.pi


        nodePoses = [
            [0, -6, 3.14],       # Node 0
            [-100,-6, 3.14]
        ]
        edgeConfigs = [
                [0, 1, 0]
        ]
        

        self.scale_then_add_nodes(nodePoses)
        for edgeConfig in edgeConfigs:
            self.add_edge(*edgeConfig)

    

    def scale_then_add_nodes(self,nodePoses):
        for pose in nodePoses:
            pose[0] = self.scale * (pose[0] - self.xOffset)
            pose[1] = self.scale * (self.yOffset - pose[1])
            self.add_node(pose)