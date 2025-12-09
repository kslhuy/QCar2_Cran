import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np

class LidarOccupancyNode(Node):
    def __init__(self):
        super().__init__('lidar_occupancy_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('CELLS_PER_METER', 20),
                ('GRID_WIDTH_METERS', 6),
                ('GRID_HEIGHT_METERS', 3),
                ('IS_OCCUPIED', 100),
                ('IS_FREE', 50),
                ('VIRTUAL', True),
            ]
        )

        # Get parameters
        self.CELLS_PER_METER = self.get_parameter("CELLS_PER_METER").value
        self.GRID_WIDTH_METERS = self.get_parameter("GRID_WIDTH_METERS").value
        self.GRID_HEIGHT_METERS = self.get_parameter("GRID_HEIGHT_METERS").value
        self.IS_OCCUPIED = self.get_parameter("IS_OCCUPIED").value
        self.IS_FREE = self.get_parameter("IS_FREE").value
        self.VIRTUAL = self.get_parameter("VIRTUAL").value

        self.grid_width = int(self.GRID_WIDTH_METERS * self.CELLS_PER_METER)
        self.grid_height = int(self.GRID_HEIGHT_METERS * self.CELLS_PER_METER)
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.occupancy_pub = self.create_publisher(OccupancyGrid, "/occupancy_grid", 10)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(list(msg.ranges))[::-1]
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        if not self.VIRTUAL:
            angles = (angles + np.pi) % (2 * np.pi)

        self.populate_occupancy_grid(ranges, angles)
        self.publish_occupancy_grid()

    def populate_occupancy_grid(self, ranges, thetas):
        """
        Populate occupancy grid using lidar scans and save
        the data in class member variable self.occupancy_grid.
        """
        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=self.IS_FREE, dtype=int)

        ranges = np.array(ranges)
        valid = ranges > 0
        ranges = ranges[valid]
        thetas = thetas[valid]
        xs = ranges * np.sin(thetas)
        ys = ranges * np.cos(thetas)

        i, j = self.local_to_grid_parallel(xs, ys)

        occupied_indices = np.where((i > 0) & (i < self.grid_height) & (j > 0) & (j < self.grid_width))
        self.occupancy_grid[i[occupied_indices], j[occupied_indices]] = self.IS_OCCUPIED

    def local_to_grid_parallel(self, x, y):
        i = np.round(y * -self.CELLS_PER_METER + (self.grid_height - 1)).astype(int)
        j = np.round(x * self.CELLS_PER_METER + self.CELL_Y_OFFSET).astype(int)
        return i, j

    def publish_occupancy_grid(self):
        """
        Publish the occupancy grid as a ROS2 OccupancyGrid message.
        """
        msg = OccupancyGrid()

        # Header
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        # Map metadata
        msg.info.resolution = 1.0 / self.CELLS_PER_METER  # meters per cell
        msg.info.width = self.grid_width
        msg.info.height = self.grid_height

        # Origin (bottom-left corner in real-world coordinates)
        msg.info.origin.position.x = -self.GRID_WIDTH_METERS / 2.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # Convert occupancy grid to ROS format (0-100, -1 for unknown)
        ros_grid = np.copy(self.occupancy_grid).astype(np.int8)
        ros_grid[ros_grid == self.IS_FREE] = 0      # Free cells
        ros_grid[ros_grid == self.IS_OCCUPIED] = 100  # Occupied cells

        # Flatten and convert to list (ROS expects row-major order)
        msg.data = ros_grid.flatten().tolist()

        self.occupancy_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarOccupancyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
