import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui
import sys
import numpy as np
from pyqtgraph.Qt import QtWidgets


class LidarSub(Node):
    def __init__(self):
        super().__init__('lidar_sub')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.scan_callback,
            10
        )

        lidarPlot = pg.plot(title="lidar")

        squareSize = 10

        lidarPlot.setXRange(-squareSize, squareSize)

        lidarPlot.setYRange(-squareSize, squareSize)

        self.lidarData = lidarPlot.plot([], [], pen=None, symbol='o', symbolBrush='r', symbolPen=None, symbolSize=2)
        self.detected = lidarPlot.plot([], [], pen=None, symbol='o', symbolBrush='g', symbolPen=None, symbolSize=2)
        self.target = lidarPlot.plot([], [], pen=None, symbol='o', symbolBrush='b', symbolPen=None, symbolSize=10)
        self.target2 = lidarPlot.plot([], [], pen=None, symbol='o', symbolBrush='y', symbolPen=None, symbolSize=10)
        # Occupancy grid (added as image item)
        self.occupancy_img = pg.ImageItem()
        self.occupancy_img.setZValue(-10)  # Draw under LiDAR points
        lidarPlot.addItem(self.occupancy_img)

                
        self.L = 3
        self.CELLS_PER_METER = 20
        self.grid_width_meters = 6
        self.grid_height = int(self.L * self.CELLS_PER_METER)
        self.grid_width = int(self.grid_width_meters * self.CELLS_PER_METER)
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1
       
        self.IS_OCCUPIED = 100
        self.IS_FREE = 50

    def local_to_grid_parallel(self, x, y):
        i = np.round(y * -self.CELLS_PER_METER + (self.grid_height - 1)).astype(int)
        j = np.round(x * self.CELLS_PER_METER + self.CELL_Y_OFFSET).astype(int)
        return i, j

    def populate_occupancy_grid(self, ranges, thetas):
        """
        Populate occupancy grid using lidar scans and save
        the data in class member variable self.occupancy_grid.

        Optimization performed to improve the speed at which we generate the occupancy grid.

        Args:
            scan_msg (LaserScan): message from lidar scan topic
        """
        # reset empty occupacny grid (-1 = unknown)

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
        
        
    # === Occupancy Grid Plot Update Function ===
    def update_occupancy_image(self, cell_size=1/20):
        """
        Update the PyQtGraph image layer with new occupancy grid data.
        """
        # Flip vertically so image matches LiDAR frame orientation
        image = np.flipud(self.occupancy_grid).T  # transpose for (x,y) alignment
        self.occupancy_img.setImage(image, levels=(0, 100))

        # Scale image so each cell matches real-world meters
        shape = self.occupancy_grid.shape
        self.occupancy_img.setRect(
            pg.QtCore.QRectF(
                -shape[1] / 2.0 * cell_size,
                0,
                shape[1] * cell_size,
                shape[0] * cell_size
            )
        )    

    def scan_callback(self, msg: LaserScan):

        ranges = np.array(list(msg.ranges))[::-1] 
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        angles = (angles + np.pi) % (2 * np.pi)
        self.populate_occupancy_grid(ranges, angles)
        self.update_occupancy_image()

        x = np.sin(angles)*ranges

        y = np.cos(angles)*ranges

        self.lidarData.setData(x,y)
        QtWidgets.QApplication.instance().processEvents()



def main(args=None):
    rclpy.init(args=args)
    node = LidarSub()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
