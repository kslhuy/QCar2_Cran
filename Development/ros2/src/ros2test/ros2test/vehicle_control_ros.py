import time
from hal.products.mats import SDCSRoadMap
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from qcar2_interfaces.msg import MotorCommands
import numpy as np
from geometry_msgs.msg import PoseStamped
from hal.utilities.control import StanleyController
from ros2test.controller import SpeedController
from ros2test.util import quaternion_to_yaw
import pyqtgraph as pg
from pyqtgraph.Qt import QtWidgets
from collections import deque
from sensor_msgs.msg import LaserScan
from pynput import keyboard
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

class VehicleControl(Node):
    def __init__(self):
        super().__init__('vehicle_control')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('v_ref', 0.9),
                ('K_p', 0.1),
                ('K_i', 1),
                ('K_stanley', 1),
                ('NODE_SEQUENCE', [10, 4, 20, 10]),
                ('ENCODER_COUNTS_PER_REV', 720.0),
                ('WHEEL_RADIUS', 0.033),
                ('PIN_TO_SPUR_RATIO', 0.09536679536679536),
                ('VIRTUAL', True),
            ]
        )        
        
        # Get parameters
        self.v_ref = self.get_parameter("v_ref").value
        self.K_p = self.get_parameter("K_p").value
        self.K_i = self.get_parameter("K_i").value
        self.K_stanley = self.get_parameter("K_stanley").value
        self.nodeSequence = self.get_parameter("NODE_SEQUENCE").value
        self.ENCODER_COUNTS_PER_REV = self.get_parameter("ENCODER_COUNTS_PER_REV").value
        self.WHEEL_RADIUS = self.get_parameter("WHEEL_RADIUS").value
        self.PIN_TO_SPUR_RATIO = self.get_parameter("PIN_TO_SPUR_RATIO").value
        self.VIRTUAL = self.get_parameter("VIRTUAL").value
        
        self.CPS_TO_MPS = (1/(self.ENCODER_COUNTS_PER_REV*4) # motor-speed unit conversion
            * self.PIN_TO_SPUR_RATIO * 2*np.pi * self.WHEEL_RADIUS)
        
        roadmap = SDCSRoadMap(leftHandTraffic=False)
        self.waypointSequence = roadmap.generate_path(self.nodeSequence)
        # self.waypointSequence = self.waypointSequence[:, :self.waypointSequence.shape[1] // 2]
        
        self.utils = Utils()

        vehicle_control_plot = pg.plot(title="vehicle_control")

        vehicle_control_plot.setXRange(-4, 4)

        vehicle_control_plot.setYRange(-2, 6)
        
        self.plotwaypoints = vehicle_control_plot.plot([], [], pen=None, symbol='o', symbolBrush='r', symbolPen=None, symbolSize=2)
        # self.plotpos = vehicle_control_plot.plot([], [], pen=None, symbol='o', symbolBrush='g', symbolPen=None, symbolSize=2)
        
        self.poslistx = deque(maxlen=300)
        self.poslisty = deque(maxlen=300)
        
        self.ekf_sub = self.create_subscription(PoseStamped, '/ekf_pose', self.ekf_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/qcar2_joint', self.joint_callback, 10)

        self.motor_pub = self.create_publisher(MotorCommands, "/qcar2_motor_speed_cmd", 10)
        
        self.speedController = SpeedController(
            kp=self.K_p,
            ki=self.K_i
        )
        self.stanleyController = StanleyController(
            waypoints=self.waypointSequence,
            k=self.K_stanley,
            cyclic=False
        )
        
        self.t0 = time.time()
        self.t = 0
        self.motorTach = 0
        self.delta = 0 

        # Occupancy grid with LIDAR
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan', 
            self.scan_callback,
            10
        )

        squareSize = 10

        vehicle_control_plot.setXRange(-squareSize, squareSize)

        vehicle_control_plot.setYRange(-squareSize, squareSize)

        self.lidarData = vehicle_control_plot.plot([], [], pen=None, symbol='o', symbolBrush='b', symbolPen=None, symbolSize=2)
        self.detected = vehicle_control_plot.plot([], [], pen=None, symbol='o', symbolBrush='g', symbolPen=None, symbolSize=2)
        self.target = vehicle_control_plot.plot([], [], pen=None, symbol='o', symbolBrush='b', symbolPen=None, symbolSize=10)
        self.target2 = vehicle_control_plot.plot([], [], pen=None, symbol='o', symbolBrush='y', symbolPen=None, symbolSize=10)
        # Occupancy grid (added as image item)
        self.occupancy_img = pg.ImageItem()
        self.occupancy_img.setZValue(-10)  # Draw under LiDAR points
        vehicle_control_plot.addItem(self.occupancy_img)

        self.occupancy_pub = self.create_publisher(OccupancyGrid, "/occupancy_grid", 10)
                
        self.L = 3
        self.CELLS_PER_METER = 20
        self.grid_width_meters = 6
        self.grid_height = int(self.L * self.CELLS_PER_METER)
        self.grid_width = int(self.grid_width_meters * self.CELLS_PER_METER)
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1
       
        self.IS_OCCUPIED = 100
        self.IS_FREE = 50

        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        # Subscribe to the occupancy grid topic
        self.occupancy_sub = self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',
            self.occupancy_callback,
            10
        )

    key_states = {
        'm': False,
    }

    def on_press(self, key):
        """Handles key press events."""
        try:
            if key.char in self.key_states:
                self.key_states[key.char] =  not self.key_states[key.char]
        except AttributeError:
            pass  # Ignore special keys

    def on_release(self, key):
        """Handles key release events."""
        pass
        # try:
        #     if key.char in self.key_states:
        #         self.key_states[key.char] = False
        # except AttributeError:
        #     pass  # Ignore special keys

    def joint_callback(self, msg: JointState): 
        # self.get_logger().info("joint " + str(msg.velocity[0] * self.CPS_TO_MPS))
        self.motorTach = msg.velocity[0] * self.CPS_TO_MPS
        
    def ekf_callback(self, msg: PoseStamped): 
        if self.key_states['m']:
            self.send_control(0, 0)
            return
        xs = self.waypointSequence[0]
        ys = self.waypointSequence[1] 
        pose = msg.pose
        tp = self.t
        self.t = time.time() - self.t0
        dt = self.t-tp

        x = pose.position.x
        y = pose.position.y
        self.poslistx.append(x)
        self.poslisty.append(y)
        # self.plotpos.setData(self.poslistx, self.poslisty)

        ox = pose.orientation.x
        oy = pose.orientation.y
        oz = pose.orientation.z
        ow = pose.orientation.w
        
        th = quaternion_to_yaw(ox, oy, oz, ow)
        th2 = (np.pi/2) - th
        xs = xs - x
        ys = ys - y
        x2 = xs * np.cos(th2) - ys * np.sin(th2)
        y2 = xs * np.sin(th2) + ys * np.cos(th2)
        self.plotwaypoints.setData(x2, y2)
        
        QtWidgets.QApplication.instance().processEvents()
        
        p = ( np.array([x, y])
            + np.array([np.cos(th), np.sin(th)]) * 0.2)
        v = self.motorTach

        u = self.speedController.update(v, self.v_ref, dt)
        self.delta = self.stanleyController.update(p, th, v)

        # Stop if path is complete
        # There's a problem in StanleyController update algorithm where it doesn't stop at the end of the path (pathComplete never True)
        # Check manually here
        if self.stanleyController.wpi >= self.stanleyController.N - 2:
            self.get_logger().info("path completed")
            self.send_control(0, 0)
            return

        lookahead_x = x2[(self.stanleyController.wpi + 30)%self.stanleyController.N]
        lookahead_y = y2[(self.stanleyController.wpi + 30)%self.stanleyController.N]
        
        # dx = np.cos(self.delta)*0.7
        # dy = np.sin(self.delta)*0.7
        dx = lookahead_x
        dy = lookahead_y

        if self.avoid_obstacle_occupancy_grid(u, dx, dy):
            return  # Skip sending control if obstacle detected
        
        # self.get_logger().info(f"delta = {self.delta}")

        self.send_control(u, self.delta)
        # self.send_control(0, self.delta)


    def local_to_grid(self, x, y):
        i = int(y * -self.CELLS_PER_METER + (self.grid_height - 1))
        j = int(x * self.CELLS_PER_METER + self.CELL_Y_OFFSET)
        return (i, j)

    def local_to_grid_parallel(self, x, y):
        i = np.round(y * -self.CELLS_PER_METER + (self.grid_height - 1)).astype(int)
        j = np.round(x * self.CELLS_PER_METER + self.CELL_Y_OFFSET).astype(int)
        return i, j

    def grid_to_local(self, point):
        i, j = point[0], point[1]
        y = (i - (self.grid_height - 1)) / -self.CELLS_PER_METER
        x = (j - self.CELL_Y_OFFSET) / self.CELLS_PER_METER
        return (x, y)
    
    def check_collision(self, cell_a, cell_b, margin=0):
        """
        Checks whether the path between two cells
        in the occupancy grid is collision free.

        The margin is done by checking if adjacent cells are also free.

        One of the issues is that if the starting cell is next to a wall, then it already considers there to be a collision.
        See check_collision_loose


        Args:
            cell_a (i, j): index of cell a in occupancy grid
            cell_b (i, j): index of cell b in occupancy grid
            margin (int): margin of safety around the path
        Returns:
            collision (bool): whether path between two cells would cause collision
        """
        for i in range(-margin, margin + 1):  # for the margin, check
            cell_a_margin = (cell_a[0], cell_a[1] + i)
            cell_b_margin = (cell_b[0], cell_b[1] + i)
            for cell in self.utils.traverse_grid(cell_a_margin, cell_b_margin):
                if (cell[0] * cell[1] < 0) or (cell[0] >= self.grid_height) or (cell[1] >= self.grid_width):
                    continue
                try:
                    if self.occupancy_grid[cell] == self.IS_OCCUPIED:
                        return True
                except:
                    #print(f"Sampled point is out of bounds: {cell}")
                    return True
        return False

    def check_collision_loose(self, cell_a, cell_b, margin=0):
        """
        Checks whether the path between two cells
        in the occupancy grid is collision free.

        The margin is done by checking if adjacent cells are also free.

        This looser implementation only checks half way for meeting the margin requirement.


        Args:
            cell_a (i, j): index of cell a in occupancy grid
            cell_b (i, j): index of cell b in occupancy grid
            margin (int): margin of safety around the path
        Returns:
            collision (bool): whether path between two cells would cause collision
        """
        for i in range(-margin, margin + 1):  # for the margin, check
            cell_a_margin = (int((cell_a[0] + cell_b[0]) / 2), int((cell_a[1] + cell_b[1]) / 2) + i)
            cell_b_margin = (cell_b[0], cell_b[1] + i)
            for cell in self.utils.traverse_grid(cell_a_margin, cell_b_margin):
                if (cell[0] * cell[1] < 0) or (cell[0] >= self.grid_height) or (cell[1] >= self.grid_width):
                    continue
                try:
                    if self.occupancy_grid[cell] == self.IS_OCCUPIED:
                        return True
                except:
                    #print(f"Sampled point is out of bounds: {cell}")
                    return True
        return False
       
    def avoid_obstacle_occupancy_grid(self, u, dx, dy):
        
        current_pos = np.array(self.local_to_grid(0, 0))
        
        # dx_rot = -dy
        # dy_rot = dx
        dx_rot = dx
        dy_rot = dy
        self.target.setData([dx_rot], [dy_rot])
        self.target2.setData([], [])

        goal_pos = np.array(self.local_to_grid(dx_rot, dy_rot))
        target = None
        MARGIN = 4

        if self.check_collision(current_pos, goal_pos, margin=MARGIN):
            self.obstacle_detected = True

            shifts = [i * (-1 if i % 2 else 1) for i in range(1, 21)]

            found = False
            for shift in shifts:
                # We consider various points to the left and right of the goal position
                new_goal = goal_pos + np.array([0, shift])

                # If we are currently super close to the wall, this logic doesn't work
                if not self.check_collision(current_pos, new_goal, margin=int(1.5 * MARGIN)):
                    target = self.grid_to_local(new_goal)
                    found = True
                    print("Found condition 1")
                    break

            if not found:
                # This means that the obstacle is very close to us, we need even steeper turns
                middle_grid_point = np.array(current_pos + (goal_pos - current_pos) / 2).astype(int)

                for shift in shifts:
                    new_goal = middle_grid_point + np.array([0, shift])
                    if not self.check_collision(current_pos, new_goal, margin=int(1.5 * MARGIN)):
                        target = self.grid_to_local(new_goal)
                        found = True
                        print("Found condition 2")
                        break

            if not found:
                # Try again with a looser collision checker, we are probably very close to the obstacle, so check only collision free in the second half
                middle_grid_point = np.array(current_pos + (goal_pos - current_pos) / 2).astype(int)

                for shift in shifts:
                    new_goal = middle_grid_point + np.array([0, shift])
                    if not self.check_collision_loose(current_pos, new_goal, margin=MARGIN):
                        target = self.grid_to_local(new_goal)
                        found = True
                        print("Found condition 3")
                        break

        else:
            self.obstacle_detected = False
            target = self.grid_to_local(goal_pos)
            
        if target:
            if self.obstacle_detected:
                target_x_rot = target[1]
                target_y_rot = -target[0]

                K_p = 0.8

                L = np.linalg.norm(target)
                y = target_y_rot
                angle = K_p * (2 * y) / (L**2)
                angle = np.clip(angle, -np.radians(30), np.radians(30))

                self.send_control(0.6*u, angle)
                self.target2.setData([target[0]], [target[1]])
                return True
            else:
                return False
        else:
            self.send_control(0, 0)
            return True

    def send_control(self, motor, steer):
        msg = MotorCommands()
        msg.motor_names.append("steering_angle")
        msg.motor_names.append("motor_throttle")
        msg.values.append(steer)
        msg.values.append(motor)
        self.motor_pub.publish(msg)

    def local_to_grid_parallel(self, x, y):
        i = np.round(y * -self.CELLS_PER_METER + (self.grid_height - 1)).astype(int)
        j = np.round(x * self.CELLS_PER_METER + self.CELL_Y_OFFSET).astype(int)
        return i, j
        
    # def publish_occupancy_grid(self):
    #     """
    #     Publish the occupancy grid as a ROS2 OccupancyGrid message
    #     """
    #     msg = OccupancyGrid()
        
    #     # Header
    #     msg.header = Header()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.header.frame_id = "base_link"
        
    #     # Map metadata
    #     msg.info.resolution = 1.0 / self.CELLS_PER_METER  # meters per cell
    #     msg.info.width = self.grid_width
    #     msg.info.height = self.grid_height
        
    #     # Origin (bottom-left corner in real-world coordinates)
    #     msg.info.origin.position.x = -self.grid_width_meters / 2.0
    #     msg.info.origin.position.y = 0.0
    #     msg.info.origin.position.z = 0.0
    #     msg.info.origin.orientation.w = 1.0
        
    #     # Convert occupancy grid to ROS format (0-100, -1 for unknown)
    #     # Your format: IS_FREE=50, IS_OCCUPIED=100
    #     # ROS format: 0=free, 100=occupied, -1=unknown
    #     ros_grid = np.copy(self.occupancy_grid).astype(np.int8)
    #     ros_grid[ros_grid == self.IS_FREE] = 0      # Free cells
    #     ros_grid[ros_grid == self.IS_OCCUPIED] = 100  # Occupied cells
        
    #     # Flatten and convert to list (ROS expects row-major order)
    #     msg.data = ros_grid.flatten().tolist()
        
    #     self.occupancy_pub.publish(msg)

    def scan_callback(self, msg: LaserScan):

        ranges = np.array(list(msg.ranges))[::-1] 
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        if not self.VIRTUAL :
            angles = (angles + np.pi) % (2 * np.pi)

        x = np.sin(angles)*ranges

        y = np.cos(angles)*ranges

        self.lidarData.setData(x,y)
        QtWidgets.QApplication.instance().processEvents()

    def occupancy_callback(self, msg: OccupancyGrid):
        """
        Callback to process the received occupancy grid.
        """
        # Process the occupancy grid data as needed
        self.occupancy_grid = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.update_occupancy_image()

    def update_occupancy_image(self):
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
                -shape[1] / 2.0 * (1 / self.CELLS_PER_METER),
                0,
                shape[1] * (1 / self.CELLS_PER_METER),
                shape[0] * (1 / self.CELLS_PER_METER)
            )
        )
        QtWidgets.QApplication.instance().processEvents()

def main(args=None):
    rclpy.init(args=args)
    node = VehicleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



class Utils:
    def __init__(self):
        pass

    def traverse_grid(self, start, end):
        """
        Bresenham's line algorithm for fast voxel traversal

        CREDIT TO: Rogue Basin
        CODE TAKEN FROM: http://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm
        """
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        return points
    
    def polar_to_cartesian(r, angle_deg, origin=(0, 0)):
        """
        Convert polar coordinates (r, angle) to Cartesian (x, y).
        Angle is in degrees. Supports scalars or NumPy arrays.
        """
        x0 = 0
        y0 = 0
        angle_rad = np.deg2rad(angle_deg)
        x = x0 + r * np.cos(angle_rad)
        y = y0 + r * np.sin(angle_rad)
        return np.array([x, y])

    def cartesian_to_polar(x, y, origin=(0, 0)):
        """
        Convert Cartesian coordinates (x, y) to polar (r, angle).
        Returns (r, angle_rad, angle_deg).
        Supports scalars or NumPy arrays.
        """
        x0, y0 = origin
        dx = x - x0
        dy = y - y0
        r = np.hypot(dx, dy)
        angle_rad = np.arctan2(dy, dx)
        angle_deg = np.rad2deg(angle_rad)
        return np.array([r, angle_rad, angle_deg])