from pal.utilities.math import wrap_to_pi
from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.scope import MultiScope
import pal.resources.images as images

from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar2 import QLabsQCar2
from qvl.real_time import QLabsRealTime
#Environment
from qvl.basic_shape import QLabsBasicShape
from qvl.walls import QLabsWalls
from qvl.qcar_flooring import QLabsQCarFlooring
from qvl.crosswalk import QLabsCrosswalk

from hal.content.qcar_functions import QCarEKF

from src.Controller.DummyController import DummyVehicle
from src.Controller.idm_control import IDMControl
from src.Controller.CACC import CACC
from src.OpenRoad import OpenRoad
from VehicleProcess import vehicle_process_main

import time, io, math, threading, os, multiprocessing
import numpy as np
import pandas as pd


class PortManager:
    """Manages port allocation for vehicle communication"""
    
    def __init__(self, max_vehicles=5):
        self.max_vehicles = max_vehicles
        # Port configuration for up to 5 vehicles (bidirectional design)
        # Format: [send_port, recv_port, ack_port] for each vehicle
        self.port_config = {
            0: [6000, 6000, 6002],  # Vehicle 0: send=6000, recv=6000, ack=6002
            1: [6010, 6010, 6012],  # Vehicle 1: send=6010, recv=6010, ack=6012
            2: [6020, 6020, 6022],  # Vehicle 2: send=6020, recv=6020, ack=6022  
            3: [6030, 6030, 6032],  # Vehicle 3: send=6030, recv=6030, ack=6032
            4: [6040, 6040, 6042],  # Vehicle 4: send=6040, recv=6040, ack=6042
        }
    
    def get_ports_for_vehicle(self, vehicle_id):
        """Get port configuration for a specific vehicle"""
        if vehicle_id not in self.port_config:
            raise ValueError(f"No port configuration for vehicle {vehicle_id}")
        return self.port_config[vehicle_id]
    
    def create_peer_port_mapping(self, vehicle_id, peer_id):
        """Create port mapping for communication between two vehicles"""
        peer_send_port, peer_recv_port, peer_ack_port = self.get_ports_for_vehicle(peer_id)
        return {
            'send_to_peer': peer_recv_port,  # Send to peer's receive port
            'recv_from_peer': peer_send_port, # Receive from peer's send port
            'ack_from_peer': peer_ack_port    # Acknowledge to peer's ack port
        }


class GraphManager:
    """Manages fleet communication graph topology"""
    
    def __init__(self, num_vehicles, leader_index):
        self.num_vehicles = num_vehicles
        self.leader_index = leader_index
        self.graph = None
    
    def create_graph(self, graph_type="fully_connected"):
        """
        Create adjacency matrix representing the fleet vehicle connections.
        
        graph_type: str - Type of graph topology
                   "fully_connected" - All vehicles connected to all others
                   "chain" - Chain topology (each vehicle connected to neighbors)
                   "star" - Star topology (leader connected to all, others only to leader)
                   "custom" - Custom adjacency matrix (to be defined)
        
        Returns: numpy array - Adjacency matrix where graph[i][j] = 1 means vehicle i can communicate with vehicle j
        """
        self.graph_type = graph_type  # Store the graph type
        n = self.num_vehicles
        
        if graph_type == "fully_connected":
            # Fully connected graph - all vehicles connected to all others
            graph = np.ones((n, n), dtype=int)
            # Set diagonal to 0 (vehicle doesn't connect to itself)
            np.fill_diagonal(graph, 0)
            
        elif graph_type == "chain":
            # Chain topology - each vehicle connected to its immediate neighbors
            graph = np.zeros((n, n), dtype=int)
            for i in range(n):
                if i > 0:  # Connect to previous vehicle
                    graph[i][i-1] = 1
                if i < n-1:  # Connect to next vehicle
                    graph[i][i+1] = 1
                    
        elif graph_type == "star":
            # Star topology - leader connected to all, others only to leader
            graph = np.zeros((n, n), dtype=int)
            leader_idx = self.leader_index
            for i in range(n):
                if i != leader_idx:
                    graph[leader_idx][i] = 1  # Leader to follower
                    graph[i][leader_idx] = 1  # Follower to leader
                    
        elif graph_type == "custom":
            # Custom graph - can be modified as needed
            # Example: predefined adjacency matrix
            if n == 3:
                graph = np.array([
                    [0, 1, 1],  # Vehicle 0 connects to 1, 2
                    [1, 0, 1],  # Vehicle 1 connects to 0, 2
                    [1, 1, 0]   # Vehicle 2 connects to 0, 1
                ])
            elif n == 4:
                graph = np.array([
                    [0, 1, 1, 1],  # Vehicle 0 connects to 1, 2, 3
                    [1, 0, 1, 1],  # Vehicle 1 connects to 0, 2, 3
                    [1, 1, 0, 1],  # Vehicle 2 connects to 0, 1, 3
                    [1, 1, 1, 0]   # Vehicle 3 connects to 0, 1, 2
                ])
            else:
                # Default to fully connected for other sizes
                graph = np.ones((n, n), dtype=int)
                np.fill_diagonal(graph, 0)
        else:
            raise ValueError(f"Unknown graph type: {graph_type}")
            
        self.graph = graph
        return graph
    
    def print_graph(self):
        """Print the fleet adjacency matrix in a readable format"""
        if self.graph is None:
            print("No graph created yet")
            return
            
        print(f"\nFleet Adjacency Matrix ({self.num_vehicles} vehicles):")
        print("=" * 50)
        print("   ", end="")
        for j in range(self.num_vehicles):
            print(f"V{j:2d}", end=" ")
        print()
        
        for i in range(self.num_vehicles):
            role = "L" if i == self.leader_index else "F"
            print(f"V{i:2d}({role})", end=" ")
            for j in range(self.num_vehicles):
                print(f"{self.graph[i][j]:2d} ", end=" ")
            print()
        print("=" * 50)
        print("L = Leader, F = Follower")
        print("1 = Connected, 0 = Not Connected")
        print()
    
    def get_connected_vehicles(self, vehicle_id):
        """Get list of vehicles connected to the specified vehicle"""
        if self.graph is None:
            return []
        return [j for j in range(self.num_vehicles) if self.graph[vehicle_id][j] == 1]
    
    def get_connectivity_stats(self):
        """Get connectivity statistics for the graph"""
        if self.graph is None:
            return {}
        
        total_connections = np.sum(self.graph) // 2  # Divide by 2 since graph is symmetric
        max_connections = self.num_vehicles * (self.num_vehicles - 1) // 2
        connectivity_ratio = total_connections / max_connections if max_connections > 0 else 0
        
        return {
            'total_connections': total_connections,
            'max_connections': max_connections,
            'connectivity_ratio': connectivity_ratio
        }


class CommunicationGraph:
    """Manages peer-to-peer communication setup based on graph topology"""
    
    def __init__(self, port_manager, graph_manager):
        self.port_manager = port_manager
        self.graph_manager = graph_manager
        self.peer_ports = {}
    
    def setup_peer_ports(self):
        """Setup peer port mappings based on graph connectivity"""
        self.peer_ports = {}
        for vehicle_id in range(self.graph_manager.num_vehicles):
            self.peer_ports[vehicle_id] = {}
            for peer_id in range(self.graph_manager.num_vehicles):
                # Only add peer if there's a connection in the graph
                if peer_id != vehicle_id and self.graph_manager.graph[vehicle_id][peer_id] == 1:
                    self.peer_ports[vehicle_id][peer_id] = self.port_manager.create_peer_port_mapping(vehicle_id, peer_id)
        
        return self.peer_ports


class VehicleConfigBuilder:
    """Builds configuration dictionaries for vehicle processes"""
    
    def __init__(self, port_manager, graph_manager, communication_graph, config):
        self.port_manager = port_manager
        self.graph_manager = graph_manager
        self.communication_graph = communication_graph
        self.config = config
        self.vehicle_configs = []
    
    def build_vehicle_configs(self, num_vehicles, leader_index, controller, observer, 
                            init_position_table, qcar_scale, distance):
        """Build configuration for all vehicles"""
        self.vehicle_configs = []
        
        for i in range(num_vehicles):
            config = self._build_single_vehicle_config(i, num_vehicles, leader_index, 
                                                     controller, observer, init_position_table, 
                                                     qcar_scale, distance)
            self.vehicle_configs.append(config)
        
        return self.vehicle_configs
    
    def _build_single_vehicle_config(self, vehicle_id, num_vehicles, leader_index, 
                                   controller, observer, init_position_table, 
                                   qcar_scale, distance):
        """Build configuration for a single vehicle"""
        is_leader = (vehicle_id == leader_index)
        send_port, recv_port, ack_port = self.port_manager.get_ports_for_vehicle(vehicle_id)
        
        # Chain-following configuration
        following_target = None if is_leader else vehicle_id - 1
        
        # Extract initial pose
        vehicle_initial_pose, spawn_location, spawn_rotation = self._get_initial_pose(vehicle_id, init_position_table)
        
        # Build the configuration dictionary
        vehicle_config = {
            'vehicle_id': vehicle_id,
            'controller_type': controller,
            'is_leader': is_leader,
            'fleet_size': num_vehicles,
            'initial_pose': vehicle_initial_pose,
            
            # Chain-following configuration
            'following_target': following_target,
            
            # Fleet graph information
            'fleet_graph': {
                'type': self.graph_manager.graph_type if hasattr(self.graph_manager, 'graph_type') else 'fully_connected',
                'matrix': self.graph_manager.graph.tolist()
            },
            'connected_vehicles': self.graph_manager.get_connected_vehicles(vehicle_id),
            'num_connections': len(self.graph_manager.get_connected_vehicles(vehicle_id)),
            
            # Bidirectional communication settings
            'target_ip': "127.0.0.1",
            'send_port': send_port,
            'recv_port': recv_port,
            'ack_port': ack_port,
            'peer_ports': self.communication_graph.peer_ports[vehicle_id],
            'communication_mode': 'bidirectional',
            
            # GPS settings
            'gps_server_ip': "127.0.0.1",
            'gps_server_port': 8001,
            
            # Control parameters
            'max_steering': getattr(self.config, 'max_steering', 0.6),
            'lookahead_distance': getattr(self.config, 'lookahead_distance', 7.0),
            'general_update_rate': getattr(self.config, 'update_rate', 100),
            'controller_rate': getattr(self.config, 'control_rate', 50),
            'observer_rate': getattr(self.config, 'observer_rate', 100),
            'gps_update_rate': getattr(self.config, 'gps_update_rate', 5),

            # Observer configuration
            'observer': {
                'local_observer_type': 'kalman',
                'enable_distributed': True,
                "distributed_observer_type": "consensus",
                'enable_noise_measurement': False,
                'enable_prediction': True,
                'consensus_gain': 0.1,
            },
            
            # Vehicle spawn information
            'vehicle_scale': qcar_scale,
            'spawn_location': spawn_location,
            'spawn_rotation': spawn_rotation,
            
            # Fleet information
            'leader_index': leader_index,
            'distance_between_cars': distance,

            # Trust and Safety
            'enable_trust_evaluation': True,

            # Additional config parameters
            'simulation_time': getattr(self.config, 'simulation_time', 0),
            'enable_steering_control': getattr(self.config, 'enable_steering_control', True),
            'road_type': self.config.get_road_type_name() if self.config else 'OpenRoad',
            'controller_type': self.config.get_controller_type_name() if self.config else 'CACC',
            'node_sequence': getattr(self.config, 'node_sequence', [0, 1]),
            'dummy_controller_params': getattr(self.config, 'dummy_controller_params', {}),

            # Vehicle Physics
            'use_physical_qcar': getattr(self.config, 'use_physical_qcar', False),
            'use_control_observer_mode': getattr(self.config, 'use_control_observer_mode', False),
            'calibrate': getattr(self.config, 'calibrate', False),
            
            # Sensor configuration
            'enable_cam_lidar_fusion': getattr(self.config, 'enable_cam_lidar_fusion', True),
            'enable_visualization_camera': getattr(self.config, 'enable_visualization_camera', True),
            'enable_visualization_lidar': getattr(self.config, 'enable_visualization_lidar', True),
        }
        
        return vehicle_config
    
    def _get_initial_pose(self, vehicle_id, init_position_table):
        """Extract initial pose data for a vehicle"""
        if init_position_table is not None and vehicle_id < len(init_position_table):
            pos_row = init_position_table[vehicle_id]
            vehicle_initial_pose = [
                pos_row[1],  # PositionX
                pos_row[2],  # PositionY
                pos_row[3],  # PositionZ
                pos_row[4],  # RotationX
                pos_row[5],  # RotationY
                pos_row[6]   # RotationZ
            ]
            spawn_location = [pos_row[1], pos_row[2], pos_row[3]]
            spawn_rotation = [pos_row[4], pos_row[5], pos_row[6]]
        else:
            vehicle_initial_pose = None
            spawn_location = [0, 0, 0]
            spawn_rotation = [0, 0, 0]
        
        return vehicle_initial_pose, spawn_location, spawn_rotation


class QcarFleet:
    """ QCarFleet Class
    
    :NumQcar: Number of Qcars in the Fleet
    :Distance: the distance between the following point and the target car for each follower car.
    :Controller: The controller used for each car model
    :Observer: The observer used for the fleet.

    :QcarIndexList: List of Qcar Index, from 0 to (NumQCar-1)
    """

    def __init__(self, NumQcar: int, LeaderIndex: int, Distance: float, Controller: str, Observer:str, QlabType:str = "OpenRoad", config=None, topology: str = "fully_connected"):
        """
        Controller/Observer     :str        - String The name in the 
        QlanType                :str        - Simulation Map
        Distance                :float      - The distance between the following position and the object car position 
        config                  :object     - Configuration object containing fleet parameters
        topology                :str        - Communication topology ('fully_connected', 'chain', 'star', 'custom')

        Function: Initiation and Build the fleet.
        
        Note: NumQcar supports up to 5 vehicles (modular design), but typically 3 vehicles are used.
        Each vehicle gets assigned specific ports for socket communication on localhost.
        """

        self.qlabs = QuanserInteractiveLabs()
        self.vehicle_processes = []  # Store process objects instead of Vehicle instances
        self.vehicle_configs = []   # Store vehicle configurations for processes
        self.NumQcar = NumQcar
        self.LeaderIndex = LeaderIndex
        self.QcarIndexList = range(0, self.NumQcar)
        self.Distance = Distance            
        self.Controller = Controller        
        self.Observer = Observer
        self.config = config
        self.topology = topology  # Store topology type
        
        # Initialize supporting classes for modular configuration
        self.port_manager = PortManager()
        self.graph_manager = GraphManager(self.NumQcar, self.LeaderIndex)
        self.communication_graph = CommunicationGraph(self.port_manager, self.graph_manager)
        self.config_builder = VehicleConfigBuilder(self.port_manager, self.graph_manager, 
                                                 self.communication_graph, self.config)
        
        # Multiprocessing setup
        self.stop_event = multiprocessing.Event()
        self.status_queue = multiprocessing.Queue()
        self.start_event = multiprocessing.Event()  # Add start event for synchronized fleet startup
        
        self.InitEnv(QlabType)
        #Number of the Qcars in the fleet, int
        if self.NumQcar < 2:
            print("Error: Number of cars in the fleet is too small")
            quit()
            #Check the number of cars in the fleet. 
        self.InitQcarSpawnData(QlabType)  # Only prepare spawn data, don't spawn here
        
        # if not self.LeaderIndex in self.QcarIndexList:
        #     print("Error: Leader Car Index Illegal")
        #     quit()
        # else:
        #     QLabsRealTime().start_real_time_model(self.rtModel, actorNumber=self.LeaderIndex)

        self.InitThread()  # Keep for compatibility, but won't use threading locks
        self.PrepareVehicleConfigs()  # Prepare configurations for vehicle processes
        pass


    #region: Initiation
    def InitEnv(self , QlabType:str):
        """
        Initiate the environment of the Qlab 
        """
        try:
            self.qlabs.open("localhost")
            #qlabs.open("host.docker.internal")
            print("Connected to QLabs")
        except:
            print("Error: Unable to connect to QLabs")
            quit()
        self.qlabs.destroy_all_spawned_actors()
        QLabsRealTime().terminate_all_real_time_models()

        if (QlabType == "Studio"):
            # Setup environment
            x_offset = 0.13
            y_offset = 1.67
            hFloor = QLabsQCarFlooring(self.qlabs)
            hFloor.spawn_degrees([x_offset, y_offset, 0.001], rotation=[0, 0, -90], configuration=0)
            hWall = QLabsWalls(self.qlabs)
            hWall.set_enable_dynamics(False)
            for y in range(5):
                hWall.spawn_degrees(location=[-2.4 + x_offset, (-y*1.0)+2.55 + y_offset, 0.001], rotation=[0, 0, 0])
            for x in range(5):
                hWall.spawn_degrees(location=[-1.9+x + x_offset, 3.05+ y_offset, 0.001], rotation=[0, 0, 90])
            for y in range(6):
                hWall.spawn_degrees(location=[2.4+ x_offset, (-y*1.0)+2.55 + y_offset, 0.001], rotation=[0, 0, 0])
            for x in range(4):
                hWall.spawn_degrees(location=[-0.9+x+ x_offset, -3.05+ y_offset, 0.001], rotation=[0, 0, 90])
            hWall.spawn_degrees(location=[-2.03 + x_offset, -2.275+ y_offset, 0.001], rotation=[0, 0, 48])
            hWall.spawn_degrees(location=[-1.575+ x_offset, -2.7+ y_offset, 0.001], rotation=[0, 0, 48])
            myCrossWalk = QLabsCrosswalk(self.qlabs)
            myCrossWalk.spawn_degrees(location=[-2 + x_offset, -1.475 + y_offset, 0.01], rotation=[0, 0, 0], scale=[0.1, 0.1, 0.075], configuration=0)
            mySpline = QLabsBasicShape(self.qlabs)
            mySpline.spawn_degrees(location=[2.05 + x_offset, -1.5 + y_offset, 0.01], rotation=[0, 0, 0], scale=[0.27, 0.02, 0.001], waitForConfirmation=False)


        #QLabsRealTime().terminate_all_real_time_models(RTModelHostName='host.docker.internal')
        pass

    def InitQcarSpawnData(self, QlabType:str):
        """
        Prepare spawn data for Qcars - actual spawning will happen in each process
        """

        match QlabType:
            case "OpenRoad":
                base_dir = os.path.dirname(__file__)
                csv_path = os.path.join(base_dir, "data", "QcarInitSettingOpenRoad.csv")
                InitPositionTable = pd.read_csv(csv_path)
                self.QcarScale = [1,1,1] 

            case "Studio":
                base_dir = os.path.dirname(__file__)
                csv_path = os.path.join(base_dir, "data", "QcarInitSettingStudio.csv")
                InitPositionTable = pd.read_csv(csv_path)
                self.QcarScale =  [0.1,0.1,0.1]
            case _:
                self.QcarScale = [1,1,1]
                print("Error: QlabType not found")
                quit()

        InitPositionTable = InitPositionTable.to_numpy()
        
        # Store the position table for use during vehicle process initialization
        self.InitPositionTable = InitPositionTable
        
        print(f"Prepared spawn data for {self.NumQcar} vehicles with scale {self.QcarScale}")
        pass

    def InitThread(self):
        self.lock = threading.Lock()
        pass

    def create_fleet_graph(self, graph_type="fully_connected"):
        """
        Create adjacency matrix representing the fleet vehicle connections.
        
        graph_type: str - Type of graph topology
                   "fully_connected" - All vehicles connected to all others
                   "chain" - Chain topology (each vehicle connected to neighbors)
                   "star" - Star topology (leader connected to all, others only to leader)
                   "custom" - Custom adjacency matrix (to be defined)
        
        Returns: numpy array - Adjacency matrix where graph[i][j] = 1 means vehicle i can communicate with vehicle j
        """
        n = self.NumQcar
        
        if graph_type == "fully_connected":
            # Fully connected graph - all vehicles connected to all others
            graph = np.ones((n, n), dtype=int)
            # Set diagonal to 0 (vehicle doesn't connect to itself)
            np.fill_diagonal(graph, 0)
            
        elif graph_type == "chain":
            # Chain topology - each vehicle connected to its immediate neighbors
            graph = np.zeros((n, n), dtype=int)
            for i in range(n):
                if i > 0:  # Connect to previous vehicle
                    graph[i][i-1] = 1
                if i < n-1:  # Connect to next vehicle
                    graph[i][i+1] = 1
                    
        elif graph_type == "star":
            # Star topology - leader connected to all, others only to leader
            graph = np.zeros((n, n), dtype=int)
            leader_idx = self.LeaderIndex
            for i in range(n):
                if i != leader_idx:
                    graph[leader_idx][i] = 1  # Leader to follower
                    graph[i][leader_idx] = 1  # Follower to leader
                    
        elif graph_type == "custom":
            # Custom graph - can be modified as needed
            # Example: predefined adjacency matrix
            if n == 3:
                graph = np.array([
                    [0, 1, 1],  # Vehicle 0 connects to 1, 2
                    [1, 0, 1],  # Vehicle 1 connects to 0, 2
                    [1, 1, 0]   # Vehicle 2 connects to 0, 1
                ])
            elif n == 4:
                graph = np.array([
                    [0, 1, 1, 1],  # Vehicle 0 connects to 1, 2, 3
                    [1, 0, 1, 1],  # Vehicle 1 connects to 0, 2, 3
                    [1, 1, 0, 1],  # Vehicle 2 connects to 0, 1, 3
                    [1, 1, 1, 0]   # Vehicle 3 connects to 0, 1, 2
                ])
            else:
                # Default to fully connected for other sizes
                graph = np.ones((n, n), dtype=int)
                np.fill_diagonal(graph, 0)
        else:
            raise ValueError(f"Unknown graph type: {graph_type}")
            
        return graph

    def print_fleet_graph(self, graph):
        """
        Print the fleet adjacency matrix in a readable format
        """
        print(f"\nFleet Adjacency Matrix ({self.NumQcar} vehicles):")
        print("=" * 50)
        print("   ", end="")
        for j in range(self.NumQcar):
            print(f"V{j:2d}", end=" ")
        print()
        
        for i in range(self.NumQcar):
            role = "L" if i == self.LeaderIndex else "F"
            print(f"V{i:2d}({role})", end=" ")
            for j in range(self.NumQcar):
                print(f"{graph[i][j]:2d} ", end=" ")
            print()
        print("=" * 50)
        print("L = Leader, F = Follower")
        print("1 = Connected, 0 = Not Connected")
        print()

    def PrepareVehicleConfigs(self):
        """
        Prepare configuration dictionaries for each vehicle process.
        Each process will create its own QLabs connection and spawn its vehicle.
        
        NEW DESIGN: Uses supporting classes for modular configuration building
        """
        # Create fleet communication graph
        self.fleet_graph = self.graph_manager.create_graph(self.topology)
        self.graph_manager.print_graph()
        
        # Setup peer-to-peer communication based on graph
        self.communication_graph.setup_peer_ports()
        self.peer_ports = self.communication_graph.peer_ports
        
        # Validate vehicle count doesn't exceed port configuration
        max_vehicles = self.port_manager.max_vehicles
        if self.NumQcar > max_vehicles:
            print(f"Error: Number of vehicles ({self.NumQcar}) exceeds maximum supported ({max_vehicles})")
            print(f"Reducing NumQcar to {max_vehicles}")
            self.NumQcar = max_vehicles
            self.QcarIndexList = range(0, self.NumQcar)
        
        # Build vehicle configurations using the builder pattern
        self.vehicle_configs = self.config_builder.build_vehicle_configs(
            self.NumQcar, self.LeaderIndex, self.Controller, self.Observer,
            getattr(self, 'InitPositionTable', None), self.QcarScale, self.Distance
        )
        
        # Print configuration summary for each vehicle
        for i, vehicle_config in enumerate(self.vehicle_configs):
            is_leader = vehicle_config['is_leader']
            send_port, recv_port, ack_port = self.port_manager.get_ports_for_vehicle(i)
            connected_peers = vehicle_config['connected_vehicles']
            
            print(f"Vehicle {i} {'(Leader)' if is_leader else '(Follower)'}: "
                  f"Send={send_port}, Recv={recv_port}, ACK={ack_port}")
            print(f"  Connected to vehicles: {connected_peers} ({len(connected_peers)} connections)")
            
            # Print initial pose if available
            if vehicle_config['initial_pose']:
                pose = vehicle_config['initial_pose']
                print(f"  Initial pose: x={pose[0]:.3f}, y={pose[1]:.3f}, yaw={pose[5]:.3f}")
        
        print(f"Prepared configurations for {self.NumQcar} vehicles with leader at index {self.LeaderIndex}")
        print("Graph-based communication setup - Vehicle connections defined by adjacency matrix")
        print("Port configuration complete for graph-based peer-to-peer communication")
        
        # Print connection summary
        stats = self.graph_manager.get_connectivity_stats()
        print(f"Graph connectivity: {stats['total_connections']}/{stats['max_connections']} connections ({stats['connectivity_ratio']:.1%})")
        pass

    def get_fleet_graph(self):
        """
        Get the current fleet adjacency matrix
        """
        return self.fleet_graph.copy() if hasattr(self, 'fleet_graph') else None

    def update_fleet_graph(self, new_graph):
        """
        Update the fleet adjacency matrix and reconfigure peer ports
        
        new_graph: numpy array - New adjacency matrix
        """
        if new_graph.shape != (self.NumQcar, self.NumQcar):
            raise ValueError(f"Graph shape {new_graph.shape} doesn't match fleet size ({self.NumQcar}, {self.NumQcar})")
        
        # Update the graph manager with the new graph
        self.graph_manager.graph = new_graph.copy()
        self.fleet_graph = new_graph.copy()
        
        print("Fleet graph updated:")
        self.graph_manager.print_graph()
        
        # Reconfigure peer ports based on new graph
        self.communication_graph.setup_peer_ports()
        self.peer_ports = self.communication_graph.peer_ports

    def get_vehicle_connections(self, vehicle_id):
        """
        Get list of vehicles that the specified vehicle can communicate with
        """
        return self.graph_manager.get_connected_vehicles(vehicle_id)

    def is_connected(self, vehicle_a, vehicle_b):
        """
        Check if two vehicles can communicate directly
        """
        if not hasattr(self, 'fleet_graph'):
            return False
        
        if vehicle_a >= self.NumQcar or vehicle_b >= self.NumQcar:
            return False
            
        return self.fleet_graph[vehicle_a][vehicle_b] == 1
    #endregion


    #region: Main Program for the Fleet
    def FleetBuilding(self):
        """
        Start processes for every vehicle in the fleet
        """
        print("Starting fleet vehicle processes...")
        
        for i, vehicle_config in enumerate(self.vehicle_configs):
            # Create a new process for each vehicle
            process = multiprocessing.Process(
                target=vehicle_process_main,
                args=(vehicle_config, self.stop_event, self.status_queue, self.start_event),
                name=f"Vehicle-{i}"
            )
            process.start()
            self.vehicle_processes.append(process)
            print(f"Started process for vehicle {i} ({'Leader' if vehicle_config['is_leader'] else 'Follower'})")
            time.sleep(0.5)  # Increased from 0.1 to 0.5 seconds
        
        # Wait for all vehicles to initialize
        print("Waiting for vehicle processes to initialize...")
        initialized_count = 0
        timeout = 30.0  # Increased from 20.0 to 30.0 seconds
        start_time = time.time()
        
        while initialized_count < self.NumQcar and (time.time() - start_time) < timeout:
            try:
                status = self.status_queue.get(timeout=1.0)
                if status['status'] == 'initialized':
                    initialized_count += 1
                    print(f"Vehicle {status['vehicle_id']} initialized ({initialized_count}/{self.NumQcar})")
                elif status['status'] == 'failed':
                    print(f"Vehicle {status['vehicle_id']} failed to initialize: {status.get('error', 'Unknown error')}")
            except:
                pass  # Timeout on queue get
        
        # More lenient initialization requirements
        # min_vehicles = max(1, self.NumQcar // 2)  # Require at least half the vehicles or 1 minimum
        if initialized_count >= self.NumQcar:
            # Signal all vehicles to start control simultaneously
            print("Activating fleet control - setting start event...")
            time.sleep(2.0)  # Brief pause before starting control
            self.start_event.set()
            print("Fleet control activated! Vehicles can now start moving.")
        # else:
        #     print(f"Warning: Only {initialized_count}/{self.NumQcar} vehicles initialized within timeout")
        #     if initialized_count >= 1:
        #         print("Proceeding with available vehicles...")
        #         self.start_event.set()
        #     else:
        #         print("Fleet control activation cancelled due to insufficient vehicles")
        pass

    def FleetCanceling(self):
        """
        Stop all vehicle processes in the fleet
        """
        print("Stopping fleet vehicle processes...")
        
        # Signal all processes to stop gracefully
        self.stop_event.set()
        
        # Clear the start event to reset for potential restart
        self.start_event.clear()
        
        # Give processes time to stop gracefully
        print("Waiting for vehicle processes to stop gracefully...")
        time.sleep(2.0)  # Allow 2 seconds for graceful shutdown
        
        # Wait for processes to finish gracefully
        timeout = 3.0  # 3 second timeout per process
        for i, process in enumerate(self.vehicle_processes):
            if process.is_alive():
                print(f"Waiting for vehicle {i} process to stop...")
                process.join(timeout)
                
                if process.is_alive():
                    print(f"Vehicle {i} process did not stop gracefully, terminating...")
                    process.terminate()
                    process.join(timeout=2.0)
                    
                    if process.is_alive():
                        print(f"Vehicle {i} process did not terminate, killing...")
                        process.kill()
                        process.join()
                
                print(f"Vehicle {i} process stopped")
        
        # Clear the process list
        self.vehicle_processes.clear()
        
        # Close QLabs connection in main process
        try:
            if hasattr(self, 'qlabs') and self.qlabs is not None:
                print("Closing main QLabs connection...")
                self.qlabs.close()
                print("Main QLabs connection closed")
        except Exception as e:
            print(f"Error closing QLabs connection: {e}")
        
        # Terminate any remaining real-time models
        try:
            from qvl.real_time import QLabsRealTime
            print("Terminating all real-time models...")
            QLabsRealTime().terminate_all_real_time_models()
            print("All real-time models terminated")
        except Exception as e:
            print(f"Error terminating real-time models: {e}")
        
        print("All fleet vehicle processes stopped")
        pass

    def get_fleet_status(self):
        """
        Get status of all vehicles in the fleet by checking their processes
        and querying the status queue
        """
        status = {}
        
        # Check process status
        for i, process in enumerate(self.vehicle_processes):
            status[i] = {
                'alive': process.is_alive(),
                'pid': process.pid if process.is_alive() else None,
                'state': 'running' if process.is_alive() else 'stopped'
            }
        
        # Get any recent status updates from the queue
        recent_updates = {}
        try:
            while True:
                update = self.status_queue.get_nowait()
                vehicle_id = update['vehicle_id']
                recent_updates[vehicle_id] = update
        except:
            pass  # Queue is empty
        
        # Merge recent updates into status
        for vehicle_id, update in recent_updates.items():
            if vehicle_id in status:
                status[vehicle_id].update(update)
        
        return status

    def is_fleet_alive(self):
        """
        Check if any vehicle process in the fleet is still running
        """
        return any(process.is_alive() for process in self.vehicle_processes)

    def monitor_fleet_status(self, interval=5.0):
        """
        Monitor and print fleet status periodically
        """
        try:
            while not self.stop_event.is_set() and self.is_fleet_alive():
                status = self.get_fleet_status()
                alive_count = sum(1 for v in status.values() if v['alive'])
                print(f"Fleet status: {alive_count}/{self.NumQcar} vehicles alive")
                
                # Print any status updates from queue
                try:
                    while True:
                        update = self.status_queue.get_nowait()
                        if update['status'] == 'running':
                            pos = update.get('position', [0, 0, 0])
                            vel = update.get('velocity', 0)
                            print(f"Vehicle {update['vehicle_id']}: pos=({pos[0]:.2f}, {pos[1]:.2f}), vel={vel:.2f}")
                except:
                    pass  # Queue is empty
                
                time.sleep(interval)
        except KeyboardInterrupt:
            print("Fleet monitoring interrupted")
    #endregion


    #region: API for writing the Fleet Leader and Get/Print Fleet Data
    def APIQcarInfoGet(self, CarIndex:int, InfoType:str = "position"):
        """
        Obtain the Data in current time for whole fleet or some qcar InformationType: all, position, rotation.
        
        CarIndex            :int                The index of the object Qcar
        InfoType            :str                The requirement information ("all", "position", "rotation", "state", "exist")
        
        Note: In process-based architecture, this method has limited functionality.
        Vehicle data is primarily accessible through the status queue.
        """
        if CarIndex not in self.QcarIndexList:
            print("Error: Illegal car index")
            return None

        # Check if process exists and is alive
        if CarIndex < len(self.vehicle_processes):
            process = self.vehicle_processes[CarIndex]
            
            match InfoType:
                case "exist":
                    return process.is_alive()
                case "state":
                    # Try to get recent state from status queue
                    try:
                        recent_states = {}
                        while True:
                            status = self.status_queue.get_nowait()
                            recent_states[status['vehicle_id']] = status
                    except:
                        pass
                    
                    return recent_states.get(CarIndex, {'status': 'unknown'})
                case "all" | "position" | "rotation":
                    print(f"Warning: {InfoType} data not directly available in process-based architecture")
                    print("Use get_fleet_status() or monitor status queue for vehicle data")
                    return None
                case _:
                    print("InfoType not supported in process-based architecture")
                    return None
        else:
            print(f"Error: Vehicle {CarIndex} process not found")
            return None

    def APIQcarWrite(self, CarIndex:int, SpeedCMD:float = 0, SteeringCMD:float = 0):
        """
        Write commands to a vehicle.
        
        Note: Direct vehicle control is not available in process-based architecture.
        Each vehicle process manages its own control loop independently.
        """
        print("Warning: Direct vehicle control not available in process-based architecture")
        print("Vehicle processes manage their own control loops based on their configuration")
        print(f"Requested: Vehicle {CarIndex}, Speed={SpeedCMD}, Steering={SteeringCMD}")
        return False
    
    #endregion