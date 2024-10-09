import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.msg import SensorInfo
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import math_utils


NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])

SENSORS_RANGE = 20

class SimulationManager(Node):

    def __init__(self):

        super().__init__('simulationManager')

        self.sensor_positions = {}
        self.balloon_positions = {}
        #le nostre variabili
        self.base_station_position = None
        self.packet_distances = []
        self.total_packets_sent = 0
        self.total_packets_received = 0
        self.total_bytes_transmitted = 0 

        self.start_time = time.time()  


        for i in range(NUMBER_OF_SENSORS):

            self.create_subscription(
                Odometry,
                f'Sensor_{i}/odometry',
                lambda odometry_msg, sensor_id = i: self.store_sensor_position(sensor_id, odometry_msg),
                10
                #self.store_sensor_position
            )
            self.create_subscription(
                SensorInfo,
                f'Sensor_{i}/tx_data',
                lambda msg, sensor_id=i: self.forward_data(sensor_id, msg),
                10
            )
        self.balloons_rx = {}

        for i in range(NUMBER_OF_BALLOONS):


            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',
                lambda odometry_msg, balloon_id = i: self.store_balloon_position(balloon_id, odometry_msg),
                10
                #self.store_sensor_position
            )

            self.balloons_rx[i] = self.create_publisher(
                SensorInfo,
                f'Balloon_{i}/rx_data',
                10
            )

         # base station position
        self.create_subscription(
            Odometry,
            '/BaseStation/odometry',
            self.store_base_station_position,
            10
        )

    def store_sensor_position(self, sensor_id, position : Odometry):

        self.sensor_positions[sensor_id] = position.pose.pose.position


    def store_balloon_position(self, balloon_id, position : Odometry):

        self.balloon_positions[balloon_id] = position.pose.pose.position

    def store_base_station_position(self, position: Odometry):

        self.base_station_position = position.pose.pose.position

    def forward_data(self, sensor_id, msg : SensorInfo):
       
        self.total_packets_sent += 1

        if self.base_station_position and sensor_id in self.sensor_positions:
            sensor_position = self.sensor_positions[sensor_id]
            for i in range(NUMBER_OF_BALLOONS):

                #measure the distance from sensor to balloon, then from balloon to base station
                
                if i in self.balloon_positions:
                    balloon_position = self.balloon_positions[i]
                    distance_sensor_to_balloon = math_utils.point_distance(sensor_position, balloon_position)
                    if distance_sensor_to_balloon < SENSORS_RANGE:
                        distance_balloon_to_base = math_utils.point_distance(balloon_position, self.base_station_position)
                        total_distance = distance_sensor_to_balloon + distance_balloon_to_base
                        self.packet_distances.append(total_distance)

                        self.balloons_rx[i].publish(msg)

                        
                        self.total_packets_received += 1
                        self.total_bytes_transmitted += len(msg.data)  # Assuming 'msg.data' holds the actual payload
                        break 
    
    def metrics_evaluation(self):
        # Calculate basic metrics
        packet_loss = (self.total_packets_sent - self.total_packets_received) / self.total_packets_sent if (self.total_packets_sent != 0) else 0
        average_distance = sum(self.packet_distances) / len(self.packet_distances) if self.packet_distances else 0
        
        # New metric: Throughput (bytes per second)
        elapsed_time = time.time() - self.start_time
        throughput = self.total_bytes_transmitted / elapsed_time if elapsed_time > 0 else 0

        # Packet success rate
        packet_success_rate = (self.total_packets_received / self.total_packets_sent) * 100 if self.total_packets_sent != 0 else 0
        
        return {
            'packet_loss': packet_loss,
            'avg_distance': average_distance,
            'throughput': throughput,
            'packet_success_rate': packet_success_rate,
            'total_data_transferred': self.total_bytes_transmitted,
            'simulation_time': elapsed_time
        }


def main():

    rclpy.init() # init ROS 2 system

    simulationManager= SimulationManager()
    executor = MultiThreadedExecutor()

    # SimulationManager instance is added to the executor. 
    # It let the executor handling parallel simulationManager's callbacks.
    executor.add_node(simulationManager)

    try:
        executor.spin()
    finally:
        metrics = simulationManager.metrics_evaluation()

        # Log metrics to console
        log_message = (
            f"Packet Loss: {metrics['packet_loss'] * 100:.2f}%\n"
            f"Average Distance Traveled by Packets: {metrics['avg_distance']:.2f} units\n"
            f"Throughput: {metrics['throughput']:.2f} bytes/second\n"
            f"Packet Success Rate: {metrics['packet_success_rate']:.2f}%\n"
            f"Total Data Transferred: {metrics['total_data_transferred']} bytes\n"
            f"Total Simulation Time: {metrics['simulation_time']:.2f} seconds"
        )

        # Print the log messages in the current terminal
        print(log_message)

        # Open a new terminal window and display the log messages

    subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', f'echo "{log_message}"; read -p "Press enter to exit..."'])


    # in order to execute all nodes' callbacks an infinite loop is started 
    # 'till the program is terminated.
    executor.spin()

    simulationManager.calculate_avg_results() # print roba

    executor.shutdown()
    simulationManager.destroy_node()

    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
