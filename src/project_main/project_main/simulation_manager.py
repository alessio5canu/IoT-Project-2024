import sys
import math
import time
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from project_interfaces.msg import SensorData
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

import math_utils

NUMBER_OF_BALLOONS = int(sys.argv[1])
NUMBER_OF_SENSORS = int(sys.argv[2])
SENSORS_RANGE = 20

class SimulationManager(Node):

    def __init__(self):
        super().__init__('simulation_manager')
        
        self.sensor_positions = {}
        self.balloon_positions = {}
        self.base_station_position = None
        self.packet_distances = []
        self.total_packets_sent = 0
        self.total_packets_received = 0
        self.start_time = time.time()  

        for i in range(NUMBER_OF_SENSORS):
            self.create_subscription(
                Odometry,
                f'Sensor_{i}/odometry',
                lambda odometry_msg, sensor_id=i: self.store_sensor_position(sensor_id, odometry_msg),
                10
            )
            self.create_subscription(
                SensorData,
                f'Sensor_{i}/tx_data',
                lambda msg, sensor_id=i: self.forward_data(sensor_id, msg),
                10
            )

        self.balloons_rx = {}
        for i in range(NUMBER_OF_BALLOONS):
            self.create_subscription(
                Odometry,
                f'Balloon_{i}/odometry',
                lambda odometry_msg, balloon_id=i: self.store_balloon_position(balloon_id, odometry_msg),
                10
            )
            self.balloons_rx[i] = self.create_publisher(
                SensorData,
                f'Balloon_{i}/rx_data',
                10
            )

        # To get the base station position
        self.create_subscription(
            Odometry,
            '/BaseStation/odometry',
            self.store_base_station_position,
            10
        )

    def store_sensor_position(self, sensor_id, position: Odometry):
        self.sensor_positions[sensor_id] = position.pose.pose.position

    def store_balloon_position(self, balloon_id, position: Odometry):
        self.balloon_positions[balloon_id] = position.pose.pose.position

    def store_base_station_position(self, position: Odometry):
        self.base_station_position = position.pose.pose.position


    def forward_data(self, sensor_id, msg: SensorData):
        self.total_packets_sent += 1

        if self.base_station_position and sensor_id in self.sensor_positions:
            sensor_position = self.sensor_positions[sensor_id]
            for i in range(NUMBER_OF_BALLOONS):

                if i in self.balloon_positions:
                    balloon_position = self.balloon_positions[i]
                    distance_sensor_to_balloon = math_utils.point_distance(sensor_position, balloon_position)

                    if distance_sensor_to_balloon < SENSORS_RANGE:
                        distance_balloon_to_base = math_utils.point_distance(balloon_position, self.base_station_position)
                        total_distance = distance_sensor_to_balloon + distance_balloon_to_base
                        self.packet_distances.append(total_distance)

                        # Publish message
                        self.balloons_rx[i].publish(msg)

                        self.total_packets_received += 1
                        self.get_logger().info(f"Packet forwarded to Balloon {i} | Distance: {total_distance:.2f}")
                        break

    def calculate_metrics(self):
        packet_loss = (self.total_packets_sent - self.total_packets_received) / self.total_packets_sent if (self.total_packets_sent != 0) else 0
        average_distance = sum(self.packet_distances) / len(self.packet_distances) if self.packet_distances else 0
        elapsed_time = time.time() - self.start_time

        # Calculate packet success rate
        packet_success_rate = (self.total_packets_received / self.total_packets_sent) * 100 if self.total_packets_sent > 0 else 0

        return packet_loss, average_distance, elapsed_time, self.total_packets_sent, packet_success_rate

def main():
    rclpy.init()

    simulation_manager = SimulationManager()
    executor = MultiThreadedExecutor()
    executor.add_node(simulation_manager)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        packet_loss, avg_distance, elapsed_time, total_packets_sent, packet_success_rate = simulation_manager.calculate_metrics()

        # Log metrics to console and also launch a new terminal window to display them
        print(f"Packet Loss: {packet_loss * 100:.2f}%")
        print(f"Average Distance Traveled by Packets: {avg_distance:.2f} units")
        print(f"Total Simulation Time: {elapsed_time:.2f} seconds")
        print(f"Total Packets Sent: {total_packets_sent}")
        print(f"Packet Success Rate: {packet_success_rate:.2f}%")

        # Show log messages in a new terminal window
        log_message = (
            f"Packet Loss Rate: {packet_loss * 100:.2f}%\n"
            f"Packet Success Rate: {packet_success_rate:.2f}%"
            f"Average Distance Traveled by Packets: {avg_distance:.2f} units\n"
            f"Total Simulation Time: {elapsed_time:.2f} seconds\n"
            f"Total Packets Sent: {total_packets_sent}\n"
         
        )
        # Open a new terminal window and display the log messages
        subprocess.Popen(['xterm', '-e', 'echo "{}"; read -p \"Press enter to exit...\"'.format(log_message)])

        executor.shutdown()
        simulation_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
