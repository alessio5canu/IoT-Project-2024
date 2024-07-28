import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
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
        #le nostre variabili
        self.packet_delays_list = []
        self.packet_distances_list = []
        self.total_packets_sent = 0
        self.total_packets_received = 0

        for i in range(NUMBER_OF_SENSORS):

            self.create_subscription(
                Odometry,
                f'Sensor_{i}/odometry',
                lambda odometry_msg, sensor_id = i: self.store_sensor_position(sensor_id, odometry_msg),
                10
                #self.store_sensor_position
            )
            self.create_subscription(
                String,
                f'Sensor_{i}/tx_data',
                lambda string_msg, sensor_id = i: self.forward_data(sensor_id, string_msg),
                #self.forward_data,
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
                String,
                f'Balloon_{i}/rx_data',
                10
            )

    def store_sensor_position(self, sensor_id, position : Odometry):

        self.sensor_positions[sensor_id] = position.pose.pose.position


    def store_balloon_position(self, balloon_id, position : Odometry):

        self.balloon_positions[balloon_id] = position.pose.pose.position


    def forward_data(self, sensor_id, msg : String):
        #ogni volta che il sensore un pacchetto
        self.total_packets_sent += 1

        for i in range(NUMBER_OF_BALLOONS):
            if sensor_id in self.sensor_positions and i in self.balloon_positions:
                
                if math_utils.point_distance(self.sensor_positions[sensor_id], self.balloon_positions[i]) < SENSORS_RANGE:
                    self.balloons_rx[i].publish(msg)
                    #ogni volta che un baloon invia un pacchetto
                    self.total_packets_received += 1
                    print("ciao2") 
                    self.get_logger().info("ciao2") 
    
    def calculate_avg_results(self):
        # packets delay average calculation
        #packets_delay_avg = 
        # packets loss average calculation
        packets_loss_avg = (self.total_packets_sent - self.total_packets_received) / self.total_packets_sent if (self.total_packets_sent > 0) else 0
        # packets distance average calculation
        packets_distance_avg = sum(self.packet_distances_list) / len(self.packet_distances_list) if self.packet_distances_list else 0

        print(f"Average Packet Loss: {packets_loss_avg:.2f}%")
        print(f"Average Packet Distance: {packets_distance_avg:.2f}%")

def main():

    rclpy.init() # init ROS 2 system

    simulationManager= SimulationManager()
    executor = MultiThreadedExecutor()

    # SimulationManager instance is added to the executor. 
    # It let the executor handling parallel simulationManager's callbacks.
    executor.add_node(simulationManager)
    print("ciao") 
    self.get_logger().info("ciao") 

    # in order to execute all nodes' callbacks an infinite loop is started 
    # 'till the program is terminated.
    executor.spin()

    simulationManager.calculate_avg_results() # print roba

    executor.shutdown()
    simulationManager.destroy_node()

    rclpy.shutdown()
    
