import rclpy
import numpy as np
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')
        
    def range_cb(self, msg):
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)
    
    def calculate_position(self):
        if not len(self.anchor_ranges):
            self.get_logger().info("0-Rückgabe")
            return 0.0, 0.0, 0.0
        
        # Extrahiere die Entfernungen der Ankerpunkte aus den empfangenen Nachrichten
        anchor_distances = [msg.range for msg in self.anchor_ranges]
        
        # Triangulationsalgorithmus zur Positionsschätzung
        # Hier ein einfaches Beispiel für eine 2D-Positionsschätzung mit 4 Ankerpunkten:
        # Annahme: Ankerpunkte befinden sich auf den Koordinaten (0, 0), (1, 0), (0, 1), (1, 1)
        
        # Berechne die Durchschnittsentfernung zu den Ankerpunkten
        mean_distance = np.mean(anchor_distances)
        
        # Berechne die Schätzung der x- und y-Koordinate des Roboters
        x = np.sqrt(mean_distance**2 / 2)  # Beispiel für x-Koordinate
        y = np.sqrt(mean_distance**2 / 2)  # Beispiel für y-Koordinate
        self.get_logger().info(f'Locator: {x} {y}')
        return x, y, 0.0  # Z-Koordinate ist hier 0, da es sich um eine 2D-Position handelt


def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
