import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import random


class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.ranges = None
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        self.data = None
        
    ##
    #
    #                       sensor_oben
    #   sensor_hinten        ROBOT        sensor_vorne
    #                       sensor_unten
    ##
    def timer_cb(self):
        msg = Twist()
        # Variabeln Init
        sensor_vorne = 0.0
        anY = 0.0
        ex = 0.0
        if self.ranges is not None:
            sensor_vorne = self.ranges[0] - 0.3
            sensor_oben = self.ranges[90]
            sensor_hinten = self.ranges[180]
            sensor_unten = self.ranges[270]
            
            #self.get_logger().info(f'0: {self.ranges[0]} 45: {self.ranges[45]} 90: {self.ranges[90]} 135: {self.ranges[135]} 225: {self.ranges[225]} 270: {self.ranges[270]}')

            # TODO: rogrammieren
            # Abfrage, das rechts gesperrt ist
            if  sensor_vorne > 1: # Kann grade aus fahren
                ex = 0.4 # Wenn weit vorne nichts ist; können wir schnell fahren
            elif sensor_vorne <= 0.3:
                # Wenn vorne nicht weiter geht und alles andere ist frei, dann drehe dich trotzdem
                ex = 0.1
                anY = -0.3
                #self.get_logger().info('Option A')
            else: 
                ex = 0.2 # Wir sind bald an einer Wand und müssen langsamer fahren
            if sensor_vorne <= 0.5 and sensor_oben <= 0.5:
                ex = 0.1
                anY = -0.4
                self.get_logger().info('Option B: Sensor vorne und oben melden nah')
            if sensor_oben <= 0.5:
                self.get_logger().info('Option C: Sensor oben meldet zu nah')
                anY = -0.4
                #anY += random.random()*-1
            

        msg.linear.x = ex
        msg.angular.z = anY # bewirkt das der roboter um die eigene achse dreht
        self.publisher.publish(msg)
    
    def laser_cb(self, msg):
        #self.ranges = msg.ranges[0]
        self.ranges = msg.ranges



def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
