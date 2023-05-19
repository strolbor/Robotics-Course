import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

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
    # msg.angular.z da dreht sich der Roboter um die eigene Achse gany wild
    # msg.angular.y macht nichts
    #
    # TODO: auch seitlichen checken
    # 
    ##
    def timer_cb(self):
        msg = Twist()
        x = 0.0
        anY = 0.0
        if self.ranges is not None:
            x = self.ranges[0] - 0.3
            self.get_logger().info(f'{len(self.ranges)} hat Werte: 0: {self.ranges[0]} 45: {self.ranges[45]} 90: {self.ranges[90]} 135: {self.ranges[135]} 225: {self.ranges[225]} 270: {self.ranges[270]}')
            anY = 0.0
            if x < 0.1: # Man kann nicht weiter Vorgehen
                x = x
                anY = -0.1

            else: # Kann gradeaus fahren
                x= 0.1

            
            if  x >= 0: # Kann grade aus fahren
                x = x

            else: # Man kann nicht weiter vorgehen
                x = 0.0
                anY = -0.1


        msg.linear.x = x
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
