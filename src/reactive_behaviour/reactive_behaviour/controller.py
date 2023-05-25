##
# Scoring: [scoring-13] [INFO] [1684840962.783593249] [robotA.scoring_node]: score: 39.82 at t=240.0s
#       ohne Random
# [scoring-13] [INFO] [1684841325.213964539] [robotA.scoring_node]: score: 39.88 at t=240.0s
#       mit Random
#
# ----
# mehr if unten
# Score:
#    [scoring-13] [INFO] [1684914921.764265498] [robotA.scoring_node]: score: 38.2 at t=240.0s
#    [scoring-13] [INFO] [1684914922.266787270] [robotA.scoring_node]: score: 38.2 at t=240.5s
#   
#
## 

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
        

    def timer_cb(self):
        msg = Twist()
        # Variabeln Init
        # --> Diese Variabeln werden benötigt, um unsere Befehle zu übertragen
        anY = 0.0
        ex = 0.0
        # TURNS & SPEEDs
        RIGTHTURN   = 0.3
        LEFTTURN    = -0.3
        HIGHSPEED   = 0.4
        MEDIUMSPEED = 0.2
        LOWSPEED    = 0.1
        if self.ranges is not None:
            ##
            #
            #                       sensor_oben
            #   sensor_hinten        ROBOT--->       sensor_vorne  
            #                       sensor_unten
            ##
            sensor_vorne = self.ranges[0]
            sensor_oben = self.ranges[90]
            sensor_hinten = self.ranges[180]
            sensor_unten = self.ranges[270]
            
            # TODO: rogrammieren
            if sensor_vorne > 1.3:
                #self.get_logger().info('Option A')
                # Wenn vorne frei ist, fahre schnell gradeaus
                ex = HIGHSPEED
                #if random.randint(0,10) <= 4:
                #    anY = 2*LEFTTURN
                #    self.get_logger().info('Option RANDOM Integrant')
            elif sensor_vorne <= 0.8:
                # Wir sind nah an der Wand und müssen uns drehen
                #self.get_logger().info('Option B')
                if sensor_oben > sensor_unten:
                    anY = RIGTHTURN
                else:
                    anY = LEFTTURN
                ex = LOWSPEED
            else:
                #self.get_logger().info('Option C')
                # Wir sind noch bisschen von der Wand entfernt, also können wir schneller drehen
                if sensor_oben > sensor_unten:
                    anY = RIGTHTURN
                else:
                    anY = LEFTTURN       
                ex = MEDIUMSPEED
            if sensor_oben <= 0.8 and anY == 0.0:
                anY = LEFTTURN
            if sensor_unten <= 0.8 and anY == 0.0:
                anY = RIGTHTURN
            

        msg.linear.x = ex # bewirkt, das der roboter fährt
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
