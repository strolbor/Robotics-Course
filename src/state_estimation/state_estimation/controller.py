import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from sensor_msgs.msg import LaserScan
from math import sqrt, acos
import numpy as np

class VelocityController(Node):


    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0
        self.goal = None
        self.position = None
        self.lastpos = []
        self.lastalpha = 0.0
        self.logger = 1
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')
        
    def timer_cb(self):
        letzterEintrag =0
        laenge_ziel = 0.0

        # Message Objekt erstellen
        msg = Twist()


        # Lastpos
        #                 0,1,2
        #                 x,y,d

        if self.position != None:
            positiondaten = [self.position[0],self.position[1]]
            if positiondaten not in self.lastpos:
                self.lastpos.append(positiondaten)
                self.lastpos = self.lastpos[-5:]

                if self.logger == 1:
                    #some logging
                    self.get_logger().info(f"lastpos: {self.lastpos} ")
                    if self.position != None:
                        self.get_logger().info(f"Position: {self.position[0]} und {self.position[1]}")
                    if self.goal != None:
                        self.get_logger().info(f"Goal: {self.goal[0]} und {self.goal[1]}")

            
        letzterEintrag = len(self.lastpos)-1
        ex = 0.0 # Speed
        ez = 0.0 # Drehung
        RIGTHTURN   = 0.3
        LEFTTURN    = -0.3
        if(letzterEintrag>=2):

            # Letzte Position vom Roboter Vektor
            #                               x                                   x                           y                           y
            
            # Vektor -> Ende - Anfang
            #           Goalpos - Roboterpos

            
            # Bestandteil x vom Vektor
            b1 = self.goal[0] - self.position[0]
            # BEstandteil y vom Vektor
            b2 = self.goal[1] - self.position[1]

            # Roboter zum Tiel Winkel
                                
            
            vecA = np.array([b1,b2])
            

            # Bestandteil x vom Vektor
            c1 = self.lastpos[letzterEintrag][0] - self.lastpos[letzterEintrag-1][0]
            # Bestandteil y vom Vektor
            c2 = self.lastpos[letzterEintrag][1] - self.lastpos[letzterEintrag-1][1]
            
            # Roboter gefahren WInkel
            vecB = np.array([c1,c2])


            # Winkel zum Ziel by Urs
            tmp = np.dot(vecA,vecB)/(2*np.linalg.norm(vecA)*np.linalg.norm(vecB)) # sKALARPORDUKT#
            alpha = np.arccos(tmp)
                       
            # Winkel zum Ziel by Florian

            #tmp = np.dot(vecA,vecB)/(np.linalg.norm(vecA)*np.linalg.norm(vecB)) # sKALARPORDUKT#
            #alpha = np.arccos(tmp)
            
           
            # selfh.heading @ (self.goal - position) / linealg.norm()
            
                       

            laenge_ziel = np.linalg.norm(vecB)

            ## Logging
            if self.lastalpha != alpha:
                if self.logger == 1:
                    self.get_logger().info(f"Zum Ziel Winkel: {alpha}")
                    self.get_logger().info(f"Laenge: {laenge_ziel}")
                    self.get_logger().info(f"vecA: {vecA}")
                    self.get_logger().info(f"vecB: {vecB}")
                
                self.lastalpha = alpha

            xdis = self.forward_distance

            
            
            if xdis < 0.3:
                ex = -0.2
            else:
                ex = 0.1
                ez = -0.1 * alpha

        
        
        msg.linear.x = ex
        msg.angular.z = ez
        self.publisher.publish(msg)

    def goal_cb(self, msg):
        goal = msg.pose.position.x, msg.pose.position.y
        if self.goal != goal:
            self.get_logger().info(f'received a new goal: (x={goal[0]}, y={goal[1]})')
            self.goal = goal
    
  

    def laser_cb(self, msg):
        r = msg.ranges
        r = [x if x>= msg.range_min and x < msg.range_max else 10.0 for x in r]
        self.forward_distance = min(r[:45] + r[-45:])
        
    def position_cb(self, msg):
        self.position = msg.point.x, msg.point.y
        


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
