import rclpy
import numpy as np
from rclpy.node import Node

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped
import numpy as np
from math import sqrt

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
            return 0.0, 0.0, 0.0

        # Nachricht von Basti, wie wir die Werte angeben
        # ABER wollte nicht so wie ich

        # msg.range # float  -> self.anchor_ranges[0].range
        # msg.anchor # point 3d ->  self.anchor_ranges[0].anchor
        # msg.anchor.x #float ->  self.anchor_ranges[0].anchor.x
        # msg.anchor.y #float
        # msg.anchor.z #float
        

        # Korrekte Syntax herausfinden, um unsere Funktion zu füttern
        #i =0
        #for r in self.anchor_ranges:
        #    self.get_logger().info(f"Test {i}: {r.range} @ {r.anchor}")
        #    i += 1

        # YOUR CODE GOES HERE:
        # Urs
        if len(self.anchor_ranges) >= 4:
            ## Point 1
            P1 = [self.anchor_ranges[0].anchor.x,self.anchor_ranges[0].anchor.y,self.anchor_ranges[0].anchor.z]
            r1 = self.anchor_ranges[0].range

            ## Point 2
            P2 = [self.anchor_ranges[1].anchor.x,self.anchor_ranges[1].anchor.y,self.anchor_ranges[1].anchor.z]
            r2 = self.anchor_ranges[1].range

            ## Point 3
            P3 = [self.anchor_ranges[2].anchor.x,self.anchor_ranges[2].anchor.y,self.anchor_ranges[2].anchor.z]
            r3 = self.anchor_ranges[2].range

            ## Point 4
            P4 = [self.anchor_ranges[3].anchor.x,self.anchor_ranges[3].anchor.y,self.anchor_ranges[3].anchor.z]
            r4 = self.anchor_ranges[3].range

            x,y,z = self.trilateration(P1,P2,P3,P4,r1,r2,r3,r4)

            #self.get_logger().info(f"Test Position: ({x} {y} {z})")
            return x,y,z
        else:
            return 0.0, 0.0, 0.0


    # Tom hat es gemacht
    def trilateration(self, P1, P2, P3, P4, r1, r2, r3, r4):
        p1 = np.array([0, 0, 0])
        p2 = np.array([P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]])
        p3 = np.array([P3[0] - P1[0], P3[1] - P1[1], P3[2] - P1[2]])
        v1 = p2 - p1 # was ist hier der Fehler
        v2 = p3 - p1

        Xn = (v1) / np.linalg.norm(v1)

        tmp = np.cross(v1, v2)

        Zn = (tmp) / np.linalg.norm(tmp)

        Yn = np.cross(Xn, Zn)

        i = np.dot(Xn, v2)
        d = np.dot(Xn, v1)
        j = np.dot(Yn, v2)

        X = ((r1 ** 2) - (r2 ** 2) + (d ** 2)) / (2 * d)
        Y = (((r1 ** 2) - (r3 ** 2) + (i ** 2) + (j ** 2)) / (2 * j)) - ((i / j) * (X))
        Z1 = np.sqrt(max(0, r1 ** 2 - X ** 2 - Y ** 2))
        Z2 = -Z1

        K1 = P1 + X * Xn + Y * Yn + Z1 * Zn
        K2 = P1 + X * Xn + Y * Yn + Z2 * Zn

        d1 = sqrt( (P4[0] - K1[0])**2 + (P4[1] - K1[1])**2 + (P4[2] - K1[2])**2 )
        d2 = sqrt( (P4[0] - K2[0])**2 + (P4[1] - K2[1])**2 + (P4[2] - K2[2])**2 )

        K1, K2 = [round(i, 5) for i in K1], [round(i, 5) for i in K2]

        if abs(d1 - r4) < abs(d2 - r4):
            return K1
        else:
            return K2



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
