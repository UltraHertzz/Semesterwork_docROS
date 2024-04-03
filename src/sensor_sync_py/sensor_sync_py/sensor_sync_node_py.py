import rclpy
from rclpy.node import Node
import message_filters
from sensor_msgs.msg import Imu, LaserScan

class MySyncedNode(Node):
    def __init__(self):
        super().__init__('sensor_sync_node_py')
        laser_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        imu_sub = message_filters.Subscriber(self, Imu, '/bno055/imu')
        self.laser_sync_pub = self.create_publisher(LaserScan, '/synced_scan', 10)
        self.imu_sync_pub = self.create_publisher(Imu, '/synced_imu',10)
        
        # ApproximateTimeSynchronizer
        ats = message_filters.ApproximateTimeSynchronizer([laser_sub, imu_sub], queue_size=10, slop=0.01)
        #ats = message_filters.TimeSynchronizer([laser_sub,imu_sub],queue_size=10)
        ats.registerCallback(self.callback)

    def callback(self, laser_sub, imu_sub):
        # publish synchronized laser, imu message
        self.get_logger().info('Received synchronized messages')
        self.laser_sync_pub.publish(laser_sub)
        self.imu_sync_pub.publish(imu_sub)

def main(args=None):
    rclpy.init(args=args)
    node = MySyncedNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
