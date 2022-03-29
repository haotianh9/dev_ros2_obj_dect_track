import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of node
from geometry_msgs.msg import Point
import numpy as np
import time
from gpiozero import Servo

PIN_H=20
PIN_W=21
class KalmanFilter(Node):
    """
    Create an ImageObjectDetection class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('kalman_filter')
        self.subscription = self.create_subscription(Point,'center_pos', self.listener_callback, 10)
        self.subscription
        self.dt=0.1
        self.rec_time=None
        self.est_pos=None
        self.est_vel=None
        self.horizontal_servo=Servo(PIN_H)
        self.vertical_servo=Servo(PIN_W)
    def listener_callback(self, data):
        """
        Callback function.
        """
        self.get_logger().info('Receiving position')
        
        if self.rec_time == None:
            self.rec_time=time.time()
            self.est_pos=np.array([data.x,data.y])
            self.est_vel=np.array([0.0,0.0])
        else:
            t_1=self.rec_time
            self.rec_time=time.time()
            old_pos=self.est_pos
            self.est_pos=np.array([data.x,data.y])
            self.est_vel=(self.est_pos-old_pos)/(self.rec_time-t_1)
            n_step=int(((self.rec_time-t_1)/self.dt)/2)
            for i in range(n_step):
                self.horizontal_servo.value=0.1*(self.est_pos[0]+i*self.dt*self.est_vel[0]-0.5)
                self.vertical_servo.value=0.1*(self.est_pos[1]+i*self.dt*self.est_vel[1]-0.5)
                time.sleep(self.dt)
def main(args=None):
    
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    kalman_filter = KalmanFilter()
    
    # Spin the node so the callback function is called.
    rclpy.spin(kalman_filter)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    kalman_filter.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()