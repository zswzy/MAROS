# adjust theta_bar by reading user's input

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


from std_msgs.msg import Float64


start_messsage = '''
Enter a float number between 0-50 to change theta_bar
if 0 --> time-triggered control
Larger value leads to longer inter-event time,
but may cause performance degradation
'''

def main():

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1
    )

    rclpy.init()
    node = rclpy.create_node('etm_adjuster')


    

    publisher_theta_bar = node.create_publisher(
        Float64, 
        '/etm/theta_bar', 
        qos_profile) 
    
    user_value = None

    try:
        print(start_messsage)
        while True:
            user_value = float(input("Enter a positive value (recommanded between 0-2000) : "))
            node.get_logger().info(f"change theta_bar to: {user_value:.3f}")

            msg = Float64()
            msg.data = user_value
            publisher_theta_bar.publish(msg)

    except Exception as e:
        print(e)

if __name__ == '__main__':
    main()