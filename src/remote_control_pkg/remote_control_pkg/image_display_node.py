import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageDisplayNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw')
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.listener_callback,
            10)
        self.subscription
        self.bridge = CvBridge()
        self.get_logger().info(f'Subscribed to {image_topic}')

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
