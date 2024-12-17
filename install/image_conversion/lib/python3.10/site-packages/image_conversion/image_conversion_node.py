import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_srvs.srv import SetBool

class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion_node')
        self.bridge = CvBridge()
        self.mode = 2  # Default mode: Color

        # Declare parameters
        self.declare_parameter('input_topic', '/usb_cam/image_raw')
        self.declare_parameter('output_topic', '/converted_image')

        # Get parameters
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Subscribers and publishers
        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, 10)
        self.publisher = self.create_publisher(Image, output_topic, 10)

        # Service to change mode
        self.srv = self.create_service(SetBool, 'change_mode', self.change_mode_callback)

        self.get_logger().info('ImageConversionNode started.')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.mode == 1:
                # Convert to grayscale
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            # Publish the image
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.publisher.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def change_mode_callback(self, request, response):
        if request.data:
            self.mode = 1
            response.success = True
            response.message = "Switched to Grayscale mode"
        else:
            self.mode = 2
            response.success = True
            response.message = "Switched to Color mode"
        self.get_logger().info(response.message)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

