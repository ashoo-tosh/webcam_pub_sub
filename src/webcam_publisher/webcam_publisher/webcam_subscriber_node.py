import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for creating ROS 2 nodes
from sensor_msgs.msg import Image  # ROS 2 message type for images
from cv_bridge import CvBridge  # Converts between ROS Image messages and OpenCV images
import cv2  # OpenCV library for displaying images

class WebcamSubscriber(Node):
    """
    A ROS 2 node that subscribes to the 'webcam_publisher' topic,
    converts incoming ROS Image messages to OpenCV images, and displays them.
    """
    def __init__(self):
        super().__init__('webcam_subscriber')  # Initialize the node with the name 'webcam_subscriber'

        # Create a subscription to the 'webcam_publisher' topic
        # The callback function listener_callback will be called whenever a new message arrives
        self.subscription = self.create_subscription(
            Image,                    # Message type
            'webcam_publisher',       # Topic name
            self.listener_callback,   # Callback function
            10                        # QoS history depth
        )
        # This line is optional, keeps a reference to the subscription
        self.subscription  

        # CvBridge object for converting ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Log that the node has started successfully
        self.get_logger().info("Webcam Subscriber Node has started.")

    def listener_callback(self, msg):
        """
        Callback function executed whenever a new Image message is received.
        Converts the ROS Image message to an OpenCV image and displays it.
        """
        try:
            # Convert ROS Image message to OpenCV image (BGR format)
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

            # Display the image in a window
            cv2.imshow("Webcam Feed", frame)

            # WaitKey is required to refresh the OpenCV window; 1ms delay keeps it responsive
            cv2.waitKey(1)
        except Exception as e:
            # Log any conversion errors
            self.get_logger().error(f"Error converting image: {e}")
    

def main(args=None):
    """
    Main entry point for the ROS 2 subscriber node.
    Initializes ROS 2, creates the node, spins it, and properly shuts down.
    """
    rclpy.init(args=args)  # Initialize ROS 2 Python client library
    node = WebcamSubscriber()  # Create the webcam subscriber node

    try:
        rclpy.spin(node)  # Keep the node running to receive messages
    except KeyboardInterrupt:
        pass  # Allow graceful shutdown on Ctrl+C
    finally:
        node.destroy_node()  # Properly destroy the node
        rclpy.shutdown()     # Shutdown ROS 2
        cv2.destroyAllWindows()  # Close OpenCV windows


if __name__ == '__main__':
    main()  # Run the main function
