import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for creating ROS 2 nodes
from sensor_msgs.msg import Image  # ROS 2 message type for images
from cv_bridge import CvBridge  # ROS <-> OpenCV image converter
import cv2  # OpenCV library for computer vision

class WebcamPublisher(Node):
    """
    A ROS 2 node that captures video frames from a webcam
    and publishes them as sensor_msgs/Image messages.
    """
    def __init__(self):
        super().__init__('webcam_publisher')  # Initialize the node with the name 'webcam_publisher'

        # Create a ROS 2 publisher that publishes Image messages to the 'webcam_publisher' topic
        self.publisher_ = self.create_publisher(Image, 'webcam_publisher', 10)

        # Create a CvBridge object to convert between OpenCV images and ROS Image messages
        self.bridge = CvBridge()

        # Initialize webcam capture
        # Using cv2.CAP_V4L2 explicitly for Linux V4L2 backend
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # Alternative for default backend: self.cap = cv2.VideoCapture(0)

        # Check if the webcam was successfully opened
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam at /dev/video0")
            return

        # Set webcam resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

        # Set the timer period (in seconds) for publishing frames
        # 0.03 seconds corresponds to roughly 33 frames per second
        timer_period = 0.03
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Callback function executed every timer_period seconds.
        Captures a frame from the webcam, converts it to a ROS Image message,
        and publishes it to the 'webcam_publisher' topic.
        """
        ret, frame = self.cap.read()  # Capture a frame from the webcam
        if not ret:
            self.get_logger().warn('No frame captured from webcam')
            return

        # Convert the OpenCV image (BGR format) to a ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        # Publish the Image message
        self.publisher_.publish(msg)

        # Log info for debugging
        self.get_logger().info('Publishing video frame')

    def destroy_node(self):
        """
        Override destroy_node to release the webcam before shutting down the node.
        """
        self.cap.release()  # Release the webcam resource
        super().destroy_node()  # Call parent destroy_node method

def main(args=None):
    """
    Main entry point for the ROS 2 node.
    Initializes the ROS 2 system, creates the WebcamPublisher node,
    and spins it until interrupted.
    """
    rclpy.init(args=args)  # Initialize ROS 2 Python client library
    node = WebcamPublisher()  # Create the webcam publisher node

    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down webcam publisher.')  # Log shutdown message
    finally:
        node.destroy_node()  # Properly destroy the node
        rclpy.shutdown()  # Shutdown ROS 2

if __name__ == '__main__':
    main()  # Run the main function
