# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from sensor_msgs.msg import Image

# OpenCV
from cv_bridge import CvBridge
import cv2


class ImagePublisher(Node):
    """
    ImagePublisher class inherits from the Node class.
    Publishes video frames from the webcam to a ROS topic.
    """

    def __init__(self, node_name="img_pub", device_index=0, topic_name="video_frames"):
        """
        Class constructor to set up the node
        """
        self.node_name = node_name
        self.device_index = device_index
        self.topic_name = topic_name

        super().__init__(self.node_name)

        # Publisher for video_frames topic with queue size 10
        # TODO: Set qos_profile
        self.publisher_ = self.create_publisher(Image, self.topic_name, 1)

        # How fast to publish messages.
        timer_period = 0.025  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a VideoCapture object
        self.cap = cv2.VideoCapture(self.device_index)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # keep track of time for publishing messages
        self.prev_time = self.get_clock().now().nanoseconds
        self.curr_time = self.get_clock().now().nanoseconds

    def timer_callback(self):
        """
        Callback function that publishes a video frame
        """
        # Capture frame-by-frame
        ret, frame = self.cap.read()

        # Calculate FPS
        self.curr_time = self.get_clock().now().nanoseconds
        fps = 1e9 / (self.curr_time - self.prev_time)
        self.prev_time = self.curr_time

        if ret:
            # Publish the image.
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))

        # Display the message on the console
        self.get_logger().info("Publishing video frame")
        self.get_logger().info("FPS: {}".format(fps))


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_publisher = ImagePublisher(device_index=2)

    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    image_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
