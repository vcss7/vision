import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageSubscriber(Node):
    """
    ImageSubscriber class inherits from the Node class.
    Subscribes to an image topic and displays the image.
    """

    def __init__(self, node_name="img_sub", topic_name="video_frames"):
        """
        Class constructor to set up the node
        """
        self.node_name = node_name
        self.topic_name = topic_name

        super().__init__("image_subscriber")

        # Subscribe to an image topic
        # TODO: Set qos_profile
        self.subscription = self.create_subscription(
            Image, self.topic_name, self.listener_callback, 1
        )
        self.subscription

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.curr_time = self.get_clock().now().nanoseconds
        self.prev_time = self.get_clock().now().nanoseconds

    def listener_callback(self, data):
        """
        Callback function that reads the image and displays it on the screen
        """
        # Caluclate FPS
        self.curr_time = self.get_clock().now().nanoseconds
        fps = 1e9 / (self.curr_time - self.prev_time)
        self.prev_time = self.curr_time

        # Display the message on the console
        self.get_logger().info("Receiving video frame")
        self.get_logger().info("FPS: {}".format(fps))

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)

        # Display image
        cv2.imshow("camera", current_frame)

        cv2.waitKey(1)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
