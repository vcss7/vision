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

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # TODO: set the node name with a custom name
        super().__init__("image_subscriber")

        # Subscribe to an image topic
        # TODO: Set the topic name with a custom name
        #   queue size should probably be 1
        #   set qos_profile
        self.subscription = self.create_subscription(
            Image, "video_frames", self.listener_callback, 10
        )
        self.subscription

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def listener_callback(self, data):
        """
        Callback function that reads the image and displays it on the screen
        """
        # Display the message on the console
        self.get_logger().info("Receiving video frame")

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
