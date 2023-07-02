import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImagePublisher(Node):
    """
    ImagePublisher class inherits from the Node class.
    Publishes video frames from the webcam to a ROS topic.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # TODO: Set the node name with a custom name.
        super().__init__("image_publisher")

        # Publisher for video_frames topic with queue size 10
        # TODO: Set the topic name with a custom name.
        #   queue_size should probably be 1
        #   set qos_profile
        self.publisher_ = self.create_publisher(Image, "video_frames", 10)

        # TODO: Minimize the delay between the published messages.
        timer_period = 0.025  # seconds

        # Create the timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a VideoCapture object
        # TODO: Set variable for custom camera device index
        self.cap = cv2.VideoCapture(0)

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

        # Calculate fps
        fps = 1 / (self.curr_time - self.prev_time) * 1e9
        self.prev_time = self.curr_time
        self.curr_time = self.get_clock().now().nanoseconds

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
    image_publisher = ImagePublisher()

    # Spin the node so the callback function is called.
    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    image_publisher.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
