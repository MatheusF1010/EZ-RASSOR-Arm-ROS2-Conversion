import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, CameraInfo, Image
import time

class CameraImage(Node):
    def __init__(self):

        self.current_frame = None

        super().__init__("camera_image")
        self.get_logger().info('Created camera_image node')

        # Receive the image from rover's camera
        self.image_receiver = self.create_subscription(Image, "/ezrassor/depth_camera/image_raw", self.on_pic_received, 10)


        UPDATE_TIME = 1.0 # picture will be updated every second
        self.image_publisher = self.create_publisher(Image, "/frame_img", 10)
        self.timer = self.create_timer(UPDATE_TIME, self.send_image)


    def on_pic_received(self, img):
        '''
        Updates current image frame without hz limit
        '''
        self.current_frame = img

    def send_image(self):
        '''
        Sends current image frame capped at 1 frame per second
        '''
        if self.current_frame is not None:
            self.image_publisher.publish(self.current_frame)
            self.get_logger().info('Published image')
    
def main(args=None):
    rclpy.init(args=args)

    camera_demo = CameraImage()

    rclpy.spin(camera_demo)
    camera_demo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()