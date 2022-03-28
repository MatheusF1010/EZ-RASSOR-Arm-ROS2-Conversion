# Import necessary dependencies
import rclpy
from rclpy.node import Node
from rclpy.time import Time
# from .pointcloud_processor import PointCloudProcessor
from sensor_msgs.msg import PointCloud2, CameraInfo, LaserScan
from std_msgs.msg import String
import numpy as np
import image_geometry

def angle_between_rays(ray1, ray2):
    return np.arccos(np.clip(np.dot(ray1, ray2), -1.0, 1.0))

class ObstacleDetector(Node):
    XYZ = {"RIGHT": 0, "DOWN": 1, "FORWARD": 2}

    def __init__(
        self,
        max_angle,
        max_obstacle_dist,
        min_hole_diameter,
        scan_time,
        range_min,
        range_max):

        self.max_slope = np.tan(max_angle * np.pi / 180.0)
        self.max_obstacle_dist = max_obstacle_dist
        self.min_hole_diameter = min_hole_diameter
        self.scan_time = scan_time
        self.range_min = range_min
        self.range_max = range_max

        super().__init__("obstacle_detector")
        self.get_logger().info('Created node')
        self.point_cloud = None

        # Publisher nodes w/ depth camera data
        # /ezrassor/depth_camera/camera_info
        # /ezrassor/depth_camera/depth/camera_info
        # /ezrassor/depth_camera/depth/image_raw
        # /ezrassor/depth_camera/image_raw
        # /ezrassor/depth_camera/points


        camera_info_topic = "ezrassor/depth_camera/camera_info" #"camera/depth/camera_info"
        # Create the subscribers
        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.init_pc_info, 10)
        self.get_logger().info('Created camera_info_sub')
        self.camera_info_sub
        self.get_logger().info('Awaiting {}'.format(camera_info_topic))


        point_cloud_topic= "ezrassor/depth_camera/points" #"camera/depth/points"
        self.pt_sub = self.create_subscription(PointCloud2, point_cloud_topic, self.on_pc_update, 10)
        self.get_logger().info('Created PointCloud2 subs')
        self.pt_sub

        # Testing sub
        # self.strsub = self.create_subscription(String,'topic',self.listener_callback,10)
        # self.get_logger().info('Created str sub')
        # self.strsub
        
        # Create the publishers
        self.hike_pub = self.create_publisher(LaserScan, "/obstacle_detection/hike", 10)
        self.slope_pub = self.create_publisher(LaserScan, "/obstacle_detection/slope", 10)
        self.combined_pub = self.create_publisher(LaserScan, "/obstacle_detection/combined", 10)

    def listener_callback(self, msg):
        self.get_logger().info("I heard {} {}".format(msg.data, str(self.get_clock().now().to_msg().sec)))
    
    def init_pc_info(self, camera_info):
        self.get_logger().info('I received camera_info data')
        #self.get_logger().info(camera_info)
        # Initialize camera
        cam_model = image_geometry.PinholeCameraModel()
        cam_model.fromCameraInfo(camera_info)
        width = camera_info.width
        self.get_logger().info('Camera width: {}'.format(width))
        # Find vector for leftmost view in camera
        raw_pixel_left = (0, cam_model.cy())
        rect_pixel_left = cam_model.rectifyPoint(raw_pixel_left)
        left_ray = cam_model.projectPixelTo3dRay(rect_pixel_left)

        # Find vector for rightmost view in camera
        raw_pixel_right = (width - 1, cam_model.cy())
        rect_pixel_right = cam_model.rectifyPoint(raw_pixel_right)
        right_ray = cam_model.projectPixelTo3dRay(rect_pixel_right)

        # Find vector for center view of camera
        raw_pixel_center = (cam_model.cx(), cam_model.cy())
        rect_pixel_center = cam_model.rectifyPoint(raw_pixel_center)
        center_ray = cam_model.projectPixelTo3dRay(rect_pixel_center)

        #Find the range of angles to be covered by the point cloud
        self.angle_max = angle_between_rays(
            left_ray, center_ray
        )
        self.angle_min = -angle_between_rays(
            center_ray, right_ray
        )

        self.ranges_size = camera_info.width
        self.frame_id = camera_info.header.frame_id
        self.angle_increment = (self.angle_max - self.angle_min) / (
            self.ranges_size - 1
        )
        self.get_logger().info('frame id: {}'.format(self.frame_id))
        self.get_logger().info('angle_max: {}'.format(self.angle_max))
        self.get_logger().info('angle_min: {}'.format(self.angle_min))
        self.get_logger().info('angle_increment: {}'.format(self.angle_increment))



    def on_pc_update(self, pc):
        self.point_cloud = pc
        self.get_logger().info('I received pc data')
        self.point_cloud_to_laser_scan()
    
    def point_cloud_to_laser_scan(self):
        """Convert a PointCloud to a LaserScan.
        Given a PointCloud2 message representing the area in front of the
        robot, this method calculates the slope and the gap in distance (hike)
        between consecutive points in a given direction. These slope and hike
        values are compared to thresholds to determine the distance to the
        closest obstacle in each direction. Slope is used to detect
        above-ground (positive) obstacles and hike is used to detect holes
        (negative obstacles).
        """
        #self.get_logger().info('Range size: {}'.format(self.ranges_size))
        # Initial LaserScans assume infinite travel in every direction
        hike_ranges = [float("nan")] * self.ranges_size
        slope_ranges = [float("nan")] * self.ranges_size
        min_ranges = [float("nan")] * self.ranges_size

        # Populate the point cloud
        pc = self.get_points()

        # Perform obstacle detection if there are points in the pc
        if pc is not None:
            # Arrays for values of points in each direction
            forward = pc[:, self.XYZ["FORWARD"]]
            right = pc[:, self.XYZ["RIGHT"]]
            down = pc[:, self.XYZ["DOWN"]]
            steps, dists = self.to_laser_scan_data(forward, right)

            # Create matrix where steps, dists, and down arrays are the columns
            directions = np.column_stack((steps, dists, down))
            # Sort rows by first column (steps)
            directions = directions[directions[:, 0].argsort()]
            # Group rows by step
            directions = np.split(
                directions,
                np.unique(directions[:, 0], return_index=True)[1][1:],
                axis=0,
            )

            # Loop through the rows for each step and find obstacles
            for direction in directions:
                # Sort rows for this step by the second column (dist)
                direction = direction[direction[:, 1].argsort()]

                # Step is first column of any row
                step = int(direction[0, 0])
                #self.get_logger().info("My step is {}".format(step))

                # Slice the down and dist arrays to do vectorized operations
                # at idx and idx-1
                down1 = direction[:-1, 2]
                down2 = direction[1:, 2]
                dist1 = direction[:-1, 1]
                dist2 = direction[1:, 1]

                # Calculate slope for each pair of points
                drop = np.subtract(down2, down1)
                hike = np.subtract(dist2, dist1)
                slope = np.abs(np.divide(drop, hike))

                # Find first index of row where the value crosses thresholds
                cond_hike = hike > self.min_hole_diameter
                cond_slope = slope > self.max_slope
                index_hike = cond_hike.argmax() if cond_hike.any() else None
                index_slope = cond_slope.argmax() if cond_slope.any() else None

                # Populate laserscan with closest point detected
                if (
                    index_hike is not None
                    and direction[index_hike, 1] <= self.max_obstacle_dist
                ):
                    hike_ranges[step] = direction[index_hike, 1]
                if index_slope is not None:
                    slope_ranges[step] = direction[index_slope, 1]

                # Combine above laserscans
                if step < self.ranges_size:
                    min_ranges[step] = np.nanmin((hike_ranges[step], slope_ranges[step]))

            self.hike_pub.publish(self.create_laser_scan(hike_ranges))
            self.slope_pub.publish(self.create_laser_scan(slope_ranges))
            self.combined_pub.publish(self.create_laser_scan(min_ranges)) 

    def to_laser_scan_data(self, forward, right):
        # Multiply angles by -1 to get counterclockwise (right to left) ordering
        angles = np.negative(np.arctan2(right, forward))
        # Group angles to discrete indices in laserscan array
        steps = np.divide(
            np.subtract(angles, self.angle_min), self.angle_increment
        ).astype(int)
        # Find the distance each forward, right coordinate from the robot
        dists = np.sqrt(np.add(np.square(forward), np.square(right)))
        return steps, dists
    
    def get_points(self):
        if self.point_cloud is None:
            return None

        # Read points directly from point cloud message
        points = np.frombuffer(self.point_cloud.data, dtype=np.float32)
        # Reshape into array of xyz values
        points = np.reshape(points, (-1, 8))[:, :3]
        # Remove nan points
        points = points[~np.isnan(points).any(axis=1)]       
        return points if points.size > 0 else None
    
    def create_laser_scan(self, ranges):
        """Return a LaserScan based on the given ranges list."""
        scan = LaserScan()
        scan.header.frame_id = self.frame_id
        # Get the current time stamp for the LaserScan message
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges
        scan.intensities = []
        return scan
       
        
def main(args=None):
    rclpy.init(args=args)
    max_angle=45.0
    max_obstacle_dist=4.0
    min_hole_diameter=3.0
    scan_time=1.0 / 30
    range_min=0.105
    range_max=10.0

    detector = ObstacleDetector(
        max_angle,
        max_obstacle_dist,
        min_hole_diameter,
        scan_time,
        range_min,
        range_max,
    )

    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()