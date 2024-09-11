"""

██████  ██    ██ ██████  ██   ██.
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: visualization_node.py
* auth: Alfred Xue
* desc: visualization node main file
"""

from foxglove_msgs.msg import ImageMarkerArray
import rclpy
from foxglove_msgs.msg import ImageAnnotations
from rclpy.node import Node
from foxglove_msgs.msg import Color
from foxglove_msgs.msg import TextAnnotation
from foxglove_msgs.msg import Point2
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import ImageMarker
from utfr_msgs.msg import PerceptionDebug
from utfr_msgs.msg import ConeDetections
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class VisualizationNode(Node):

    def __init__(self):
        super().__init__("visualization_node")
        self.loadParams()
        self.initVariables()
        self.initSubscribers()
        self.initPublishers()
        self.initServices()
        self.initTimers()

    def loadParams(self):
        pass

    def initVariables(self):
        pass

    def initSubscribers(self):
        """
        Subscribe to.

        perception debug
        lidar point cloud
        """
        self.perception_debug_subscriber_left_ = self.create_subscription(
            PerceptionDebug, "/perception/debug_left", self.perceptionDebugLeftCB, 1
        )
        self.perception_debug_subscriber_right_ = self.create_subscription(
            PerceptionDebug, "/perception/debug_right", self.perceptionDebugRightCB, 1
        )

        self.lidar_projection_subscriber_left_ = self.create_subscription(
            PerceptionDebug,
            "/perception/lidar_projection_publisher_left_",
            self.lidarProjectionLeftCB,
            1,
        )

        self.lidar_projection_subscriber_left_ = self.create_subscription(
            PerceptionDebug,
            "/perception/lidar_projection_publisher_right_",
            self.lidarProjectionRightCB,
            1,
        )

        self.cone_detections_ = self.create_subscription(
            ConeDetections,
            "/perception/cone_detections",
            self.perceptionConeDetectionsCB,
            1,
        )

        print(self.perception_debug_subscriber_left_)
        print(self.perception_debug_subscriber_right_)

    def initPublishers(self):
        self.left_image_marker_publisher_ = self.create_publisher(
            ImageMarkerArray, "/visualization/left_image_markers", 1
        )

        self.left_image_text_publisher_ = self.create_publisher(
            ImageAnnotations, "/visualization/left_image_text", 1
        )

        self.right_image_marker_publisher_ = self.create_publisher(
            ImageMarkerArray, "/visualization/right_image_markers", 1
        )

        self.right_image_text_publisher_ = self.create_publisher(
            ImageAnnotations, "/visualization/right_image_text", 1
        )

        self.lidar_projection_publisher_left_ = self.create_publisher(
            ImageMarkerArray, "/visualization/lidar_projection_left", 1
        )

        self.lidar_projection_publisher_right_ = self.create_publisher(
            ImageMarkerArray, "/visualization/lidar_projection_right", 1
        )

        self.cone_markers_publisher_ = self.create_publisher(
            MarkerArray, "/visualization/cone_markers", 1
        )

        self.cone_debug_markers_publisher = self.create_publisher(
            MarkerArray, "/visualization/cone_debug_markers", 1
        )

        print(self.left_image_marker_publisher_)

    def initServices(self):
        pass

    def initTimers(self):
        pass

    # colors for the different types of cones
    colorRGBALUT = [
        ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
        ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
        ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0),
        ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),
        ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),
    ]
    colorLUT = [
        Color(r=0.0, g=1.0, b=1.0, a=1.0),
        Color(r=0.0, g=0.0, b=1.0, a=1.0),
        Color(r=1.0, g=0.3, b=0.0, a=1.0),
        Color(r=1.0, g=0.0, b=1.0, a=1.0),
        Color(r=0.0, g=0.0, b=0.0, a=1.0),
    ]
    textLUT = [
        "Unknown Cone",
        "Blue Cones",
        "Yellow Cone",
        "Small Orange Cone",
        "Large Orange Cone",
    ]

    def boundaryBoxPoints(self, bounding_box):
        """Returns verticies of bounding boxes as points, given a bounding_box object"""
        return [
            Point(
                x=float(bounding_box.x),
                y=float(bounding_box.y),
                z=0.0,
            ),
            Point(
                x=float(bounding_box.x + bounding_box.width),
                y=float(bounding_box.y),
                z=0.0,
            ),
            Point(
                x=float(bounding_box.x + bounding_box.width),
                y=float(bounding_box.y + bounding_box.height),
                z=0.0,
            ),
            Point(
                x=float(bounding_box.x),
                y=float(bounding_box.y + bounding_box.height),
                z=0.0,
            ),
        ]

    def lidarProjectionLeftCB(self, msg):
        self.get_logger().warn("Recieved lidar projection left msg")
        markers = ImageMarkerArray()
        left_projections = msg.left

        for point in left_projections:
            markers.markers.append(
                ImageMarker(
                    header=msg.header,
                    scale=2.5,
                    type=ImageMarker.POLYGON,
                    outline_color=ColorRGBA(r=1.0, g=1.0, b=0.0),
                    points=[Point(x=float(point.x), y=float(point.y), z=0.0)],
                )
            )

        # publish image marker array
        self.lidar_projection_publisher_left_.publish(markers)

    def lidarProjectionRightCB(self, msg):
        self.get_logger().warn("Recieved lidar projection right msg")
        markers = ImageMarkerArray()
        right_projections = msg.right

        for point in right_projections:
            markers.markers.append(
                ImageMarker(
                    header=msg.header,
                    scale=2.5,
                    type=ImageMarker.POLYGON,
                    outline_color=ColorRGBA(r=1.0, g=1.0, b=0.0),
                    points=[Point(x=float(point.x), y=float(point.y), z=0.0)],
                )
            )

        self.lidar_projection_publisher_right_.publish(markers)

    def perceptionDebugLeftCB(self, msg):
        self.get_logger().warn("Recieved left perception debug msg")
        left_markers = ImageMarkerArray()
        left_image_annotation = ImageAnnotations()
        print(msg.header.stamp)

        left_detections = msg.left
        for left_bounding_box in left_detections:
            if (
                left_bounding_box.type < len(self.colorLUT)
                and left_bounding_box.type > 0
            ):
                left_markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=self.colorRGBALUT[left_bounding_box.type],
                        points=self.boundaryBoxPoints(left_bounding_box),
                    )
                )

                # add text annotation
                left_image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(left_bounding_box.x),
                            y=float(left_bounding_box.y - 10),
                        ),
                        text=self.textLUT[left_bounding_box.type]
                        + " "
                        + str(left_bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=self.colorLUT[left_bounding_box.type],
                    )
                )
            else:
                continue

        # publish image marker array
        self.left_image_marker_publisher_.publish(left_markers)
        self.left_image_text_publisher_.publish(left_image_annotation)

    def perceptionDebugRightCB(self, msg):
        self.get_logger().warn("Recieved right perception debug msg")
        right_markers = ImageMarkerArray()
        right_image_annotation = ImageAnnotations()
        right_detections = msg.right

        for right_bounding_box in right_detections:
            if (
                right_bounding_box.type < len(self.colorLUT)
                and right_bounding_box.type > 0
            ):
                right_markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=self.colorRGBALUT[right_bounding_box.type],
                        points=self.boundaryBoxPoints(right_bounding_box),
                    )
                )

                # add text annotation
                right_image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(right_bounding_box.x),
                            y=float(right_bounding_box.y - 10),
                        ),
                        text=self.textLUT[right_bounding_box.type]
                        + " "
                        + str(right_bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=self.colorLUT[right_bounding_box.type],
                    )
                )
            else:
                continue

        # publish image marker array
        self.right_image_marker_publisher_.publish(right_markers)
        self.right_image_text_publisher_.publish(right_image_annotation)

    # (r,g,b,a)
    cubeColorLUT = {
        "left_cone": (1.0, 1.0, 0.0, 1.0),
        "right_cone": (0.0, 1.0, 1.0, 1.0),
        "large_orange_cone": (1.0, 0.5, 0.0, 1.0),
        "small_orange_cone": (1.0, 0.5, 0.0, 1.0),
        "unknown_cone": (1.0, 1.0, 1.0, 1.0),
    }

    def cubeMarkerFromCone(self, cone, type_cone, header, frame, id):
        cube_marker = Marker()
        # populate the marker
        cube_marker.header = header
        cube_marker.header.frame_id = frame
        cube_marker.ns = "utfr_foxglove"
        cube_marker.id = id
        cube_marker.type = 1  # cube
        cube_marker.action = 0  # add
        cube_marker.pose.position.x = cone.pos.x
        cube_marker.pose.position.y = cone.pos.y
        cube_marker.pose.position.z = cone.pos.z
        cube_marker.pose.orientation.x = 0.0
        cube_marker.pose.orientation.y = 0.0
        cube_marker.pose.orientation.z = 0.0
        cube_marker.pose.orientation.w = 1.0
        cube_marker.scale.x = 0.2
        cube_marker.scale.y = 0.2
        cube_marker.scale.z = 0.2

        color = self.cubeColorLUT[type_cone]
        cube_marker.color.r = color[0]
        cube_marker.color.g = color[1]
        cube_marker.color.b = color[2]
        cube_marker.color.a = color[3]

        return cube_marker

    def perceptionConeDetectionsCB(self, msg):
        """
        Publish the cone detections as yellow cube markers
        """
        self.get_logger().warn("Recieved cone detections msg")
        cube_cone_dets = MarkerArray()
        left_cones = msg.left_cones
        right_cones = msg.right_cones
        large_orange_cones = msg.large_orange_cones
        small_orange_cones = msg.small_orange_cones
        unknown_cones = msg.unknown_cones
        i = 0
        for cone in left_cones:
            cube_cone_dets.markers.append(
                self.cubeMarkerFromCone(cone, "left_cone", msg.header, "ground", i)
            )
            i += 1

        for cone in right_cones:
            cube_cone_dets.markers.append(
                self.cubeMarkerFromCone(cone, "right_cone", msg.header, "ground", i)
            )
            i += 1

        for cone in large_orange_cones:
            cube_cone_dets.markers.append(
                self.cubeMarkerFromCone(
                    cone, "large_orange_cone", msg.header, "ground", i
                )
            )
            i += 1

        for cone in small_orange_cones:
            cube_cone_dets.markers.append(
                self.cubeMarkerFromCone(
                    cone, "small_orange_cone", msg.header, "ground", i
                )
            )
            i += 1

        for cone in unknown_cones:
            cube_cone_dets.markers.append(
                self.cubeMarkerFromCone(cone, "unknown_cone", msg.header, "ground", i)
            )
            i += 1
        self.cone_markers_publisher_.publish(cube_cone_dets)


def main(args=None):
    print("Hi from visualization.")
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
