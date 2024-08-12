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

        self.cone_markers_publisher_ = self.create_publisher(
            MarkerArray, "/visualization/cone_markers", 1
        )

        print(self.left_image_marker_publisher_)

    def initServices(self):
        pass

    def initTimers(self):
        pass

    def perceptionDebugLeftCB(self, msg):
        self.get_logger().warn("Recieved left perception debug msg")
        markers = ImageMarkerArray()
        image_annotation = ImageAnnotations()
        print(msg.header.stamp)

        left_detections = msg.left
        for bounding_box in left_detections:
            # initialize image marker array
            if bounding_box.type == 0:  # unknown cone
                markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
                        points=[
                            Point(
                                x=float(bounding_box.x), y=float(bounding_box.y), z=0.0
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
                        ],
                    )
                )

                # add text annotation

                image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(bounding_box.x),
                            y=float(bounding_box.y - 10),
                        ),
                        text="Unknown Cone " + str(bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=Color(r=0.0, g=1.0, b=1.0, a=1.0),
                    )
                )

            elif bounding_box.type == 1:  # blue cone
                markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
                        points=[
                            Point(
                                x=float(bounding_box.x), y=float(bounding_box.y), z=0.0
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
                        ],
                    )
                )

                image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(bounding_box.x),
                            y=float(bounding_box.y - 10),
                        ),
                        text="Blue Cone " + str(bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=Color(r=0.0, g=0.0, b=1.0, a=1.0),
                    )
                )

            elif bounding_box.type == 2:  # yellow cone
                markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0),
                        points=[
                            Point(
                                x=float(bounding_box.x), y=float(bounding_box.y), z=0.0
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
                        ],
                    )
                )

                image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(bounding_box.x),
                            y=float(bounding_box.y - 20),
                        ),
                        text="Yellow Cone " + str(bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=Color(r=1.0, g=0.3, b=0.0, a=1.0),
                    )
                )

            elif bounding_box.type == 3:  # small orange cone
                markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),
                        points=[
                            Point(
                                x=float(bounding_box.x), y=float(bounding_box.y), z=0.0
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
                        ],
                    )
                )
                image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(bounding_box.x),
                            y=float(bounding_box.y - 20),
                        ),
                        text="Small Orange Cone " + str(bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=Color(r=1.0, g=0.0, b=1.0, a=1.0),
                    )
                )

            elif bounding_box.type == 4:  # large orange cone
                markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),
                        points=[
                            Point(
                                x=float(bounding_box.x), y=float(bounding_box.y), z=0.0
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
                        ],
                    )
                )
                image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(bounding_box.x),
                            y=float(bounding_box.y - 20),
                        ),
                        text="Large Orange Cone " + str(bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=Color(r=0.0, g=0.0, b=0.0, a=1.0),
                    )
                )

            else:
                continue
        # initialize text

        # done for loop, publish image marker array
        self.left_image_marker_publisher_.publish(markers)

        self.left_image_text_publisher_.publish(image_annotation)

    def perceptionDebugRightCB(self, msg):
        self.get_logger().warn("Recieved right perception debug msg")
        right_markers = ImageMarkerArray()
        right_image_annotation = ImageAnnotations()

        right_detections = msg.right

        for right_bounding_box in right_detections:
            # initialize image marker array
            if right_bounding_box.type == 0:  # unknown cone
                right_markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0),
                        points=[
                            Point(
                                x=float(right_bounding_box.x),
                                y=float(right_bounding_box.y),
                                z=0.0,
                            ),
                            Point(
                                x=float(
                                    right_bounding_box.x + right_bounding_box.width
                                ),
                                y=float(right_bounding_box.y),
                                z=0.0,
                            ),
                            Point(
                                x=float(
                                    right_bounding_box.x + right_bounding_box.width
                                ),
                                y=float(
                                    right_bounding_box.y + right_bounding_box.height
                                ),
                                z=0.0,
                            ),
                            Point(
                                x=float(right_bounding_box.x),
                                y=float(
                                    right_bounding_box.y + right_bounding_box.height
                                ),
                                z=0.0,
                            ),
                        ],
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
                        text="Unknown Cone " + str(right_bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=Color(r=0.0, g=1.0, b=1.0, a=1.0),
                    )
                )

            elif right_bounding_box.type == 1:  # blue cone
                right_markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),
                        points=[
                            Point(
                                x=float(right_bounding_box.x),
                                y=float(right_bounding_box.y),
                                z=0.0,
                            ),
                            Point(
                                x=float(
                                    right_bounding_box.x + right_bounding_box.width
                                ),
                                y=float(right_bounding_box.y),
                                z=0.0,
                            ),
                            Point(
                                x=float(
                                    right_bounding_box.x + right_bounding_box.width
                                ),
                                y=float(
                                    right_bounding_box.y + right_bounding_box.height
                                ),
                                z=0.0,
                            ),
                            Point(
                                x=float(right_bounding_box.x),
                                y=float(
                                    right_bounding_box.y + right_bounding_box.height
                                ),
                                z=0.0,
                            ),
                        ],
                    )
                )

                right_image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(right_bounding_box.x),
                            y=float(right_bounding_box.y - 10),
                        ),
                        text="Blue Cone " + str(right_bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=Color(r=0.0, g=0.0, b=1.0, a=1.0),
                    )
                )

            elif right_bounding_box.type == 2:  # yellow cone
                right_markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=ColorRGBA(r=1.0, g=0.3, b=0.0, a=1.0),
                        points=[
                            Point(
                                x=float(right_bounding_box.x),
                                y=float(right_bounding_box.y),
                                z=0.0,
                            ),
                            Point(
                                x=float(
                                    right_bounding_box.x + right_bounding_box.width
                                ),
                                y=float(right_bounding_box.y),
                                z=0.0,
                            ),
                            Point(
                                x=float(
                                    right_bounding_box.x + right_bounding_box.width
                                ),
                                y=float(
                                    right_bounding_box.y + right_bounding_box.height
                                ),
                                z=0.0,
                            ),
                            Point(
                                x=float(right_bounding_box.x),
                                y=float(
                                    right_bounding_box.y + right_bounding_box.height
                                ),
                                z=0.0,
                            ),
                        ],
                    )
                )

                right_image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(right_bounding_box.x),
                            y=float(right_bounding_box.y - 20),
                        ),
                        text="Yellow Cone " + str(right_bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=Color(r=1.0, g=0.3, b=0.0, a=1.0),
                    )
                )

            elif right_bounding_box.type == 3:  # small orange cone
                right_markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0),
                        points=[
                            Point(
                                x=float(right_bounding_box.x),
                                y=float(right_bounding_box.y),
                                z=0.0,
                            ),
                            Point(
                                x=float(
                                    right_bounding_box.x + right_bounding_box.width
                                ),
                                y=float(right_bounding_box.y),
                                z=0.0,
                            ),
                            Point(
                                x=float(
                                    right_bounding_box.x + right_bounding_box.width
                                ),
                                y=float(
                                    right_bounding_box.y + right_bounding_box.height
                                ),
                                z=0.0,
                            ),
                            Point(
                                x=float(right_bounding_box.x),
                                y=float(
                                    right_bounding_box.y + right_bounding_box.height
                                ),
                                z=0.0,
                            ),
                        ],
                    )
                )
                right_image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(right_bounding_box.x),
                            y=float(right_bounding_box.y - 20),
                        ),
                        text="Small Orange Cone " + str(right_bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=Color(r=1.0, g=0.0, b=1.0, a=1.0),
                    )
                )

            elif right_bounding_box.type == 4:  # large orange cone
                right_markers.markers.append(
                    ImageMarker(
                        header=msg.header,
                        scale=2.5,
                        type=ImageMarker.POLYGON,
                        outline_color=ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),
                        points=[
                            Point(
                                x=float(right_bounding_box.x),
                                y=float(right_bounding_box.y),
                                z=0.0,
                            ),
                            Point(
                                x=float(
                                    right_bounding_box.x + right_bounding_box.width
                                ),
                                y=float(right_bounding_box.y),
                                z=0.0,
                            ),
                            Point(
                                x=float(
                                    right_bounding_box.x + right_bounding_box.width
                                ),
                                y=float(
                                    right_bounding_box.y + right_bounding_box.height
                                ),
                                z=0.0,
                            ),
                            Point(
                                x=float(right_bounding_box.x),
                                y=float(
                                    right_bounding_box.y + right_bounding_box.height
                                ),
                                z=0.0,
                            ),
                        ],
                    )
                )
                right_image_annotation.texts.append(
                    TextAnnotation(
                        timestamp=msg.header.stamp,
                        position=Point2(
                            x=float(right_bounding_box.x),
                            y=float(right_bounding_box.y - 20),
                        ),
                        text="Large Orange Cone " + str(right_bounding_box.score),
                        font_size=20.0,
                        text_color=Color(r=1.0, g=1.0, b=1.0, a=1.0),
                        background_color=Color(r=0.0, g=0.0, b=0.0, a=1.0),
                    )
                )

            else:
                continue

        # done for loop, publish image marker array
        self.right_image_marker_publisher_.publish(right_markers)

        self.right_image_text_publisher_.publish(right_image_annotation)

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
            cube_marker = Marker()
            # populate the marker
            cube_marker.header = msg.header
            cube_marker.header.frame_id = "os_lidar"
            cube_marker.ns = "utfr_foxglove"
            cube_marker.id = i
            cube_marker.type = 1  # cube
            cube_marker.action = 0  # add
            cube_marker.pose.position = cone.pos
            cube_marker.pose.orientation.x = 0.0
            cube_marker.pose.orientation.y = 0.0
            cube_marker.pose.orientation.z = 0.0
            cube_marker.pose.orientation.w = 1.0
            cube_marker.scale.x = 0.2
            cube_marker.scale.y = 0.2
            cube_marker.scale.z = 0.2
            cube_marker.color.a = 1.0
            cube_marker.color.r = 1.0
            cube_marker.color.g = 1.0
            cube_marker.color.b = 0.0

            cube_cone_dets.markers.append(cube_marker)
            i += 1

        for cone in right_cones:
            cube_marker = Marker()
            # populate the marker
            cube_marker.header = msg.header
            cube_marker.header.frame_id = "os_lidar"
            cube_marker.ns = "utfr_foxglove"
            cube_marker.id = i
            cube_marker.type = 1  # cube
            cube_marker.action = 0  # add
            cube_marker.pose.position = cone.pos
            cube_marker.pose.orientation.x = 0.0
            cube_marker.pose.orientation.y = 0.0
            cube_marker.pose.orientation.z = 0.0
            cube_marker.pose.orientation.w = 1.0
            cube_marker.scale.x = 0.2
            cube_marker.scale.y = 0.2
            cube_marker.scale.z = 0.2
            cube_marker.color.a = 1.0
            cube_marker.color.r = 1.0
            cube_marker.color.g = 1.0
            cube_marker.color.b = 0.0

            cube_cone_dets.markers.append(cube_marker)
            i += 1

        for cone in large_orange_cones:
            cube_marker = Marker()
            # populate the marker
            cube_marker.header = msg.header
            cube_marker.header.frame_id = "os_lidar"
            cube_marker.ns = "utfr_foxglove"
            cube_marker.id = i
            cube_marker.type = 1  # cube
            cube_marker.action = 0  # add
            cube_marker.pose.position = cone.pos
            cube_marker.pose.orientation.x = 0.0
            cube_marker.pose.orientation.y = 0.0
            cube_marker.pose.orientation.z = 0.0
            cube_marker.pose.orientation.w = 1.0
            cube_marker.scale.x = 0.2
            cube_marker.scale.y = 0.2
            cube_marker.scale.z = 0.2
            cube_marker.color.a = 1.0
            cube_marker.color.r = 1.0
            cube_marker.color.g = 1.0
            cube_marker.color.b = 0.0

            cube_cone_dets.markers.append(cube_marker)
            i += 1

        for cone in small_orange_cones:
            cube_marker = Marker()
            # populate the marker
            cube_marker.header = msg.header
            cube_marker.header.frame_id = "os_lidar"
            cube_marker.ns = "utfr_foxglove"
            cube_marker.id = i
            cube_marker.type = 1  # cube
            cube_marker.action = 0  # add
            cube_marker.pose.position = cone.pos
            cube_marker.pose.orientation.x = 0.0
            cube_marker.pose.orientation.y = 0.0
            cube_marker.pose.orientation.z = 0.0
            cube_marker.pose.orientation.w = 1.0
            cube_marker.scale.x = 0.2
            cube_marker.scale.y = 0.2
            cube_marker.scale.z = 0.2
            cube_marker.color.a = 1.0
            cube_marker.color.r = 1.0
            cube_marker.color.g = 1.0
            cube_marker.color.b = 0.0

            cube_cone_dets.markers.append(cube_marker)
            i += 1

        for cone in unknown_cones:
            cube_marker = Marker()
            # populate the marker
            cube_marker.header = msg.header
            cube_marker.header.frame_id = "os_lidar"
            cube_marker.ns = "utfr_foxglove"
            cube_marker.id = i
            cube_marker.type = 1  # cube
            cube_marker.action = 0  # add
            cube_marker.pose.position = cone.pos
            cube_marker.pose.orientation.x = 0.0
            cube_marker.pose.orientation.y = 0.0
            cube_marker.pose.orientation.z = 0.0
            cube_marker.pose.orientation.w = 1.0
            cube_marker.scale.x = 0.2
            cube_marker.scale.y = 0.2
            cube_marker.scale.z = 0.2
            cube_marker.color.a = 1.0
            cube_marker.color.r = 1.0
            cube_marker.color.g = 1.0
            cube_marker.color.b = 0.0

            cube_cone_dets.markers.append(cube_marker)
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
