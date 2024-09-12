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
from utfr_msgs.msg import ConeMap
from utfr_msgs.msg import EgoState
from utfr_msgs.msg import ParametricSpline
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
import tf2_ros


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
        self.delete_all_marker = Marker()
        self.delete_all_marker.action = 3 # delete all
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_br = tf2_ros.TransformBroadcaster(self)

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
        self.cone_map_ = self.create_subscription(
            ConeMap,
            "/mapping/cone_map",
            self.mappingConeMapCB,
            1
        )
        self.ego_state_ = self.create_subscription(
            EgoState,
            "/ekf/ego_state",
            self.ekfEgoStateCB,
            1
        )
        self.controller_path_ = self.create_subscription(
            ParametricSpline,
            "/planning/center_path",
            self.planningControllerPathCB,
            1
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

        self.cone_map_publisher_ = self.create_publisher(
            MarkerArray, "/visualization/cone_map", 1
        )

        self.ego_state_publisher_ = self.create_publisher(
            Marker, "/visualization/ego_state", 1
        )

        self.controller_path_publisher_ = self.create_publisher(
            Path, "/visualization/controller_path", 1
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
        try:
            self.get_logger().warn("Recieved cone detections msg")
            print(msg.header.frame_id)
            cone_markers = MarkerArray()
            self.delete_all_marker.header = msg.header
            cone_markers.markers.append(self.delete_all_marker)

            left_cones = msg.left_cones
            right_cones = msg.right_cones
            large_orange_cones = msg.large_orange_cones
            small_orange_cones = msg.small_orange_cones
            unknown_cones = msg.unknown_cones

            def addCones(cones, r, g, b, scale=0.2):
                for cone in cones:
                    cone_marker = Marker()
                    cone_marker.header = msg.header
                    cone_marker.header.frame_id = msg.header.frame_id
                    cone_marker.ns = "utfr_foxglove"
                    cone_marker.id = len(cone_markers.markers)
                    cone_marker.type = 1  # cube
                    cone_marker.action = 0  # add
                    # pose = Pose()
                    # pose.position = cone.pos
                    # pose.orientation.x = 0.0
                    # pose.orientation.y = 0.0
                    # pose.orientation.z = 0.0
                    # pose.orientation.w = 1.0
                    # transform = self.tf_buffer.lookup_transform('ground', msg.header.frame_id, rclpy.time.Time())
                    # cone_marker.pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                    cone_marker.pose.position = cone.pos
                    cone_marker.pose.position.z = scale/2.0
                    cone_marker.pose.orientation.x = 0.0
                    cone_marker.pose.orientation.y = 0.0
                    cone_marker.pose.orientation.z = 0.0
                    cone_marker.pose.orientation.w = 1.0
                    cone_marker.scale.x = scale
                    cone_marker.scale.y = scale
                    cone_marker.scale.z = scale
                    cone_marker.color.a = 1.0
                    cone_marker.color.r = r
                    cone_marker.color.g = g
                    cone_marker.color.b = b

                    cone_markers.markers.append(cone_marker)

            addCones(left_cones, 0.0, 0.0, 1.0)
            addCones(right_cones, 1.0, 1.0, 0.0)
            addCones(small_orange_cones, 1.0, 0.549, 0.0, 0.1)
            addCones(large_orange_cones, 1.0, 0.549, 0.0, 0.3)
            addCones(unknown_cones, 1.0, 1.0, 1.0)

            self.cone_markers_publisher_.publish(cone_markers)
        except Exception as e:
            print(f"An error occurred: {e}") 

    def mappingConeMapCB(self, msg):
        """
        Publish the cone map as cube markers
        """
        try:
            self.get_logger().warn("Recieved cone maps msg")
            print(msg.header.frame_id)
            cone_markers = MarkerArray()
            self.delete_all_marker.header = msg.header
            cone_markers.markers.append(self.delete_all_marker)

            left_cones = msg.left_cones
            right_cones = msg.right_cones
            large_orange_cones = msg.large_orange_cones
            small_orange_cones = msg.small_orange_cones
            unknown_cones = msg.unknown_cones
            
            def addCones(cones, r, g, b, scale=0.2):
                for cone in cones:
                    cone_marker = Marker()
                    cone_marker.header = msg.header
                    cone_marker.header.frame_id = "map"
                    cone_marker.ns = "utfr_foxglove"
                    cone_marker.id = len(cone_markers.markers)
                    cone_marker.type = 1  # cube
                    cone_marker.action = 0  # add
                    pose = Pose()
                    pose.position = cone.pos
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = 0.0
                    pose.orientation.w = 1.0
                    # transform = self.tf_buffer.lookup_transform('ground', msg.header.frame_id, rclpy.time.Time())
                    # cone_marker.pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
                    # cone_marker.pose.position.z = scale/2.0
                    cone_marker.pose = pose
                    cone_marker.pose.position.z = scale/2.0
                    cone_marker.scale.x = scale
                    cone_marker.scale.y = scale
                    cone_marker.scale.z = scale
                    cone_marker.color.a = 1.0
                    cone_marker.color.r = r
                    cone_marker.color.g = g
                    cone_marker.color.b = b

                    cone_markers.markers.append(cone_marker)

            addCones(left_cones, 0.0, 0.0, 1.0)
            addCones(right_cones, 1.0, 1.0, 0.0)
            addCones(small_orange_cones, 1.0, 0.549, 0.0, 0.1)
            addCones(large_orange_cones, 1.0, 0.549, 0.0, 0.3)
            addCones(unknown_cones, 1.0, 1.0, 1.0)

            self.cone_map_publisher_.publish(cone_markers)
        except Exception as e:
            print(f"An error occurred: {e}")

    def ekfEgoStateCB(self, msg):
        """
        Publish the ego state as a marker
        """
        self.get_logger().warn("Recieved ego state msg")
        if msg.header.frame_id == '':
            msg.header.frame_id = 'map'
        print(msg.header.frame_id)
        
        # Transform begin
        # transform = TransformStamped()

        # transform.header.stamp = self.get_clock().now().to_msg()
        # transform.header.frame_id = 'map'
        # transform.child_frame_id = 'imu_link'

        # transform.transform.translation.x = msg.pose.pose.position.x
        # transform.transform.translation.y = msg.pose.pose.position.y
        # transform.transform.translation.z = 0.265

        # transform.transform.rotation = msg.pose.pose.orientation

        # self.tf_br.sendTransform(transform)
        # Transform end

        car_marker = Marker()
        car_marker.header = msg.header
        car_marker.header.frame_id = "ground"
        car_marker.ns = "utfr_foxglove"
        car_marker.id = 0
        car_marker.type = 1  # cube
        car_marker.action = 0  # add
        # pose = Pose()
        # pose.position = msg.pose.pose.position
        # pose.orientation = msg.pose.pose.orientation
        # transform = self.tf_buffer.lookup_transform('ground', msg.header.frame_id, rclpy.time.Time())
        # car_marker.pose = tf2_geometry_msgs.do_transform_pose(pose, transform)
        # car_marker.pose.position.x += 0.26885
        # car_marker.pose.position.z += 1.175/2.0
        car_marker.pose.position.x = 0.26885
        car_marker.pose.position.y = 0.0
        car_marker.pose.position.z = 1.175/2.0
        car_marker.pose.orientation.x = 0.0
        car_marker.pose.orientation.y = 0.0
        car_marker.pose.orientation.z = 0.0
        car_marker.pose.orientation.w = 1.0
        car_marker.scale.x = 2.862
        car_marker.scale.y = 1.37
        car_marker.scale.z = 1.175
        car_marker.color.a = 1.0
        car_marker.color.r = 0.0
        car_marker.color.g = 0.0
        car_marker.color.b = 1.0

        self.ego_state_publisher_.publish(car_marker)

    def planningControllerPathCB(self, msg):
        try:
            self.get_logger().warn("Recieved controller path msg")
            if msg.header.frame_id == '':
               msg.header.frame_id = 'base_footprint'
            print(msg.header.frame_id)
            path_msg = Path()

            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = "ground"

            # print("({},{})".format(self.ego.pose.pose.position.x, self.ego.pose.pose.position.y))
            print(*msg.x_params, sep=',')
            print(*msg.y_params, sep=',')
            n = 150
            s = 0.0
            ds = 0.025
            for i in range(n):
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = "ground"

                pose_stamped.pose.position.x = 0.0
                pose_stamped.pose.position.y = 0.0
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.x = 0.0
                pose_stamped.pose.orientation.y = 0.0
                pose_stamped.pose.orientation.z = 0.0
                pose_stamped.pose.orientation.w = 1.0


                s += ds
                for j in range(6):
                    pose_stamped.pose.position.x += msg.x_params[j] * pow(s, 5-j)
                    pose_stamped.pose.position.y -= msg.y_params[j] * pow(s, 5-j)
                
                transform = self.tf_buffer.lookup_transform('ground', msg.header.frame_id, rclpy.time.Time())
                pose_stamped.pose = tf2_geometry_msgs.do_transform_pose(pose_stamped.pose, transform)
                # print("({},{})".format(pose_stamped.pose.position.x, pose_stamped.pose.position.y))
                path_msg.poses.append(pose_stamped)

            self.controller_path_publisher_.publish(path_msg)
        except Exception as e:
            print(f"An error occurred: {e}")

def main(args=None):
    print("Hi from visualization.")
    rclpy.init(args=args)
    node = VisualizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
