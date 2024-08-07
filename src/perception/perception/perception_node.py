#!/usr/bin/env python3
"""

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: perception_node.py
* auth: Kelvin Cui
* desc: perception node main file
"""
# System Requirements
import cv2
import numpy as np
import matplotlib.pyplot as plot

# import xgboost
import onnxruntime as ort
import time

from scipy.optimize import linear_sum_assignment


# ROS2 Requirements
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import rospkg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message Requirements
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from utfr_msgs.msg import ConeDetections
from utfr_msgs.msg import Heartbeat
from utfr_msgs.msg import Cone
from utfr_msgs.msg import BoundingBox
from utfr_msgs.msg import PerceptionDebug

# Service Requirements
from std_srvs.srv import Trigger

# Service Requirements
from std_srvs.srv import Trigger

# Import deep and classical process functions
from perception.submodules.deep import deep_process
from perception.submodules.deep import bounding_boxes_to_cone_detections
from perception.submodules.deep import check_for_cuda
from perception.submodules.deep import labelColor
from perception.submodules.deep import transform_det_lidar


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.loadParams()
        self.initVariables()
        self.initHeartbeat()
        self.initSubscribers()
        self.initPublishers()
        self.initServices()
        self.initConeTemplate()
        self.initTimers()
        # self.initClassical()

    def loadParams(self):
        """
        Initialize and load params from config.yaml:

        baseline : double:
          distance between camera centers

        left_topic : string:
          name of left image publisher topic

        cone_detections_topic : string:
          name of cone detections publisher topic

        heartbeat_topic : string:
          name of heartbeat publisher topic

        update_rate : double:
          rate of capture, processing and publishing in [ms]

        distortion : array:
          distortion coefficients for undistorting frames from camera

        instrinsics_left, intrinsics_right : array:
          intrinsic camera matrix for 3d coordinate mapping, in a single 9x1 array

        rotation : array:
          rotation matrix for rectifying left and right images, 9x1 array

        translation : array:
          translation matrix for camera calibration, 3x1 array

        rightfy_left, rectify_right : array:
          rectification maps for camera rectification, 9x1 array

        save_pic : boolean
          true to save all frames while node is running, false to not

          confidence : double:
            minimum confidence value to accept a cone detection

          perception_debug_topic : string:
            name of perception debug topic (cone bounding boxes)

        cone_heights: array:
          array of cone heights in mm [0, blue, yellow, small orange, large
          orange]


        """
        self.declare_parameter("baseline", 10.0)
        self.declare_parameter("left_camera_topic", "/left_camera_node/images")
        self.declare_parameter("right_camera_topic", "/right_camera_node/images")
        self.declare_parameter("cone_detections_topic", "/perception/cone_detections")
        self.declare_parameter("heartbeat_topic", "/perception/heartbeat")
        self.declare_parameter("processed_lidar_topic", "/lidar_pipeline/clustered")
        self.declare_parameter("update_rate", 33.33)
        self.declare_parameter("distortion_left", [0.0])
        self.declare_parameter("distortion_right", [0.0])
        self.declare_parameter("intrinsics_left", [0.0])
        self.declare_parameter("intrinsics_right", [0.0])
        self.declare_parameter("rotation", [0.0])
        self.declare_parameter("translation", [0.0])
        self.declare_parameter("rectify_left", [0.0])
        self.declare_parameter("rectify_right", [0.0])
        self.declare_parameter("save_pic", "False")
        self.declare_parameter("confidence", 0.70)
        self.declare_parameter("perception_debug_topic", "/perception/debug")
        self.declare_parameter("cone_heights", [0.0])
        self.declare_parameter("lidar_only_detection", False)

        self.baseline_ = (
            self.get_parameter("baseline").get_parameter_value().double_value
        )
        self.left_camera_topic = (
            self.get_parameter("left_camera_topic").get_parameter_value().string_value
        )
        self.right_camera_topic = (
            self.get_parameter("right_camera_topic").get_parameter_value().string_value
        )
        self.cone_detections_topic_ = (
            self.get_parameter("cone_detections_topic")
            .get_parameter_value()
            .string_value
        )
        self.heartbeat_topic_ = (
            self.get_parameter("heartbeat_topic").get_parameter_value().string_value
        )
        self.processed_lidar_topic = (
            self.get_parameter("processed_lidar_topic")
            .get_parameter_value()
            .string_value
        )
        self.update_rate_ = (
            self.get_parameter("update_rate").get_parameter_value().double_value
        )
        self.distortion_left = (
            self.get_parameter("distortion_left")
            .get_parameter_value()
            .double_array_value
        )
        self.distortion_right = (
            self.get_parameter("distortion_right")
            .get_parameter_value()
            .double_array_value
        )
        self.intrinsics_left = (
            self.get_parameter("intrinsics_left")
            .get_parameter_value()
            .double_array_value
        )
        self.intrinsics_right = (
            self.get_parameter("intrinsics_right")
            .get_parameter_value()
            .double_array_value
        )
        self.rotation = (
            self.get_parameter("rotation").get_parameter_value().double_array_value
        )
        self.translation = (
            self.get_parameter("translation").get_parameter_value().double_array_value
        )
        self.rectify_left = (
            self.get_parameter("rectify_left").get_parameter_value().double_array_value
        )
        self.rectify_right = (
            self.get_parameter("rectify_right").get_parameter_value().double_array_value
        )
        self.perception_debug_topic_ = (
            self.get_parameter("perception_debug_topic")
            .get_parameter_value()
            .string_value
        )
        self.save_pic = (
            self.get_parameter("save_pic").get_parameter_value().string_value
        )
        self.confidence_ = (
            self.get_parameter("confidence").get_parameter_value().double_value
        )

        self.cone_heights = (
            self.get_parameter("cone_heights").get_parameter_value().double_array_value
        )

        self.lidar_only_detection = (
            self.get_parameter("lidar_only_detection").get_parameter_value().bool_value
        )

        # reshape the arrays into numpy matrix form

        self.intrinsics_left = np.array(self.intrinsics_left).reshape(3, 3)
        self.intrinsics_right = np.array(self.intrinsics_right).reshape(3, 3)
        self.rotation = np.array(self.rotation).reshape(3, 3)
        self.rectify_left = np.array(self.rectify_left).reshape(3, 3)
        self.rectify_right = np.array(self.rectify_right).reshape(3, 3)
        self.distortion_left = np.array(self.distortion_left)
        self.distortion_right = np.array(self.distortion_right)
        self.translation = np.array(self.translation)
        self.cone_heights = np.array(self.cone_heights)

    def initVariables(self):
        # Initialize callback variables:
        self.left_img_recieved_ = False
        self.right_img_recieved_ = False
        self.first_img_arrived_ = False
        self.left_ready_ = False
        self.right_ready_ = False
        self.saved_count = 0
        self.lidar_msg = None

        # Work
        self.right_img_ = None
        self.left_img_ = None

        self.img_size = (1440, 1080)

        # Initialize image conversion bridge:
        self.bridge = CvBridge()

        # generate separate maps for left and right

        self.mapx_right, self.mapy_right = cv2.initUndistortRectifyMap(
            self.intrinsics_right,
            self.distortion_right,
            self.rectify_right,
            self.intrinsics_right,
            (1440, 1080),
            cv2.CV_32FC1,
        )

        self.mapx_left, self.mapy_left = cv2.initUndistortRectifyMap(
            self.intrinsics_left,
            self.distortion_left,
            self.rectify_left,
            self.intrinsics_left,
            (1440, 1080),
            cv2.CV_32FC1,
        )

        # create session for onnxruntime ofr detections
        cuda = check_for_cuda()
        print("Check for cuda:", cuda)
        # providers = ["AzureExecutionProvider"] if cuda else ["CPUExecutionProvider"]
        providers = ort.get_available_providers()
        print("Available Providers:", providers)

        # Get the current device for inference
        device = ort.get_device()
        print("Current Device for Inference:", device)
        self.session = ort.InferenceSession(
            "src/perception/perception/best.onnx",
            providers=["CUDAExecutionProvider"],
        )

        # create transform frame variables
        self.lidar_frame = "lidar"
        self.left_camera_frame = "left_camera"
        self.right_camera_frame = "right_camera"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        print(self.tf_listener)

        # TODO - add last tf for lidar to whatever the base frame is

    def initSubscribers(self):
        """
        Initialize Subscribers

        left_cam_subscriber_ :
          msg: sensor_msgs::Image, topic: kLeftImage

        right_cam_subscriber_ :
          msg: sensor_msgs::Image, topic: kRightImage

        mapping_debug_subscriber_ :
          msg: sensor_msgs::PointCloud2, topic:
        """
        self.left_cam_subscriber_ = self.create_subscription(
            Image, self.left_camera_topic, self.leftCameraCB, 1
        )

        self.right_cam_subscriber_ = self.create_subscription(
            Image, self.right_camera_topic, self.rightCameraCB, 1
        )

        self.processed_lidar_subscriber_ = self.create_subscription(
            PointCloud2, self.processed_lidar_topic, self.lidarCB, 1
        )

        # Latching Subscribers:
        qos_latching = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.left_cam_ready_subscriber_ = self.create_subscription(
            Bool, "/left_camera_node/ready", self.leftCameraReadyCB, qos_latching
        )

        self.right_cam_ready_subscriber_ = self.create_subscription(
            Bool, "/right_camera_node/ready", self.rightCameraReadyCB, qos_latching
        )

        # Call the left_cam_subscriber_ and right_cam_subscriber_ to prevent
        # unused variable warnings
        self.left_cam_subscriber_
        self.right_cam_subscriber_
        self.left_cam_ready_subscriber_
        self.right_cam_ready_subscriber_

    def initPublishers(self):
        """
        Initialize Publisher

        cone_detections_publisher_ :
          msg: utfr_msgs::ConeDetections, topic: kConeDetections

        heartbeat_publisher_:
          msg: utfr_msgs::Heartbeat, topic: kHeartbeat

        perception_degub_publisher_ :
          msg: utfr_msgs::PerceptionDebug, topic: kPerceptionDebug


        """
        self.cone_detections_publisher_ = self.create_publisher(
            ConeDetections, self.cone_detections_topic_, 1
        )

        self.cone_detections_debug_ = self.create_publisher(
            ConeDetections, "/perception/detections_debug", 1
        )

        self.heartbeat_publisher_ = self.create_publisher(
            Heartbeat, self.heartbeat_topic_, 1
        )

        self.perception_debug_publisher_left_ = self.create_publisher(
            PerceptionDebug, self.perception_debug_topic_ + "_left", 1
        )

        self.perception_debug_publisher_right_ = self.create_publisher(
            PerceptionDebug, self.perception_debug_topic_ + "_right", 1
        )

    def initServices(self):
        """
        Initialize Services

        left_camera_client_ : triggers left camera shutter
          Trigger, topic: left_camera_node/trigger_image

        right_camera_client_ : triggers right camera shutter
          Trigger, topic: right_camera_node/trigger_image
        """

        self.left_camera_client_ = self.create_client(
            Trigger, "left_camera_node/trigger_image"
        )
        self.left_camera_request_ = Trigger.Request()

        self.right_camera_client_ = self.create_client(
            Trigger, "right_camera_node/trigger_image"
        )
        self.right_camera_request_ = Trigger.Request()

    def initTimers(self):
        """
        Initialize main update timer for timerCB.
        """
        # convert timer period in [ms] to timper period in [s]
        timer_period_s = self.update_rate_ / 1000
        self.timer_ = self.create_timer(timer_period_s, self.timerCB)

        # Call the timer_  to prevent unused variable warnings
        self.timer_

    def initHeartbeat(self):
        """
        Initialize parameters for heartbeat message
        heartbeat_.data: string:
          name of node
        heartbeat_.update_rate: double:
          update rate of node
        """

        self.heartbeat_ = Heartbeat()

        self.heartbeat_.module.data = "perception"
        self.heartbeat_.update_rate = self.update_rate_
        # TODO: Put these in a config/yaml file and read from there
        self.MIN_HEIGHT = 720
        self.MIN_WIDTH = 1020
        self.MAX_FRAME_RATE = 1.0
        if self.first_img_arrived_ == True:
            self.previous_left_img_ = self.left_img_
            self.previous_right_img_ = self.right_img_

        self.heartbeat_publisher_ = self.create_publisher(
            Heartbeat, self.heartbeat_topic_, 1
        )

    def initConeTemplate(self):
        # create instance of cone as a template
        self.cone_template = Cone()
        self.cone_template.pos.x = 0.0
        self.cone_template.pos.y = 0.0
        self.cone_template.pos.z = 0.0
        self.cone_template.type = 0

    def publishHeartbeat(self):
        """
        Requires list of frames to check status of node.
        Publish heartbeat message.
        """
        self.heartbeat_.status = self.cameraStatus()
        self.heartbeat_.header.stamp = self.get_clock().now().to_msg()
        self.heartbeat_publisher_.publish(self.heartbeat_)

    def leftCameraCB(self, msg):
        """
        Callback function for left_cam_subscriber_
        """
        try:
            self.left_img_header = msg.header
            self.left_img_ = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            self.left_img_ = cv2.cvtColor(self.left_img_, cv2.COLOR_BayerRG2RGB)
            self.left_img_recieved_ = True

            if self.first_img_arrived_ == False:
                self.previous_left_img_ = self.left_img_
                self.first_img_arrived_ = True

        except CvBridgeError as e:
            exception = "Perception::leftCameraCB: " + e
            self.get_logger().error(exception)

    def rightCameraCB(self, msg):
        """
        Callback function for right_cam_subscriber_
        """
        try:
            self.right_img_header = msg.header
            self.right_img_ = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )
            self.right_img_ = cv2.cvtColor(self.right_img_, cv2.COLOR_BayerRG2RGB)
            self.right_img_recieved_ = True

            if self.first_img_arrived_ == False:
                self.previous_right_img_ = self.right_img_
                self.first_img_arrived_ = True

        except CvBridgeError as e:
            exception = "Perception::rightCameraCB: " + e
            self.get_logger().error(exception)

    def leftCameraReadyCB(self, msg):
        """
        Callback function for left_cam_subscriber_
        """
        self.left_ready_ = msg.data

    def rightCameraReadyCB(self, msg):
        """
        Callback function for right_cam_subscriber_
        """
        self.right_ready_ = msg.data

    def lidarCB(self, msg):
        """
        Callback function for processed lidar point cloud
        """
        # TODO - create a variable in initvariables which stores the most recent
        # processed lidar point cloud whenever lidarCB is called
        self.get_logger().warn("Received latest lidar message")
        self.lidar_msg = msg  # Store the incoming lidar data

    def process(self, left_img_, right_img_):
        """
        main detection function for perception node

        inputs:
          left_img_: left camera image, in a 3d array of width, height in rgb
          right_img_: right camera image, 3d array

        outputs:
          left_bounding_boxes: array of left camera detections ([x, y, w, h])
          right_bounding_boxes: array of right camera detections
          cone_detections: array of 3d cone detections using stereo ([x, y, z, color])
        """

        left_bounding_boxes, left_classes, left_scores, left_image = deep_process(
            left_img_,
            self.translation,
            self.intrinsics_left,
            self.session,
            self.confidence_,
        )
        right_bounding_boxes, right_classes, right_scores, right_image = deep_process(
            right_img_,
            self.translation,
            self.intrinsics_left,
            self.session,
            self.confidence_,
        )

        # change bounding_boxes_to_cone_detections to return 3d cone
        # detections using camera intrinsics and simple triangulation depth
        # need to refactor this: make cone detections separately
        # use the left_bounding_boxes, right_bounding_boxes to get depth measurements
        # get 3d estimates
        # ALSO: need to split cone_detections into left and right detections and return them
        # dont need to return the bounding boxes

        left_cone_detections = bounding_boxes_to_cone_detections(
            left_bounding_boxes, left_classes, self.intrinsics_left, self.cone_heights
        )

        right_cone_detections = bounding_boxes_to_cone_detections(
            right_bounding_boxes,
            right_classes,
            self.intrinsics_right,
            self.cone_heights,
        )

        return (
            left_bounding_boxes,
            left_classes,
            left_scores,
            right_bounding_boxes,
            right_classes,
            right_scores,
            left_cone_detections,
            right_cone_detections,
        )

    def timerCB(self):
        """
        Main Processing loop for perception module.
        Send Asynchronous Trigger to both cameras at once, and process
        incoming frames.
        """

        # initialize detection msg
        # TODO - make 1 detections message and combine them at the end
        self.detections_msg = ConeDetections()
        self.detections_msg.header.frame_id = "os_lidar"

        # publish the heartbeat
        self.publishHeartbeat()

        if not self.lidar_msg:
            return

        if self.lidar_only_detection == False:
            # check if ready
            if not self.left_ready_ or not self.right_ready_:
                return

            trigger = Trigger.Request()

            # send asynchronous trigger
            self.future = self.left_camera_client_.call_async(trigger)
            self.future = self.right_camera_client_.call_async(trigger)

            if not self.left_img_recieved_:
                return

            if not self.right_img_recieved_:
                return

            # undistort

            undist_left = cv2.remap(
                self.left_img_,
                self.mapx_left,
                self.mapy_left,
                interpolation=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0, 0),
            )

            undist_right = cv2.remap(
                self.right_img_,
                self.mapx_right,
                self.mapy_right,
                interpolation=cv2.INTER_NEAREST,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0, 0),
            )

            # code to resize the image (for faster fps)

            """
            frame_left_60, frame_right_60 = self.resize_img(undist_left, undist_right, 60)
            frame_left = frame_left_60
            frame_right = frame_right_70
        
        if self.save_pic == "True":
        timestamp = int(time.time())
        local_time = time.ctime(timestamp).replace(" ", "_")
        self.save_image(frame_left, frame_right, local_time)
        """

            # equalize histogram
            """
            frame_left = self.equalize_hist(frame_left)
            frame_right = self.equalize_hist(frame_right)
            """

            frame_left = undist_left
            frame_right = undist_right

            try:
                # tf from left_cam to lidar

                tf_leftcam_lidar = self.tf_buffer.lookup_transform(
                    self.lidar_frame,
                    self.left_camera_frame,
                    time=rclpy.time.Time(seconds=0),
                    timeout=rclpy.time.Duration(seconds=0.1),
                )

                # tf from right_cam to lidar
                tf_rightcam_lidar = self.tf_buffer.lookup_transform(
                    self.lidar_frame,
                    self.right_camera_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )

            except TransformException as ex:
                self.get_logger().info(f"{ex}")
                return
            # get the detections

            (
                results_left,
                classes_left,
                scores_left,
                results_right,
                classes_right,
                scores_right,
                left_cone_detections,
                right_cone_detections,
            ) = self.process(frame_left, frame_right)

            # transform camera detections to lidar frame
            left_detections_lidar_frame = transform_det_lidar(
                left_cone_detections, tf_leftcam_lidar
            )
            right_detections_lidar_frame = transform_det_lidar(
                right_cone_detections, tf_rightcam_lidar
            )

            # Extract point cloud data
            lidar_point_cloud_data = point_cloud2.read_points_numpy(
                self.lidar_msg, field_names=["x", "y", "z"], skip_nans=True
            )

            print(left_detections_lidar_frame.shape, right_detections_lidar_frame.shape)
            # concatenate into one array
            if left_detections_lidar_frame.shape[0] == 0:
                total_cam_det = right_detections_lidar_frame
            elif right_detections_lidar_frame.shape[0] == 0:
                total_cam_det = left_detections_lidar_frame
            else:
                total_cam_det = np.concatenate(
                    (left_detections_lidar_frame, right_detections_lidar_frame), axis=0
                )
            # total_cam_det = left_detections_lidar_frame
            # total_cam_det = right_detections_lidar_frame

            # perception detections debug
            self.detections_debug = ConeDetections()
            self.detections_debug.header.frame_id = "os_lidar"

            self.detections_debug.header.stamp = self.get_clock().now().to_msg()

            for i in range(len(total_cam_det)):
                self.cone_template = Cone()
                self.cone_template.pos.x = float(total_cam_det[i][0])  # left
                self.cone_template.pos.y = float(total_cam_det[i][1])  # up
                self.cone_template.pos.z = float(total_cam_det[i][2])  # front
                self.cone_template.type = int(total_cam_det[i][3])

                if self.cone_template.type == 1:  # blue
                    self.detections_debug.left_cones.append(self.cone_template)
                elif self.cone_template.type == 2:  # yellow
                    self.detections_debug.right_cones.append(self.cone_template)
                elif self.cone_template.type == 3:
                    self.detections_debug.small_orange_cones.append(self.cone_template)
                elif self.cone_template.type == 4:
                    self.detections_debug.large_orange_cones.append(self.cone_template)
                else:  # unknown cones cone template type == 0
                    self.detections_debug.unknown_cones.append(self.cone_template)

            self.cone_detections_debug_.publish(self.detections_debug)

            # hungarian matching
            if total_cam_det.shape[0] > 0:

                diffs = lidar_point_cloud_data[:, :3].reshape(-1, 1, 3) - total_cam_det[
                    :, :3
                ].reshape(1, -1, 3)

                cost_matrix = np.linalg.norm(diffs, axis=2)

                row_ind, col_ind = linear_sum_assignment(cost_matrix)

                positions = lidar_point_cloud_data[row_ind][:, :3]
                colours = total_cam_det[col_ind][:, 3:]

                # TODO - cost matrix where indices have cost above certain threshold, make the type unknown
                costs = cost_matrix[row_ind, col_ind]
                above_threshold = np.where(costs > 4.0)[0]
                for ind in above_threshold:
                    colours[ind] = 0

                cone_detections = np.concatenate((positions, colours), axis=1)
                print(col_ind)
                camera_det = np.delete(total_cam_det, col_ind, axis=0)
                cone_detections = np.concatenate((cone_detections, camera_det), axis=0)
                print(cone_detections.shape)
            else:

                positions = lidar_point_cloud_data[:, :3]
                colours = np.zeros((positions.shape[0], 1))
                cone_detections = np.concatenate((positions, colours), axis=1)

            # self.visualize_detections(frame_left, frame_right, results_left, results_right, cone_detections)

            # perception debug msg

            if len(results_left) == 0:
                pass
            else:
                self.perception_debug_msg_left = PerceptionDebug()
                self.perception_debug_msg_left.header.stamp = self.left_img_header.stamp
                for i in range(len(results_left)):
                    bounding_box_left = BoundingBox()
                    bounding_box_left.x = int(results_left[i][0])
                    bounding_box_left.y = int(results_left[i][1])
                    bounding_box_left.width = int(results_left[i][2])
                    bounding_box_left.height = int(results_left[i][3])
                    bounding_box_left.type = labelColor(classes_left[i])
                    bounding_box_left.score = scores_left[i]
                    self.perception_debug_msg_left.left.append(bounding_box_left)

                self.perception_debug_publisher_left_.publish(
                    self.perception_debug_msg_left
                )

            if len(results_right) == 0:
                pass
            else:
                self.perception_debug_msg_right = PerceptionDebug()
                self.perception_debug_msg_right.header.stamp = (
                    self.right_img_header.stamp
                )
                for i in range(len(results_right)):
                    bounding_box_right = BoundingBox()
                    bounding_box_right.x = int(results_right[i][0])
                    bounding_box_right.y = int(results_right[i][1])
                    bounding_box_right.width = int(results_right[i][2])
                    bounding_box_right.height = int(results_right[i][3])
                    bounding_box_right.type = labelColor(classes_right[i])
                    bounding_box_right.score = scores_right[i]
                    self.perception_debug_msg_right.right.append(bounding_box_right)

                self.perception_debug_publisher_right_.publish(
                    self.perception_debug_msg_right
                )

            # imshow for opencv
            """
            cv2.imshow('left_camera', frame_left)
            k = cv2.waitKey(1)

            cv2.imshow('right_camera', frame_right)
            cv2.waitKey(1)
            """

            # print("cone detections: " + str(cone_detections))

            if cone_detections.size != 0:
                # order cones by distance
                # cone_detections = cone_detections[np.argsort(cone_detections[:, 2])]

                # publish cone detections
                self.detections_msg.header.stamp = self.get_clock().now().to_msg()

                for i in range(len(cone_detections)):
                    self.cone_template = Cone()
                    self.cone_template.pos.x = float(cone_detections[i][0])  # left
                    self.cone_template.pos.y = float(cone_detections[i][1])  # up
                    self.cone_template.pos.z = float(cone_detections[i][2])  # front
                    self.cone_template.type = int(cone_detections[i][3])

                    if self.cone_template.type == 1:  # blue
                        self.detections_msg.left_cones.append(self.cone_template)
                    elif self.cone_template.type == 2:  # yellow
                        self.detections_msg.right_cones.append(self.cone_template)
                    elif self.cone_template.type == 3:
                        self.detections_msg.small_orange_cones.append(
                            self.cone_template
                        )
                    elif self.cone_template.type == 4:
                        self.detections_msg.large_orange_cones.append(
                            self.cone_template
                        )
                    else:  # unknown cones cone template type == 0
                        self.detections_msg.unknown_cones.append(self.cone_template)

                self.detections_msg.header.stamp = self.get_clock().now().to_msg()

                self.cone_detections_publisher_.publish(self.detections_msg)

            self.left_img_recieved_ = False
            self.right_img_recieved_ = False

        else:
            print("LIDAR_ONLY")
            # Extract point cloud data
            lidar_point_cloud_data = point_cloud2.read_points_numpy(
                self.lidar_msg, field_names=["x", "y", "z"], skip_nans=True
            )

            for i in range(len(lidar_point_cloud_data)):
                self.cone_template = Cone()
                self.cone_template.pos.x = float(lidar_point_cloud_data[i][0])  # left
                self.cone_template.pos.y = float(lidar_point_cloud_data[i][1])  # up
                self.cone_template.pos.z = float(lidar_point_cloud_data[i][2])  # front
                self.cone_template.type = 1

                self.detections_msg.left_cones.append(self.cone_template)

            self.detections_msg.header.stamp = self.get_clock().now().to_msg()
            self.cone_detections_publisher_.publish(self.detections_msg)

    # Helper functions:

    def save_image(self, left_img_, right_img_, rosbag_name):
        left_img_name = rosbag_name + "_" + str(self.saved_count) + ".png"
        cv2.imwrite(
            "/home/utfrdv/utfr_dv_ros2/src/perception/images_to_mp4_left/"
            + left_img_name,
            left_img_,
        )

        right_img_name = rosbag_name + "_" + str(self.saved_count) + ".png"
        cv2.imwrite(
            "/home/utfrdv/utfr_dv_ros2/src/perception/images_to_mp4_right/"
            + right_img_name,
            right_img_,
        )

        self.saved_count += 1

    def resize_img(self, left_img_, right_img_, scale_percent):
        # Downsize image resolution/size

        right_width = int(right_img_.shape[1] * scale_percent / 100)
        right_height = int(right_img_.shape[0] * scale_percent / 100)
        right_dim = (right_width, right_height)

        # resize image
        frame_right = cv2.resize(right_img_, right_dim, interpolation=cv2.INTER_AREA)

        left_width = int(left_img_.shape[1] * scale_percent / 100)
        left_height = int(left_img_.shape[0] * scale_percent / 100)
        left_dim = (left_width, left_height)

        # resize image
        frame_left = cv2.resize(left_img_, left_dim, interpolation=cv2.INTER_AREA)

        return frame_left, frame_right

    def visualize_detections(
        self, frame_left, frame_right, results_left, results_right, cone_detections
    ):
        i = 0
        # visualizing detections loop
        for x, y, w, h in results_left:
            if y <= 470:
                continue
            cv2.rectangle(frame_left, (x, y), (x + w, y + h), (255, 0, 0), 2)
            x_3d, y_3d, z_3d, color = cone_detections[i]
            cv2.putText(
                frame_left,
                text="("
                + str(round(x_3d, 2))
                + ", "
                + str(round(y_3d, 2))
                + ", "
                + str(round(z_3d, 2))
                + ")",
                org=(int(x), int(y - 10)),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.5,
                color=(0, 0, 255),
                thickness=1,
                lineType=cv2.LINE_AA,
            )
            i += 1

        # visualizing cone detections in right camera
        for x, y, w, h in results_right:
            # if y <= 470:
            # continue
            cv2.rectangle(frame_right, (x, y), (x + w, y + h), (255, 0, 0), 2)

    def equalize_hist(self, image):
        # convert from RGB color-space to YCrCb
        ycrcb_img = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)

        # equalize the histogram of the Y channel
        ycrcb_img[:, :, 0] = cv2.equalizeHist(ycrcb_img[:, :, 0])

        # convert back to RGB color-space from YCrCb
        equalized_img = cv2.cvtColor(ycrcb_img, cv2.COLOR_YCrCb2BGR)

        return equalized_img

    def hasVisualArtifacts(self, frame):
        """
        Returns true if the frame has visual artifacts, defined
        as contours found after blurring and thresholding the image.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return len(contours) > 0

    def frameChanged(self, frame, prev_frame):
        """
        Returns True if the frame has changed significantly from the previous frame.
        """

        diff = cv2.absdiff(frame, prev_frame)
        gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 20, 255, cv2.THRESH_BINARY)
        dilated = cv2.dilate(thresh, None, iterations=3)
        contours, _ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return len(contours) > 0

    def cameraStatus(self):
        """
        Takes in list of frames and returns status of node.
        - Checks if no frame has been received
        - Checks if frame is too small
        - Checks if FPS is too low
        - Checks for visual artifacts
        - Otherwise, node is active
        """
        if self.first_img_arrived_ == False:
            return 0  # UNINITIALIZED
        elif self.left_img_ is None or self.right_img_ is None:
            return 0  # UNINITIALIZED
        # Check if frame is too small
        elif (
            self.left_img_.shape[0] < self.MIN_HEIGHT
            or self.left_img_.shape[1] < self.MIN_WIDTH
        ):
            return 2  # FATAL
        elif (
            self.right_img_.shape[0] < self.MIN_HEIGHT
            or self.right_img_.shape[1] < self.MIN_WIDTH
        ):
            return 2  # FATAL

        # TODO - fix fps, frame change cases

        # Check if FPS is too low
        # elif self.left_timestamp_ - self.right_timestamp_ > self.MAX_FRAME_RATE:
        #   return 2 #FATAL
        # elif self.right_timestamp_ - self.left_timestamp_ > self.MAX_FRAME_RATE:
        #   return 2 #FATAL
        elif self.hasVisualArtifacts(self.left_img_):
            return 2  # FATAL
        elif self.hasVisualArtifacts(self.right_img_):
            return 2  # FATAL
        # Check if frame has changed significantly (stores a boolean)
        left_frame_changed = self.frameChanged(self.left_img_, self.previous_left_img_)
        right_frame_changed = self.frameChanged(
            self.right_img_, self.previous_right_img_
        )
        if left_frame_changed == False or right_frame_changed == False:
            # Update previous frames
            self.previous_left_img_ = self.left_img_
            self.previous_right_img_ = self.right_img_
            return 0  # UNINITIALIZED
        else:
            return 1  # ACTIVE
        return 1

    def monitorCamera(self):
        """
        Monitor camera status and publish heartbeat message.
        Function or line below to be called in the main loop.
        """
        self.publishHeartbeat()


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
