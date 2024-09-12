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
from scipy.optimize import linear_sum_assignment

# import xgboost
from ultralytics import YOLO

# ROS2 Requirements
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge, CvBridgeError
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
import torch

# Message Requirements
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from utfr_msgs.msg import ConeDetections
from utfr_msgs.msg import Heartbeat
from utfr_msgs.msg import Cone
from utfr_msgs.msg import BoundingBox
from utfr_msgs.msg import PerceptionDebug
from geometry_msgs.msg import PointStamped, Point


# Service Requirements
from std_srvs.srv import Trigger

# Import deep process functions
from perception.submodules.deep import deep_process
from perception.submodules.deep import check_for_cuda
from perception.submodules.deep import labelColor

from concurrent.futures import ThreadPoolExecutor


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        self.left_image_group = MutuallyExclusiveCallbackGroup()
        self.right_image_group = MutuallyExclusiveCallbackGroup()
        self.lidar_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = ReentrantCallbackGroup()

        self.loadParams()
        self.initVariables()
        self.initHeartbeat()
        self.initSubscribers()
        self.initPublishers()
        self.initServices()
        self.initConeTemplate()
        self.initTimers()

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
        self.declare_parameter(
            "left_camera_topic", "/left_camera_node/images/compressed"
        )
        self.declare_parameter(
            "right_camera_topic", "/right_camera_node/images/compressed"
        )
        self.declare_parameter("cone_detections_topic", "/perception/cone_detections")
        self.declare_parameter("heartbeat_topic", "/perception/heartbeat")
        self.declare_parameter("processed_lidar_topic", "/lidar_pipeline/clustered")
        self.declare_parameter("update_rate", 33.33)
        self.declare_parameter("debug", True)
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
        self.debug_ = self.get_parameter("debug").get_parameter_value().bool_value
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
        self.new_left_img_recieved_ = False
        self.new_right_img_recieved_ = False
        self.new_lidar_msg_received_ = False

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
            None,
            self.intrinsics_right,
            self.img_size,
            cv2.CV_32FC1,
        )
        self.mapx_left, self.mapy_left = cv2.initUndistortRectifyMap(
            self.intrinsics_left,
            self.distortion_left,
            None,
            self.intrinsics_left,
            self.img_size,
            cv2.CV_32FC1,
        )

        # create ultralytics model for inference
        file_name = "src/perception/perception/models/yolov8n_batched.engine"
        # file_name = "src/perception/perception/yolov8n.pt"
        # file_name = "src/perception/perception/yolov8n.onnx"
        print("Deep filename: ", file_name)

        if file_name.endswith(".engine"):
            # TensorRT model cannot run on CPU, so only run on GPU
            if torch.cuda.is_available():
                print("CUDA is available. Using GPU with TensorRT model...")
                self.model = YOLO(file_name, task="detect")  # TensorRT model
                # TensorRT models do not need to be transferred with .to("cuda")
            else:
                raise RuntimeError(
                    "TensorRT model requires CUDA, but CUDA is not available. Cannot run on CPU."
                )
        else:
            # Assuming a PyTorch model (.pt), it can be loaded and used on CPU or GPU
            self.model = YOLO(file_name, task="detect")

            if torch.cuda.is_available():
                print("CUDA is available. Using GPU for PyTorch model...")
                self.model.to("cuda")
            else:
                print("CUDA is not available. Using CPU for PyTorch model...")
                self.model.to("cpu")
        # self.model.export(format="engine")

        # create transform frame variables
        self.lidar_frame = "os_sensor"
        self.left_camera_frame = "left_camera"
        self.right_camera_frame = "right_camera"

        self.left_frame_id = 0
        self.right_frame_id = 0

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
            CompressedImage,
            self.left_camera_topic,
            self.leftCameraCB,
            1,
            callback_group=self.left_image_group,
        )

        self.right_cam_subscriber_ = self.create_subscription(
            CompressedImage,
            self.right_camera_topic,
            self.rightCameraCB,
            1,
            callback_group=self.right_image_group,
        )

        self.processed_lidar_subscriber_ = self.create_subscription(
            PointCloud2,
            self.processed_lidar_topic,
            self.lidarCB,
            1,
            callback_group=self.lidar_group,
        )

        # Latching Subscribers:
        qos_latching = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.left_cam_ready_subscriber_ = self.create_subscription(
            Bool, "/left_image/ready", self.leftCameraReadyCB, qos_latching
        )

        self.right_cam_ready_subscriber_ = self.create_subscription(
            Bool, "/right_image/ready", self.rightCameraReadyCB, qos_latching
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

        self.heartbeat_publisher_ = self.create_publisher(
            Heartbeat, self.heartbeat_topic_, 1
        )

        if self.debug_:
            self.undistorted_publisher_left_ = self.create_publisher(
                Image, "/perception/debug_undistorted_left", 1
            )

            self.undistorted_publisher_right_ = self.create_publisher(
                Image, "/perception/debug_undistorted_right", 1
            )

            self.perception_debug_publisher_left_ = self.create_publisher(
                PerceptionDebug, self.perception_debug_topic_ + "_left", 1
            )

            self.perception_debug_publisher_right_ = self.create_publisher(
                PerceptionDebug, self.perception_debug_topic_ + "_right", 1
            )

            self.lidar_projection_publisher_left_ = self.create_publisher(
                PerceptionDebug, "/perception/lidar_projection_left", 1
            )

            self.lidar_projection_publisher_right_ = self.create_publisher(
                PerceptionDebug, "/perception/lidar_projection_right", 1
            )

            self.lidar_projection_publisher_matched_left_ = self.create_publisher(
                PerceptionDebug, "/perception/lidar_projection_matched_left", 1
            )

            self.lidar_projection_publisher_matched_right_ = self.create_publisher(
                PerceptionDebug, "/perception/lidar_projection_matched_right", 1
            )

        self.left_cam_processed_publisher_ = self.create_publisher(
            Heartbeat, "/perception/left_processed", 1
        )

        self.right_cam_processed_publisher_ = self.create_publisher(
            Heartbeat, "/perception/right_processed", 1
        )

    def initServices(self):
        """
        Initialize Services

        left_camera_client_ : triggers left camera shutter
          Trigger, topic: left_image/trigger_image

        right_camera_client_ : triggers right camera shutter
          Trigger, topic: right_image/trigger_image
        """

        self.left_camera_client_ = self.create_client(
            Trigger, "left_image/trigger_image"
        )
        self.left_camera_request_ = Trigger.Request()

        self.right_camera_client_ = self.create_client(
            Trigger, "right_image/trigger_image"
        )
        self.right_camera_request_ = Trigger.Request()

        self.caml = (
            self.get_clock().now().seconds_nanoseconds()[0]
            + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        )

    def initTimers(self):
        """
        Initialize main update timer for timerCB.
        """
        # convert timer period in [ms] to timper period in [s]
        timer_period_s = self.update_rate_ / 1000
        self.timer_ = self.create_timer(
            timer_period_s, self.timerCB, callback_group=self.timer_group
        )

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
        Callback function for left_cam_subscriber_ with CompressedImage message
        """
        now = (
            self.get_clock().now().seconds_nanoseconds()[0]
            + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        )
        print("Cam CB: ", now - self.caml)
        print("-----------")
        self.caml = now
        try:
            # Convert the CompressedImage message to a CV2 image
            np_arr = np.frombuffer(msg.data, np.uint8)
            left_img_ = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode JPEG image
            left_img_gpu = cv2.cuda_GpuMat()
            left_img_gpu.upload(left_img_)

            mapx_left_gpu = cv2.cuda_GpuMat()
            mapx_left_gpu.upload(self.mapx_left)

            mapy_left_gpu = cv2.cuda_GpuMat()
            mapy_left_gpu.upload(self.mapy_left)

            left_img_ = cv2.cuda.remap(
                left_img_gpu,
                mapx_left_gpu,
                mapy_left_gpu,
                interpolation=cv2.INTER_NEAREST,
            )

            self.left_img_ = left_img_.download()

            # Check if the image is valid
            if self.left_img_ is not None:
                self.left_img_header = msg.header
                self.left_frame_id = msg.header.frame_id
                self.new_left_img_recieved_ = True

                if not self.first_img_arrived_:
                    self.previous_left_img_ = self.left_img_
                    self.first_img_arrived_ = True
            else:
                self.get_logger().warn("Received an empty image")

        except CvBridgeError as e:
            exception = "Perception::leftCameraCB: " + str(e)
            self.get_logger().error(exception)
        except Exception as e:
            exception = "Perception::leftCameraCB: " + str(e)
            self.get_logger().error(exception)

        left_process = Heartbeat()
        left_process.header.stamp = self.get_clock().now().to_msg()
        self.left_cam_processed_publisher_.publish(left_process)

    def rightCameraCB(self, msg):
        """
        Callback function for right_cam_subscriber_ with CompressedImage message
        """
        try:
            # Convert the CompressedImage message to a CV2 image
            np_arr = np.frombuffer(msg.data, np.uint8)
            right_img_ = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode JPEG image
            right_img_gpu = cv2.cuda_GpuMat()
            right_img_gpu.upload(right_img_)

            mapx_right_gpu = cv2.cuda_GpuMat()
            mapx_right_gpu.upload(self.mapx_right)

            mapy_right_gpu = cv2.cuda_GpuMat()
            mapy_right_gpu.upload(self.mapy_right)

            right_img_ = cv2.cuda.remap(
                right_img_gpu,
                mapx_right_gpu,
                mapy_right_gpu,
                interpolation=cv2.INTER_NEAREST,
            )

            # convert to cpu
            self.right_img_ = right_img_.download()

            # Check if the image is valid
            if self.right_img_ is not None:
                self.right_img_header = msg.header
                self.right_frame_id = msg.header.frame_id
                self.new_right_img_recieved_ = True

                if not self.first_img_arrived_:
                    self.previous_right_img_ = self.right_img_
                    self.first_img_arrived_ = True
            else:
                self.get_logger().warn("Received an empty image")

        except CvBridgeError as e:
            exception = "Perception::rightCameraCB: " + str(e)
            self.get_logger().error(exception)
        except Exception as e:
            exception = "Perception::rightCameraCB: " + str(e)
            self.get_logger().error(exception)

        right_process = Heartbeat()
        right_process.header.stamp = self.get_clock().now().to_msg()
        self.right_cam_processed_publisher_.publish(right_process)

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
        # self.get_logger().warn("Received latest lidar message")
        self.lidar_msg = msg  # Store the incoming lidar data
        if self.lidar_msg:
            self.new_lidar_msg_received_ = True

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
        start = (
            self.get_clock().now().seconds_nanoseconds()[0]
            + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        )
        (
            left_bounding_boxes,
            left_classes,
            left_scores,
            right_bounding_boxes,
            right_classes,
            right_scores,
        ) = deep_process(
            self.model,
            left_img_,
            right_img_,
            self.confidence_,
        )
        end = (
            self.get_clock().now().seconds_nanoseconds()[0]
            + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        )
        print("deep process time: ", end - start)

        return (
            left_bounding_boxes,
            left_classes,
            left_scores,
            right_bounding_boxes,
            right_classes,
            right_scores,
        )

    def timerCB(self):
        """
        Main Processing loop for perception module.
        Send Asynchronous Trigger to both cameras at once, and process
        incoming frames.
        """

        # Wait for new lidar msg
        if not self.lidar_msg:
            return

        if self.lidar_only_detection == False:

            # check if ready
            # if not self.left_ready_ or not self.right_ready_:
            #     return

            # trigger = Trigger.Request()

            # # send asynchronous trigger
            # self.future = self.left_camera_client_.call_async(trigger)
            # self.future = self.right_camera_client_.call_async(trigger)

            if not self.new_left_img_recieved_ or not self.new_right_img_recieved_:
                return

            try:
                # tf from lidar to left_cam
                tf_lidar_to_leftcam = self.tf_buffer.lookup_transform(
                    self.left_camera_frame,
                    "ground",
                    time=rclpy.time.Time(seconds=0),
                    timeout=rclpy.time.Duration(seconds=0.1),
                )

                # tf from lidar to right cam
                tf_lidar_to_rightcam = self.tf_buffer.lookup_transform(
                    self.right_camera_frame,
                    "ground",
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1),
                )

            except TransformException as ex:
                self.get_logger().info(f"{ex}")
                return

            frame_left = self.left_img_
            frame_right = self.right_img_
            (
                results_left,
                classes_left,
                scores_left,
                results_right,
                classes_right,
                scores_right,
            ) = self.process(frame_left, frame_right)

            # Extract point cloud data
            lidar_point_cloud_data = point_cloud2.read_points_numpy(
                self.lidar_msg, field_names=["x", "y", "z"], skip_nans=True
            )

            # transform lidar detections to left cam frame
            lidar_det_leftcam_frame = self.transform_det_lidar(
                lidar_point_cloud_data, tf_lidar_to_leftcam
            )

            # tf lidar to right cam frame
            lidar_det_rightcam_frame = self.transform_det_lidar(
                lidar_point_cloud_data, tf_lidar_to_rightcam
            )

            # Transform 3D camera frame to 3D camera optical frame (axis swap)
            # x in the left cam frame = z in the transformation frame
            lidar_det_leftcam_frame = self.tf_cam_axis_swap(lidar_det_leftcam_frame)
            lidar_det_rightcam_frame = self.tf_cam_axis_swap(lidar_det_rightcam_frame)

            # 3D optical frame to 2D pixel coordinates projection with depth
            left_projected_pts = self.point_3d_to_image(
                lidar_det_leftcam_frame, self.intrinsics_left
            )  # list of u, v, depth
            left_projected_pts[:, :2] = np.round(left_projected_pts[:, :2])

            right_projected_pts = self.point_3d_to_image(
                lidar_det_rightcam_frame, self.intrinsics_right
            )  # list of u, v, depth
            right_projected_pts[:, :2] = np.round(right_projected_pts[:, :2])

            valid_idx_left = self.filter_points_in_fov(
                left_projected_pts, self.img_size
            )
            valid_idx_right = self.filter_points_in_fov(
                right_projected_pts, self.img_size
            )
            filtered_left_projected_pts = left_projected_pts[valid_idx_left]
            filtered_right_projected_pts = right_projected_pts[valid_idx_right]

            # Hungarian algorithm for matching
            matched_cone_dets = self.h_matching(
                filtered_left_projected_pts,
                filtered_right_projected_pts,
                results_left,
                results_right,
                duplicate_threshold=0.1,
                cost_threshold=300.0,
            )

            # Bounding box showing lidar point in camera frame

            if self.debug_:
                self.publish_2d_projected_det(
                    left_projected_pts=filtered_left_projected_pts,
                    left_stamp=self.left_img_header.stamp,
                    right_projected_pts=filtered_right_projected_pts,
                    right_stamp=self.right_img_header.stamp,
                )
                # TODO: Temp for debugging, need to view as cone det properly in 3D
                self.publish_2d_projected_det_matched(
                    left_projected_pts=matched_cone_dets,
                    left_stamp=self.left_img_header.stamp,
                    right_projected_pts=matched_cone_dets,
                    right_stamp=self.right_img_header.stamp,
                )
                self.displayBoundingBox(
                    results_left,
                    classes_left,
                    scores_left,
                    results_right,
                    classes_right,
                    scores_right,
                )

            # self.publish_cone_detection(cone_detections)

            self.left_img_recieved_ = False
            self.right_img_recieved_ = False

            if self.debug_:
                self.publish_undistorted(frame_left, frame_right)

        else:
            self.publish_cone_dets_lidar()

        # publish the heartbeat
        self.publishHeartbeat()

    # Helper functions
    def publish_cone_dets_lidar(self):
        print("LIDAR_ONLY")
        # Extract point cloud data
        lidar_point_cloud_data = point_cloud2.read_points_numpy(
            self.lidar_msg, field_names=["x", "y", "z"], skip_nans=True
        )
        detections_msg = ConeDetections()
        detections_msg.header.frame_id = "ground"

        for i in range(len(lidar_point_cloud_data)):
            cone_template = Cone()
            cone_template.pos.x = float(lidar_point_cloud_data[i][0])  # left
            cone_template.pos.y = float(lidar_point_cloud_data[i][1])  # up
            cone_template.pos.z = float(lidar_point_cloud_data[i][2])  # front
            cone_template.type = 1

            detections_msg.unknown_cones.append(cone_template)

        detections_msg.header.stamp = self.get_clock().now().to_msg()
        self.cone_detections_publisher_.publish(detections_msg)

    def publish_cone_dets(self, cone_detections):
        print("cone detections: " + str(cone_detections))

        if cone_detections.size != 0:
            # order cones by distance
            # cone_detections = cone_detections[np.argsort(cone_detections[:, 2])]

            # publish cone detections
            detections_msg = ConeDetections()
            detections_msg.header.frame_id = "ground"

            for i in range(len(cone_detections)):
                cone_template = Cone()
                cone_template.pos.x = float(cone_detections[i][0])  # left
                cone_template.pos.y = float(cone_detections[i][1])  # up
                cone_template.pos.z = float(cone_detections[i][2])  # front
                cone_template.type = int(cone_detections[i][3])

                if cone_template.type == 1:  # blue
                    detections_msg.left_cones.append(cone_template)
                elif cone_template.type == 2:  # yellow
                    detections_msg.right_cones.append(cone_template)
                elif cone_template.type == 3:
                    detections_msg.small_orange_cones.append(cone_template)
                elif cone_template.type == 4:
                    detections_msg.large_orange_cones.append(cone_template)
                else:  # unknown cones cone template type == 0
                    detections_msg.unknown_cones.append(cone_template)

            detections_msg.header.stamp = self.get_clock().now().to_msg()
            self.cone_detections_publisher_.publish(detections_msg)

    def publish_2d_projected_det(
        self, left_projected_pts, left_stamp, right_projected_pts, right_stamp
    ):
        perception_debug_msg_left = PerceptionDebug()
        perception_debug_msg_left.header.stamp = left_stamp
        perception_debug_msg_left.header.frame_id = "raw"

        for point in left_projected_pts:
            bounding_box_left = BoundingBox()
            bounding_box_left.x = int(point[0])
            bounding_box_left.y = int(point[1])
            bounding_box_left.width = 20
            bounding_box_left.height = 20
            perception_debug_msg_left.left.append(bounding_box_left)

        self.lidar_projection_publisher_left_.publish(perception_debug_msg_left)

        perception_debug_msg_right = PerceptionDebug()
        perception_debug_msg_right.header.stamp = self.right_img_header.stamp
        perception_debug_msg_right.header.frame_id = "raw"

        for point in right_projected_pts:
            bounding_box_right = BoundingBox()
            bounding_box_right.x = int(point[0])
            bounding_box_right.y = int(point[1])
            bounding_box_right.width = 20
            bounding_box_right.height = 20
            perception_debug_msg_right.right.append(bounding_box_right)

        self.lidar_projection_publisher_right_.publish(perception_debug_msg_right)

    def publish_2d_projected_det_matched(
        self, left_projected_pts, left_stamp, right_projected_pts, right_stamp
    ):
        perception_debug_msg_left = PerceptionDebug()
        perception_debug_msg_left.header.stamp = left_stamp
        perception_debug_msg_left.header.frame_id = "matched"

        for point in left_projected_pts:
            if point[1] == "left":
                bounding_box_left = BoundingBox()
                bounding_box_left.x = int(point[0][0])
                bounding_box_left.y = int(point[0][1])
                bounding_box_left.width = 20
                bounding_box_left.height = 20
                perception_debug_msg_left.left.append(bounding_box_left)

        self.lidar_projection_publisher_matched_left_.publish(perception_debug_msg_left)

        perception_debug_msg_right = PerceptionDebug()
        perception_debug_msg_right.header.stamp = self.right_img_header.stamp
        perception_debug_msg_right.header.frame_id = "matched"

        for point in right_projected_pts:
            if point[1] == "right":
                bounding_box_right = BoundingBox()
                bounding_box_right.x = int(point[0][0])
                bounding_box_right.y = int(point[0][1])
                bounding_box_right.width = 20
                bounding_box_right.height = 20
                perception_debug_msg_right.right.append(bounding_box_right)

        self.lidar_projection_publisher_matched_right_.publish(
            perception_debug_msg_right
        )

    def publish_undistorted(self, frame_left, frame_right):
        """Print undistorted left and right images"""
        self.undistorted_publisher_left_.publish(self.bridge.cv2_to_imgmsg(frame_left))
        self.undistorted_publisher_right_.publish(
            self.bridge.cv2_to_imgmsg(frame_right)
        )

    def displayBoundingBox(
        self,
        results_left,
        classes_left,
        scores_left,
        results_right,
        classes_right,
        scores_right,
    ):
        # Bounding boxes from deep learning model
        if len(results_left) != 0:
            perception_debug_msg_left = PerceptionDebug()
            perception_debug_msg_left.header.stamp = self.left_img_header.stamp
            for i in range(len(results_left)):
                bounding_box_left = BoundingBox()
                bounding_box_left.x = int(results_left[i][0])
                bounding_box_left.y = int(results_left[i][1])
                bounding_box_left.width = int(results_left[i][2])
                bounding_box_left.height = int(results_left[i][3])
                bounding_box_left.type = labelColor(classes_left[i])
                bounding_box_left.score = scores_left[i]
                perception_debug_msg_left.left.append(bounding_box_left)

            self.perception_debug_publisher_left_.publish(perception_debug_msg_left)

        if len(results_right) != 0:
            perception_debug_msg_right = PerceptionDebug()
            perception_debug_msg_right.header.stamp = self.right_img_header.stamp
            for i in range(len(results_right)):
                bounding_box_right = BoundingBox()
                bounding_box_right.x = int(results_right[i][0])
                bounding_box_right.y = int(results_right[i][1])
                bounding_box_right.width = int(results_right[i][2])
                bounding_box_right.height = int(results_right[i][3])
                bounding_box_right.type = labelColor(classes_right[i])
                bounding_box_right.score = scores_right[i]
                perception_debug_msg_right.right.append(bounding_box_right)

            self.perception_debug_publisher_right_.publish(perception_debug_msg_right)

    def tf_cam_axis_swap(self, points):
        return np.column_stack(
            [-points[:, 1], -points[:, 2], points[:, 0]]  # -y  # -z  # x
        )

    def point_3d_to_image(self, points, camera_matrix):
        """Projects 3D points to 2D image coordinates using a camera matrix."""
        # Ensure the camera matrix is a NumPy array
        camera_matrix = np.array(camera_matrix).reshape(3, 3)

        # Transform points to normalized camera coordinates
        # No need for adding homogeneous coordinates; just use points directly
        normalized_coords = np.dot(points, camera_matrix.T)

        # Convert to 2D image coordinates
        u = normalized_coords[:, 0] / normalized_coords[:, 2]
        v = normalized_coords[:, 1] / normalized_coords[:, 2]
        depth = normalized_coords[:, 2]

        # Combine u, v, and depth into a single array
        result = np.stack((u, v, depth), axis=-1)

        return result

    def transform_det_lidar(self, lidar_points, transform):
        """Transforms from lidar detections to camera frames using NumPy for efficiency."""
        try:
            # Create a list to store transformed points
            transformed_points = []

            # Iterate over each point and transform
            for point in lidar_points:
                # Create a PointStamped object
                p = PointStamped()
                p.point = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))

                p_tf = tf2_geometry_msgs.do_transform_point(p, transform).point
                transformed_points.append([p_tf.x, p_tf.y, p_tf.z])

            return np.array(transformed_points)

        except TransformException as ex:
            print(ex)
            return np.array([])  # Return an empty array in case of an exception

    def cost_mtx_from_bbox(
        self, projected_lidar_detections, camera_detections, depth_weight
    ):
        cost_matrix = np.zeros(
            (len(projected_lidar_detections), len(camera_detections))
        )
        for i, (lidar_x, lidar_y, lidar_depth) in enumerate(projected_lidar_detections):
            for j, (cam_x, cam_y, cam_w, cam_h) in enumerate(camera_detections):
                # Check if lidar point is inside the bounding box
                if (
                    cam_x <= lidar_x <= cam_x + cam_w
                    and cam_y <= lidar_y <= cam_y + cam_h
                ):
                    # Lidar point is inside the bounding box (minimal cost)
                    euclidean_distance = 0
                else:
                    # If outside, calculate the distance to the nearest edge of the bounding box
                    nearest_x = max(cam_x, min(lidar_x, cam_x + cam_w))
                    nearest_y = max(cam_y, min(lidar_y, cam_y + cam_h))
                    euclidean_distance = np.linalg.norm(
                        [lidar_x - nearest_x, lidar_y - nearest_y]
                    )

                # Add depth-based weighting to the cost
                cost_matrix[i, j] = euclidean_distance * (
                    1 + depth_weight * (lidar_depth)
                )

        return cost_matrix

    def h_matching(
        self,
        left_projected_lidar_pts,
        right_projected_lidar_pts,
        left_cam_dets,
        right_cam_dets,
        duplicate_threshold=0.1,
        cost_threshold=300.0,
    ):
        """Performs Hungarian matching for lidar points and bounding boxes."""

        # Perform matching for left camera (with bounding boxes)
        cost_matrix_left = self.cost_mtx_from_bbox(
            left_projected_lidar_pts, left_cam_dets, depth_weight=0.05
        )
        row_ind_left, col_ind_left = linear_sum_assignment(cost_matrix_left)

        # Perform matching for right camera (with bounding boxes)
        cost_matrix_right = self.cost_mtx_from_bbox(
            right_projected_lidar_pts, right_cam_dets, depth_weight=0.05
        )
        row_ind_right, col_ind_right = linear_sum_assignment(cost_matrix_right)

        # Resolve duplicates by comparing matched lidar points
        matched_lidar_left = [left_projected_lidar_pts[i] for i in row_ind_left]
        matched_lidar_right = [right_projected_lidar_pts[i] for i in row_ind_right]

        # Combine matches and check for duplicates
        all_matches = []
        for i, lidar_point_left in enumerate(matched_lidar_left):
            cost_left = cost_matrix_left[row_ind_left[i], col_ind_left[i]]
            if cost_left > cost_threshold:
                continue
            for j, lidar_point_right in enumerate(matched_lidar_right):
                cost_right = cost_matrix_right[row_ind_right[j], col_ind_right[j]]
                if cost_right > cost_threshold:
                    continue
                # If lidar points from both cameras are close, it's likely a duplicate
                if (
                    np.linalg.norm(
                        np.array(lidar_point_left[:2]) - np.array(lidar_point_right[:2])
                    )
                    < duplicate_threshold
                ):
                    # Compare costs and choose the better match

                    if cost_left < cost_right:
                        all_matches.append((lidar_point_left, "left"))
                    else:
                        all_matches.append((lidar_point_right, "right"))
                else:
                    all_matches.append((lidar_point_left, "left"))
                    all_matches.append((lidar_point_right, "right"))

        # 'all_matches' now contains lidar points with resolved duplicates, matched to bounding boxes
        return all_matches

    def filter_points_in_fov(self, projected_points, image_size):
        """Filters out points that are outside the image boundaries."""

        image_width, image_height = image_size
        # Create a boolean mask for points inside the image boundaries
        valid_mask = (
            (projected_points[:, 0] >= 0)
            & (projected_points[:, 0] < image_width)
            & (projected_points[:, 1] >= 0)
            & (projected_points[:, 1] < image_height)
        )

        # Return the indices of the valid points
        valid_indices = np.where(valid_mask)[0]

        return valid_indices

    # def save_image(self, left_img_, right_img_, rosbag_name):
    #     left_img_name = rosbag_name + "_" + str(self.saved_count) + ".png"
    #     cv2.imwrite(
    #         "/home/utfrdv/utfr_dv_ros2/src/perception/images_to_mp4_left/"
    #         + left_img_name,
    #         left_img_,
    #     )

    #     right_img_name = rosbag_name + "_" + str(self.saved_count) + ".png"
    #     cv2.imwrite(
    #         "/home/utfrdv/utfr_dv_ros2/src/perception/images_to_mp4_right/"
    #         + right_img_name,
    #         right_img_,
    #     )

    #     self.saved_count += 1

    # def resize_img(self, left_img_, right_img_, scale_percent):
    #     # Downsize image resolution/size

    #     right_width = int(right_img_.shape[1] * scale_percent / 100)
    #     right_height = int(right_img_.shape[0] * scale_percent / 100)
    #     right_dim = (right_width, right_height)

    #     # resize image
    #     frame_right = cv2.resize(right_img_, right_dim, interpolation=cv2.INTER_AREA)

    #     left_width = int(left_img_.shape[1] * scale_percent / 100)
    #     left_height = int(left_img_.shape[0] * scale_percent / 100)
    #     left_dim = (left_width, left_height)

    #     # resize image
    #     frame_left = cv2.resize(left_img_, left_dim, interpolation=cv2.INTER_AREA)

    #     return frame_left, frame_right

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


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    executor = MultiThreadedExecutor(
        num_threads=6
    )  # Adjust the number of threads as needed
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
