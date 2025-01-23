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
import time

# import xgboost
from ultralytics import YOLO

# ROS2 Requirements
import rclpy
from rclpy.node import Node

# from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

# from rclpy.callback_groups import ReentrantCallbackGroup
from cv_bridge import CvBridge, CvBridgeError

# from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
import torch

# Message Requirements
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

# from std_msgs.msg import Bool
from utfr_msgs.msg import ConeDetections
from utfr_msgs.msg import Heartbeat
from utfr_msgs.msg import Cone
from utfr_msgs.msg import BoundingBox
from utfr_msgs.msg import PerceptionDebug
from utfr_msgs.msg import EgoState
from geometry_msgs.msg import PointStamped, Point


# Service Requirements
from std_srvs.srv import Trigger

# Import deep process functions
from perception.submodules.deep import deep_process

# from perception.submodules.deep import check_for_cuda
from perception.submodules.deep import labelColor


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.lidar_group = (
            MutuallyExclusiveCallbackGroup()
        )  # TODO: maybe change to reentrant
        self.cam_timer_group = MutuallyExclusiveCallbackGroup()
        self.preproc_timer_group = MutuallyExclusiveCallbackGroup()
        self.heartbeat_timer_group = MutuallyExclusiveCallbackGroup()

        self.loadParams()
        self.initVariables()
        self.initHeartbeat()
        self.initSubscribers()
        self.initPublishers()
        self.initServices()
        self.initConeTemplate()
        self.initTimers()

    #Jeff: Load configuration parameters from ROS2 parameter server, allowing flexible configuration without code changes
    #Key steps:
        #Declare default parameters
        #Retrieve parameter values
        #Convert some parameters to numpy arrays for computational efficiency
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
        self.declare_parameter("left_camera_topic", "/left_camera/images")
        self.declare_parameter("right_camera_topic", "/right_camera/images")
        self.declare_parameter("cone_detections_topic", "/perception/cone_detections")
        self.declare_parameter("heartbeat_topic", "/perception/heartbeat")
        self.declare_parameter("processed_lidar_topic", "/lidar_pipeline/clustered")
        self.declare_parameter("camera_capture_rate")
        self.declare_parameter("heartbeat_rate")
        self.declare_parameter("debug", True)
        self.declare_parameter("distortion", [0.0])
        self.declare_parameter("intrinsics", [0.0])
        self.declare_parameter("rotation", [0.0])
        self.declare_parameter("translation", [0.0])
        self.declare_parameter("save_pic", "False")
        self.declare_parameter("confidence", 0.70)
        self.declare_parameter("perception_debug_topic", "/perception/debug")
        self.declare_parameter("cone_heights", [0.0])
        self.declare_parameter("lidar_only_detection", False)
        self.declare_parameter("is_cuda_cv", True)
        self.declare_parameter("is_cuda_deep", True)

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
        self.camera_capture_rate_ = (
            self.get_parameter("camera_capture_rate").get_parameter_value().double_value
        )
        self.heartbeat_rate = (
            self.get_parameter("heartbeat_rate").get_parameter_value().double_value
        )
        self.debug_ = self.get_parameter("debug").get_parameter_value().bool_value
        self.distortion = (
            self.get_parameter("distortion").get_parameter_value().double_array_value
        )
        self.intrinsics = (
            self.get_parameter("intrinsics").get_parameter_value().double_array_value
        )
        self.rotation = (
            self.get_parameter("rotation").get_parameter_value().double_array_value
        )
        self.translation = (
            self.get_parameter("translation").get_parameter_value().double_array_value
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

        self.is_cuda_cv = (
            self.get_parameter("is_cuda_cv").get_parameter_value().bool_value
        )

        self.is_cuda_deep = (
            self.get_parameter("is_cuda_deep").get_parameter_value().bool_value
        )

        # reshape the arrays into numpy matrix form
        self.intrinsics = np.array(self.intrinsics).reshape(3, 3)
        self.rotation = np.array(self.rotation).reshape(3, 3)
        self.distortion = np.array(self.distortion)
        self.translation = np.array(self.translation)
        self.cone_heights = np.array(self.cone_heights)

    #Jeff: Set up internal variables, initialize camera connection, and prepare object detection model
    #Key steps:
        #Initialize image-related flags and variables
        #Create camera undistortion maps
        #Find and connect to camera
        #Load YOLO object detection model (with GPU support if available)
    def initVariables(self):
        # Initialize callback variables:
        self.new_cam_received_ = False
        self.first_img_arrived_ = False

        # Work
        self.img_ = None
        self.img_raw_ = None
        self.img_size = (1024, 576)  # should be in config yaml
        # 480 272 gives 30fps for this webcam and 1024 x 576 gives 20fps

        # Initialize image conversion bridge:
        self.bridge = CvBridge()

        # generate map for rectification
        self.mapx, self.mapy = cv2.initUndistortRectifyMap(
            self.intrinsics,
            self.distortion,
            None,
            self.intrinsics,
            self.img_size,
            cv2.CV_32FC1,
        )

        # TODO: index 0 but it could be something else, so do a check of all connected cameras
        self.cam_capture, index = self.find_camera()
        if self.cam_capture:
            while not self.cam_capture.isOpened():
                print("Attempting to connect to camera again")
                self.cam_capture = cv2.VideoCapture(index)
        else:
            raise RuntimeError("Unable to detect camera")

        if self.is_cuda_cv:
            self.mapx_gpu = cv2.cuda_GpuMat()
            self.mapx_gpu.upload(self.mapx)

            self.mapy_gpu = cv2.cuda_GpuMat()
            self.mapy_gpu.upload(self.mapy)

        # For CPU
        else:
            pass
            # self.mapx_gpu = self.mapx
            # self.mapy_gpu = self.mapy

        # create ultralytics model for inference
        if self.is_cuda_deep:
            file_name = "src/perception/perception/models/yolov8n_batched.engine"
        else:
            file_name = "src/perception/perception/models/yolov8n.pt"

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

        # create transform frame variables
        self.lidar_frame = "os_sensor"
        self.camera_frame = "camera"

        self.right_frame_id = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        print(self.tf_listener)

        if self.is_cuda_cv:
            self.img_gpu = cv2.cuda_GpuMat()

        self.previous_img_ = None
        self.lastCamCaptureTime = time.time()
        self.lastPreProcTime = time.time()

    def initSubscribers(self):
        """
        Initialize Subscribers

        processed_lidar_subscriber_ :
          msg: sensor_msgs::PointCloud2, topic:

        ego_state_subscriber_ :
            msg: utfr_msgs::EgoState, topic:
        """
        #self.create_subscription(msg_type, topic_name, callback, qos_profile, callback_group)

        self.processed_lidar_subscriber_ = self.create_subscription(
            PointCloud2,
            self.processed_lidar_topic,
            self.lidarCB,
            1,
            callback_group=self.lidar_group,
        )

        self.ego_state_subscriber_ = self.create_subscription(
            EgoState, "synced_ego_state", self.egoStateCB, 1
        )

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
        #Jeff: self.create_publisher(msg_type, topic_name, qos_profile)
        #1. cone detections publisher:
        #2. heartbeat publisher:
        #3. ego state publisher:
        #4. perception debug publisher:
        #5. lidar projection publisher:
        #6. lidar projection publisher matched:
        #7. cam processed publisher:
        self.cone_detections_publisher_ = self.create_publisher(
            ConeDetections, self.cone_detections_topic_, 1
        )

        self.heartbeat_publisher_ = self.create_publisher(
            Heartbeat, self.heartbeat_topic_, 1
        )

        self.ego_state_publisher = self.create_publisher(
            EgoState, "perception_ego_state", 1
        )

        if self.debug_:
            self.undistorted_publisher_ = self.create_publisher(
                Image, "/perception/debug_undistorted", 1
            )

            self.perception_debug_publisher_ = self.create_publisher(
                PerceptionDebug, self.perception_debug_topic_, 1
            )

            self.lidar_projection_publisher_ = self.create_publisher(
                PerceptionDebug, "/perception/lidar_projection", 1
            )

            self.lidar_projection_publisher_matched_ = self.create_publisher(
                PerceptionDebug, "/perception/lidar_projection_matched", 1
            )

        # TODO: change subscriber on heartbeat
        self.cam_processed_publisher_ = self.create_publisher(
            Heartbeat, "/perception/cam_processed", 1
        )

    def initServices(self):
        """
        Initialize Services
        """
        pass

    def initTimers(self):
        """
        Initialize main update timer for timerCB.
        """
        #Jeff: self.create_timer(timer_period_sec, callback, callback_group=None)
        #1. camera capture timer:
        #2. heartbeat timer:
        #3. preproc timer:
        self.cameraCaptureTimer_ = self.create_timer(
            self.camera_capture_rate_ / 1000,
            self.cameraCaptureTimerCB,
            callback_group=self.cam_timer_group,
        )

        self.heartbeatTimer_ = self.create_timer(
            self.heartbeat_rate / 1000,
            self.heartbeatCB,
            callback_group=self.heartbeat_timer_group,
        )

        self.preProcTimer_ = self.create_timer(
            0.001, self.preProcCB, callback_group=self.preproc_timer_group
        )

        # Call the timer_  to prevent unused variable warnings
        self.cameraCaptureTimer_
        self.heartbeatTimer_
        self.preProcTimer_

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
        self.heartbeat_.update_rate = self.camera_capture_rate_
        # TODO: Put these in a config/yaml file and read from there
        self.MIN_HEIGHT = 720
        self.MIN_WIDTH = 1020
        self.MAX_FRAME_RATE = 1.0
        if self.first_img_arrived_ is True:
            self.previous_img_ = self.img_

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

    def lidarCB(self, msg):
        """
        Callback function for processed lidar point cloud
        """
        print("Processed Lidar")

        startTime = time.time()

        if self.img_ is not None and self.img_.size != 0:
            self.deep_and_matching(msg)

        print("Deep and matching time: " + str(time.time() - startTime))

    def egoStateCB(self, msg):
        """
        Callback function for ego state subscriber
        """
        self.ego_state = msg

    def find_camera(self):
        for index in range(0, 10):  # Check first 10 indices
            cam_capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
            if cam_capture.isOpened():
                print(f"Connected to camera at index {index}")
                cam_capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size[0])
                cam_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size[1])
                cam_capture.set(cv2.CAP_PROP_FPS, 30)
                return cam_capture, index
            cam_capture.release()
        print("No cameras found.")
        return None, -1

    def process(self, img_):
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
        (bounding_boxes, classes, scores) = deep_process(
            self.model, img_, self.confidence_
        )
        end = (
            self.get_clock().now().seconds_nanoseconds()[0]
            + self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        )
        print("deep process time: ", end - start)

        return (bounding_boxes, classes, scores)

    def preProcCB(self):
        # print("preproc ")
        if not self.new_cam_received_:
            return

        fullyStartTime = time.perf_counter()

        # Color formart change:
        # img_ = cv2.cvtColor(self.img_raw_, cv2.COLOR_BGR2GRAY)

        # self.publish_undistorted(self.img_raw_)

        try:
            # TODO: set for publishing images
            self.img_stamp_ = self.get_clock().now().to_msg()

            # self.left_frame_id = msg.header.frame_id
            # self.new_left_img_recieved_ = True

            if self.is_cuda_cv:
                startTime = time.perf_counter()
                self.img_gpu.upload(self.img_raw_)
                print("upload time: " + str(time.perf_counter() - startTime))

                startTime = time.perf_counter()
                # rectification
                img_ = cv2.cuda.remap(
                    self.img_gpu,
                    self.mapx_gpu,
                    self.mapy_gpu,
                    interpolation=cv2.INTER_NEAREST,
                )
                print("remap time: " + str(time.perf_counter() - startTime))

                startTime = time.perf_counter()
                self.img_ = img_.download()
                print("download time: " + str(time.perf_counter() - startTime))

            else:
                startTime = time.perf_counter()
                # rectification CPU
                self.img_ = cv2.remap(
                    self.img_raw_,
                    self.mapx,
                    self.mapy,
                    interpolation=cv2.INTER_NEAREST,
                )
                # print("remap cpu time: " + str(time.perf_counter() - startTime))

            # compare time it takes to resize on the GPU vs time you save in preprocessing in the YOLO model
            # img_ = cv2.resize(img_, (640, 640))

            if not self.first_img_arrived_:
                self.previous_img_ = self.img_
                self.first_img_arrived_ = True

            self.new_cam_received_ = False

            if self.debug_:
                self.publish_undistorted(self.img_)

                print("Pre Proc Hz: ", 1 / (time.time() - self.lastPreProcTime))
                self.lastPreProcTime = time.time()

        except CvBridgeError as e:
            exception = "Perception::TimerCB: " + str(e)
            self.get_logger().error(exception)
        except Exception as e:
            exception = "Perception::TimerCB: " + str(e)
            self.get_logger().error(exception)

        # print("TOTAL TIMERCB TIME", time.perf_counter() - fullyStartTime)
        # print("-----------")

    def heartbeatCB(self):
        # publish the heartbeat
        cam_process = Heartbeat()
        cam_process.header.stamp = self.get_clock().now().to_msg()
        self.cam_processed_publisher_.publish(cam_process)
        self.publishHeartbeat()

    def cameraCaptureTimerCB(self):
        """
        Runs to capture raw images from the webcam and place into img_ buffer
        """
        fullyStartTime = time.perf_counter()
        # TODO: check if unplugging camera will crash this. Also, We should be trying to reconnect to webcam if it disconnects
        ret, img_ = self.cam_capture.read()
        if not ret or img_.size == None:
            print("Error: Could not read frame.")
            return

        self.img_raw_ = img_
        self.new_cam_received_ = True
        if self.debug_:
            print("Capture time: ", time.perf_counter() - fullyStartTime)
            print("Capture Hz: ", 1 / (time.time() - self.lastCamCaptureTime))
            self.lastCamCaptureTime = time.time()

    def deep_and_matching(self, lidar_msg):
        if self.lidar_only_detection == False:
            try:
                # tf from lidar to cam
                tf_lidar_to_cam = self.tf_buffer.lookup_transform(
                    self.camera_frame,
                    "ground",
                    time=rclpy.time.Time(seconds=0),
                    timeout=rclpy.time.Duration(seconds=0.1),
                )

                tf_cam_to_lidar = self.tf_buffer.lookup_transform(
                    "ground",
                    self.camera_frame,
                    time=rclpy.time.Time(seconds=0),
                    timeout=rclpy.time.Duration(seconds=0.1),
                )

            except TransformException as ex:
                self.get_logger().info(f"{ex}")
                return

            frame = self.img_
            (results_left, classes_left, scores_left) = self.process(frame)

            # Extract point cloud data
            lidar_point_cloud_data = point_cloud2.read_points_numpy(
                lidar_msg, field_names=["x", "y", "z"], skip_nans=True
            )

            # transform lidar detections to cam frame
            lidar_det_cam_frame = self.transform_det_lidar(
                lidar_point_cloud_data, tf_lidar_to_cam
            )

            # Transform 3D camera frame to 3D camera optical frame (axis swap)
            # x in the left cam frame = z in the transformation frame
            lidar_det_cam_frame = self.tf_cam_axis_swap(lidar_det_cam_frame)

            # 3D optical frame to 2D pixel coordinates projection with depth
            projected_pts = self.point_3d_to_image(
                lidar_det_cam_frame, self.intrinsics
            )  # list of u, v, depth
            projected_pts[:, :2] = np.round(projected_pts[:, :2])

            valid_idx = self.filter_points_in_fov(projected_pts, self.img_size)

            filtered_projected_pts = projected_pts[valid_idx]

            # Hungarian algorithm for matching
            matched, left_class = self.h_matching(
                left_projected_lidar_pts=filtered_projected_pts,
                left_cam_dets=results_left,
                classes_left=classes_left,
                duplicate_threshold=0.1,
                cost_threshold=1000.0,
                depth_weight=0.5,
                alpha=0.5,
            )

            matched_cam_frame = []
            for match in matched:
                matched_cam_frame.append(self.image_to_3d_point(match, self.intrinsics))

            matched_cam_frame = self.tf_cam_axis_swap_inv(np.array(matched_cam_frame))

            matched_lidar_frame = self.transform_det_lidar(
                matched_cam_frame, tf_cam_to_lidar
            )

            cone_detections = ConeDetections()
            # TODO: clean this up and just use the same enum format as in the cone msg template
            for i, point in enumerate(matched_lidar_frame):
                cone = Cone()
                cone.pos.x = float(point[0])
                cone.pos.y = float(point[1])
                cone.pos.z = float(point[2])

                # print(point.pt[0], point.pt[1], point.pt[2])

                if left_class[i] == "blue_cone":  # blue
                    cone.type = 1
                    cone_detections.left_cones.append(cone)
                elif left_class[i] == "yellow_cone":  # yellow
                    cone.type = 2
                    cone_detections.right_cones.append(cone)
                elif left_class[i] == "small_orange_cone":
                    cone.type = 3
                    cone_detections.small_orange_cones.append(cone)
                elif left_class[i] == "large_orange_cone":
                    cone.type = 4
                    cone_detections.large_orange_cones.append(cone)
                else:  # unknown cones cone template type == 0
                    cone.type = 0
                    cone_detections.unknown_cones.append(cone)

            cone_detections.header.stamp = self.get_clock().now().to_msg()
            cone_detections.header.frame_id = "ground"
            self.cone_detections_publisher_.publish(cone_detections)

            # Cam det bounding boxes and lidar det points on image
            if self.debug_:
                self.publish_2d_projected_det(
                    projected_pts=filtered_projected_pts,
                    stamp=self.img_stamp_,
                )
                self.publish_2d_projected_det_matched(
                    projected_pts=matched, stamp=self.img_stamp_
                )
                self.displayBoundingBox(
                    results_left=results_left,
                    classes_left=classes_left,
                    scores_left=scores_left,
                    img_stamp=self.img_stamp_,
                )

        else:
            self.publish_cone_dets_lidar(lidar_msg=lidar_msg)

        self.ego_state_publisher.publish(self.ego_state)

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

    # Helper functions
    def publish_cone_dets_lidar(self, lidar_msg):
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

    def publish_2d_projected_det(self, projected_pts, stamp):
        perception_debug_msg = PerceptionDebug()
        perception_debug_msg.header.stamp = stamp
        perception_debug_msg.header.frame_id = "raw"

        for point in projected_pts:
            bounding_box = BoundingBox()
            bounding_box.x = int(point[0])
            bounding_box.y = int(point[1])
            bounding_box.width = 20
            bounding_box.height = 20
            perception_debug_msg.left.append(bounding_box)

        self.lidar_projection_publisher_.publish(perception_debug_msg)

    def publish_2d_projected_det_matched(
        self,
        projected_pts,
        stamp,
    ):
        perception_debug_msg = PerceptionDebug()
        perception_debug_msg.header.stamp = stamp
        perception_debug_msg.header.frame_id = "matched"

        for point in projected_pts:
            bounding_box = BoundingBox()
            bounding_box.x = int(point[0])
            bounding_box.y = int(point[1])
            bounding_box.width = 20
            bounding_box.height = 20
            perception_debug_msg.left.append(bounding_box)

        self.lidar_projection_publisher_matched_.publish(perception_debug_msg)

    def publish_undistorted(self, frame):
        """Print undistorted left and right images"""
        # to do calibration, set self.bridge.cv2_to_imgmsg(frame, "mono8")
        self.undistorted_publisher_.publish(self.bridge.cv2_to_imgmsg(frame))

    def displayBoundingBox(self, results_left, classes_left, scores_left, img_stamp):
        # Bounding boxes from deep learning model
        if len(results_left) != 0:
            perception_debug_msg = PerceptionDebug()
            perception_debug_msg.header.stamp = img_stamp
            for i in range(len(results_left)):
                bounding_box = BoundingBox()
                bounding_box.x = int(results_left[i][0])
                bounding_box.y = int(results_left[i][1])
                bounding_box.width = int(results_left[i][2])
                bounding_box.height = int(results_left[i][3])
                bounding_box.type = labelColor(classes_left[i])
                bounding_box.score = scores_left[i]
                perception_debug_msg.left.append(bounding_box)

            self.perception_debug_publisher_.publish(perception_debug_msg)

    def tf_cam_axis_swap(self, points):
        return np.column_stack(
            [-points[:, 1], -points[:, 2], points[:, 0]]  # -y  # -z  # x
        )

    def tf_cam_axis_swap_inv(self, points):
        if len(points) == 0:
            return np.array([])
        return np.column_stack(
            [points[:, 2], -points[:, 0], -points[:, 1]]  # x  # -z  # -y
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

    def image_to_3d_point(self, image_point, camera_matrix):
        # Add a homogeneous coordinate to the 2D image point
        # TODO: put Z instead of 1??
        image_point_homogeneous = np.array([image_point[0], image_point[1], 1])

        # Invert the camera matrix
        inv_camera_matrix = np.linalg.inv(camera_matrix)

        # Transform to normalized camera coordinates
        normalized_coords = np.dot(inv_camera_matrix, image_point_homogeneous)

        # 3D point with unit depth
        point_3d = normalized_coords / normalized_coords[2]

        # 3D point multiplied by monocular depth
        point_3d *= image_point[2]

        return point_3d

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
        self, projected_lidar_detections, camera_detections, depth_factor
    ):
        cost_matrix = np.zeros(
            (len(projected_lidar_detections), len(camera_detections))
        )
        for i, (lidar_x, lidar_y, lidar_depth) in enumerate(projected_lidar_detections):
            for j, (cam_x, cam_y, cam_w, cam_h) in enumerate(camera_detections):
                # Check if lidar point is inside the bounding box
                if (
                    cam_x - 10 <= lidar_x <= cam_x + cam_w + 10
                    and cam_y - 10 <= lidar_y <= cam_y + cam_h + 10
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
                    1 + depth_factor * (lidar_depth)
                )

        return cost_matrix

    def h_matching(
        self,
        left_projected_lidar_pts,
        left_cam_dets,
        classes_left,
        duplicate_threshold,
        cost_threshold,
        depth_weight,
        alpha,
    ):
        """Performs Hungarian matching for lidar points and bounding boxes."""

        # Perform matching for left camera
        cost_matrix_left = self.cost_mtx_from_bbox(
            left_projected_lidar_pts, left_cam_dets, depth_factor=depth_weight
        )
        row_ind_left, col_ind_left = linear_sum_assignment(cost_matrix_left)

        matched_lidar_left = [left_projected_lidar_pts[i] for i in row_ind_left]

        # Store results after Hungarian matching
        left_matches, left_classes = [], []

        for i, lidar_point_left in enumerate(matched_lidar_left):
            cost_left = cost_matrix_left[row_ind_left[i], col_ind_left[i]]
            if cost_left > cost_threshold:
                continue
            left_matches.append(lidar_point_left)
            left_classes.append(classes_left[col_ind_left[i]])

        return left_matches, left_classes

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
        elif self.img_ is None:
            return 0  # UNINITIALIZED
        # Check if frame is too small
        elif (
            self.img_.shape[0] < self.MIN_HEIGHT or self.img_.shape[1] < self.MIN_WIDTH
        ):
            return 2  # FATAL

        # TODO - fix fps, frame change cases

        # Check if FPS is too low
        # elif self.left_timestamp_ - self.right_timestamp_ > self.MAX_FRAME_RATE:
        #   return 2 #FATAL
        # elif self.right_timestamp_ - self.left_timestamp_ > self.MAX_FRAME_RATE:
        #   return 2 #FATAL
        elif self.hasVisualArtifacts(self.img_):
            return 2  # FATAL
        # Check if frame has changed significantly (stores a boolean)
        frame_changed = self.frameChanged(self.img_, self.previous_img_)
        # Update previous frames
        self.previous_img_ = self.img_
        if frame_changed == False:
            return 0  # UNINITIALIZED
        else:
            return 1  # ACTIVE


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
