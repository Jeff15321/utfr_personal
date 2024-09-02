"""

██████  ██    ██ ██████  ██   ██
██   ██ ██    ██      ██ ██   ██
██   ██ ██    ██  █████  ███████
██   ██  ██  ██  ██           ██
██████    ████   ███████      ██

* file: deep.py
* auth: Mustafa Khan
* desc: deep learning inference file
"""

import cv2
import time
import numpy as np
import onnxruntime as ort
import os
import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped, Point
from tf2_geometry_msgs import PointStamped as TFPointStamped
import tf2_geometry_msgs
import numpy as np
from tf2_ros import TransformException

# cuda = False


def check_for_cuda():
    """
    A check that is specific to the DJI Manifold 2-G's Jetson TX2
    """
    # global cuda

    # NOTE: This is a bit superficial. There might be a better way to do this check.
    # 11.5 for dv computer, 11.4 for jetson agx xavier
    output = os.popen("nvcc --version").read()
    if "cuda_12.2" in output:
        return True
    else:
        return False


def letterbox(
    im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleup=True, stride=32
):
    """
    Resize and pad an image to a specified shape while maintaining the original aspect ratio.
    """
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(
        im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )  # add border
    return im, r, (dw, dh)


def deep_process(frame, translation, intrinsics, session, confidence, visualize=False):
    """
    Applies object detection on each frame from a camera using a deep learning model.
    """

    # if frame == []:
    # return [], [], [], []

    # Define class names and colors
    names = [
        "blue_cone",
        "large_orange_cone",
        "orange_cone",
        "yellow_cone",
        "unknown_cone",
    ]
    colors = {
        name: [np.random.randint(0, 255) for _ in range(3)]
        for i, name in enumerate(names)
    }

    # Apply letterbox
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    image = img.copy()
    image, ratio, dwdh = letterbox(image, auto=False)
    image = image.transpose((2, 0, 1))
    image = np.expand_dims(image, 0)
    image = np.ascontiguousarray(image)

    # Normalize input image
    im = image.astype(np.float32) / 255.0

    # Run inference using onnxruntime
    outname = [i.name for i in session.get_outputs()]
    inname = [i.name for i in session.get_inputs()]
    inp = {inname[0]: im}
    outputs = session.run(outname, inp)[0]

    # Process outputs
    im_copy = img.copy()
    im_copy = cv2.cvtColor(im_copy, cv2.COLOR_BGR2RGB)
    ori_images = [im_copy]
    bounding_boxes = []
    classes = []
    scores = []

    for output in outputs:
        x0, y0, x1, y1, cls_id, score = output[1:7]

        if score < confidence:
            cls_id = 0  # unknown random cone

        image = ori_images[0]  # Assuming batch size of 1
        box = np.array([x0, y0, x1, y1])
        box -= np.array(dwdh * 2)
        box /= ratio
        box = box.round().astype(np.int32).tolist()
        cls_id = int(cls_id)
        score = round(float(score), 3)
        name = names[cls_id]

        if box[0] < 10 or box[1] < 10:
            continue

        box[2] = abs(box[0] - box[2])
        box[3] = abs(box[1] - box[3])

        bounding_boxes.append(box)
        classes.append(name)
        scores.append(score)

        if visualize:
            color = colors[name]
            label = f"{name} {score}"
            x0, y0, w, h = box
            cv2.rectangle(image, (x0, y0), (x0 + w, y0 + h), color, 2)
            cv2.putText(
                image,
                label,
                (x0, y0 - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                [225, 255, 255],
                thickness=2,
            )

    return bounding_boxes, classes, scores, image


def bounding_boxes_to_cone_detections(
    bounding_boxes,
    classes,
    intrinsics,
    cone_heights,
):
    coneDetections = np.zeros((np.shape(bounding_boxes)[0], 4))
    f = intrinsics[0][0]  # focal length in pixels from camera matrix
    i = 0
    skip_counter = 0
    for x, y, w, h in bounding_boxes:
        if y >= 930 and w <= 7 or h <= 7:
            skip_counter += 1
            continue
        color = labelColor(classes[i])  # int for color

        # 3.73mm height of sensor, 1080 px height of image
        depth_mm = find_depth_mono_tri(3.73, h, f, cone_heights[color], 1080)
        depth = depth_mm / 1000  # convert mm to metres

        # 3d coordinate mapping

        (
            coneDetections[i][0],
            coneDetections[i][1],
            coneDetections[i][2],
        ) = image_to_3d_point([(x + int(w / 2)), y + int(h / 2)], intrinsics, depth)

        coneDetections[i][3] = color

        i += 1
    if skip_counter == 0:
        return coneDetections
    else:
        return coneDetections[:-skip_counter]


def image_to_3d_point(image_point, camera_matrix, depth):
    # Add a homogeneous coordinate to the 2D image point
    image_point_homogeneous = np.array([image_point[0], image_point[1], 1])

    # Invert the camera matrix
    inv_camera_matrix = np.linalg.inv(camera_matrix)

    # Transform to normalized camera coordinates
    normalized_coords = np.dot(inv_camera_matrix, image_point_homogeneous)

    # 3D point with unit depth
    point_3d = normalized_coords / normalized_coords[2]

    # 3D point multiplied by monocular depth
    point_3d *= depth
    point_3d /= 250

    return point_3d


def find_depth_mono_tri(
    vertical_mm, height_bound_box, focal_length, height_cone, image_height_px
):
    """
    function to find the monocular depth using similar triangles
    focal_length / height_box_mm = depth / height_cone
    get height_cone from fsg rules
    """
    # find the height of the bounding box first using ratio of height
    # of bounding box in px and total image height in px multiplied by
    # the total vertical height of the sensor in mm
    height_box_mm = (height_bound_box / image_height_px) * vertical_mm
    # calculate depth in mm using similar triangles
    depth = height_cone * (focal_length / height_box_mm)
    return depth


def crop_image(fX, fY, fW, fH, image_in):
    image_out = image_in[fY : fY + fH, fX : fX + fW]
    return image_out


def labelColor(class_string):
    # names = ['blue_cone', 'large_orange_cone', 'orange_cone', 'unknown_cone', 'yellow_cone']
    if class_string == "unknown_cone":
        return 0
    elif class_string == "blue_cone":
        return 1
    elif class_string == "yellow_cone":
        return 2
    elif class_string == "orange_cone":
        return 3
    elif class_string == "large_orange_cone":
        return 4
    else:
        return 0


def transform_det_lidar(detections, transform):
    transformed_points = []
    for point in detections:
        # Create a PointStamped message
        p = Point()
        p.x = point[0]
        p.y = point[1]
        p.z = point[2]

        try:
            p_tf = tf2_geometry_msgs.do_transform_point(
                PointStamped(point=p), transform
            ).point
            transformed_points.append([p_tf.x, p_tf.y, p_tf.z, point[3]])

        except TransformException as ex:
            print(ex)
            return
    return np.array(transformed_points)
