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

# cuda = False


def check_for_cuda():
    """
    A check that is specific to the DJI Manifold 2-G's Jetson TX2
    """
    # global cuda

    # NOTE: This is a bit superficial. There might be a better way to do this check.
    # 11.5 for dv computer, 11.4 for jetson agx xavier
    output = os.popen("nvcc --version").read()
    if "cuda_11.5" in output:
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

    # Define class names and colors
    names = [
        "blue_cone",
        "large_orange_cone",
        "orange_cone",
        "unknown_cone",
        "yellow_cone",
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

    for i, (batch_id, x0, y0, x1, y1, cls_id, score) in enumerate(outputs):
        if score < confidence:
            # if it is less than the config confidence value, than it is unknown random cone
            cls_id = 0
        image = ori_images[int(batch_id)]
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

        # TODO: Modify bounding_box datastructure to include class and scoring information. Discuss with downstream.
        bounding_boxes.append(box)
        classes.append(name)

        if visualize == True:
            color = colors[name]
            name += " " + str(score)
            x0, y0, x1, y1 = box
            cv2.rectangle(image, (x0, y0), (x1, y1), color, 2)
            cv2.putText(
                image,
                name,
                (box[0], box[1] - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                [225, 255, 255],
                thickness=2,
            )

    return bounding_boxes, classes, image


def bounding_boxes_to_cone_detections(
    left_bounding_boxes,
    left_classes,
    right_bounding_boxes,
    right_classes,
    translation,
    intrinsics_left,
    left_img_,
    right_img_,
):
    coneDetections = np.zeros((np.shape(left_bounding_boxes)[0], 4))
    baseline = abs(translation[0] / 10)  # distance between cameras
    f = intrinsics_left[0][0]  # focal length in pixels from camera matrix
    i = 0
    left_width_ = int(left_img_.shape[1])
    leftright_frame_start = time.time()
    matched_array = []
    for x, y, w, h in left_bounding_boxes:
        color = labelColor(left_classes[i])  # int for color

        right_x, right_y, index = find_cone_other_frame_closest(
            x, y, right_bounding_boxes, left_classes[i], right_classes, matched_array
        )
        matched_array.append(index)

        disparity = abs(right_x - x)

        depth = find_depth(
            (right_x, right_y), (x, y), right_img_, left_img_, baseline, f
        )

        # 3d coordinate mapping

        f_u = intrinsics_left[0][0]
        f_v = intrinsics_left[1][1]
        c_u = int((np.shape(left_img_)[1]) / 2)
        c_v = int((np.shape(left_img_)[0]) / 2)

        (
            coneDetections[i][0],
            coneDetections[i][1],
            coneDetections[i][2],
        ) = project3DPoints(
            ((x + int(w / 2)), y + int(h / 2)),
            f_u,
            f_v,
            c_u,
            c_v,
            disparity,
            baseline,
            depth,
        )

        coneDetections[i][3] = color

        i += 1
    return coneDetections, left_img_, right_img_


def project3DPoints(pointLeft, f_u, f_v, c_u, c_v, disparity, baseline, depth):
    u_l, v_l = pointLeft[0], pointLeft[1]

    # coordinates in [m]

    x = ((baseline / disparity) * (u_l - c_u)) / 100
    y = ((baseline / disparity) * (f_u / f_v) * (v_l - c_v)) / 100
    z = depth / 100

    return (x, y, z)


def find_cone_other_frame(
    initialFrame, frameWidth, fX, fY, fW, fH, uncertainty, otherFrame
):
    search_area = crop_image(
        0, fY - uncertainty, frameWidth, fH + 2 * uncertainty, otherFrame
    )
    template_im = crop_image(fX, fY, fW, fH, initialFrame)
    result = cv2.matchTemplate(search_area, template_im, 0)
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
    top_left = min_loc
    result_x = top_left[0]
    result_y = top_left[1] + fY - uncertainty
    # top left coordinates of other frame, x, y
    return result_x, result_y


def find_cone_other_frame_closest(
    left_cone_x, left_cone_y, right_detections, left_class, right_classes, matched_array
):
    curr_min_dist = 1000
    curr_min_detection_x = 0
    curr_min_detection_y = 0
    curr_detection = 0
    i = 0
    for x, y, w, h in right_detections:
        if (
            (euclidean_distance(left_cone_x, x, left_cone_y, y)) < curr_min_dist
            and left_class == right_classes[i]
            and i not in matched_array
            and x >= left_cone_x
        ):
            curr_min_detection_x = x
            curr_min_detection_y = y
            curr_min_dist = euclidean_distance(left_cone_x, x, left_cone_y, y)
            curr_detection = i
        i += 1

    return curr_min_detection_x, curr_min_detection_y, curr_detection


def find_depth(right_point, left_point, frame_right, frame_left, baseline, f_pixel):
    # Convert focal length, f, from [mm] to [pixel]
    height_right, width_right, depth_right = frame_right.shape
    height_left, width_left, depth_left = frame_left.shape

    x_right = right_point[0]
    x_left = left_point[0]

    # Calculate the disparity
    disparity = x_left - x_right  # Displacement between left and right frames [pixels]

    # Calculate depth, z
    zDepth = abs((baseline * f_pixel) / disparity)  # Depth in [cm]

    return zDepth


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


def euclidean_distance(x1, x2, y1, y2):
    return np.sqrt(abs(x1 - x2) ** 2 + (2 * abs(y1 - y2) ** 2))
