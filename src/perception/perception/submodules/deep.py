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
import numpy as np
import onnxruntime as ort
import os
import time


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


def deep_process(model, frame_left, frame_right, confidence, visualize=False):
    """
    Applies object detection on each frame from a camera using a deep learning model.
    """
    left_float = frame_left
    right_float = frame_right

    batch = [left_float, right_float]

    start_time = time.time()
    output = model.predict(batch)

    end_time = time.time()

    print("runtime: ", end_time - start_time)

    left_output, right_output = output[0], output[1]

    outputs = [left_output, right_output]
    frames = [left_float, right_float]

    bounding_boxes = []
    classes = []
    scores = []

    start = time.time()

    for frame, output in zip(frames, outputs):
        bounding_boxes_local = []
        classes_local = []
        scores_local = []
        for box in output.boxes:
            # Get box coordinates, class and confidence
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls_id = int(box.cls)
            conf = float(box.conf)

            if conf < confidence:
                continue

            # Convert to [x, y, w, h] format
            x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)

            name = output.names[cls_id]
            score = round(conf, 3)

            bounding_boxes_local.append([x, y, w, h])
            classes_local.append(name)
            scores_local.append(score)

            if visualize:
                color = (0, 255, 0)  # Green color for bounding box
                label = f"{name} {score}"
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(
                    frame,
                    label,
                    (x, y - 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.75,
                    [225, 255, 255],
                    thickness=2,
                )

        bounding_boxes.append(bounding_boxes_local)
        classes.append(classes_local)
        scores.append(scores_local)

    end = time.time()
    print("for box deep process: ", end - start)

    return (
        bounding_boxes[0],
        classes[0],
        scores[0],
        bounding_boxes[1],
        classes[1],
        scores[1],
    )


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
