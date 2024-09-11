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
