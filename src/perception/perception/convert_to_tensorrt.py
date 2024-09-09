from ultralytics import YOLO
import argparse
import os


if __name__ == "__main__":
    model = YOLO("yolov8n.pt")
    model.export(format="engine", int8=True, simplify = True, batch=4, dynamic=True)

    # yolov8n_normal.engine is the normal yolov8 but in tensorrt with float32