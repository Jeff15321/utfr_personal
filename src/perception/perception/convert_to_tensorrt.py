from ultralytics import YOLO
import argparse
import os

if __name__ == "__main__":
    # Define the relative directory path where the model will be saved
    export_dir = "src/perception/perception/models"

    # Create the directory if it doesn't exist
    if not os.path.exists(export_dir):
        os.makedirs(export_dir)

    # Load the YOLO model
    model = YOLO("yolov8n.pt")

    # Export the model to TensorRT format in the specified directory
    model.export(
        format="engine",
        int8=True,
        simplify=True,
        batch=4,
        dynamic=True,
        device=0,
        path=os.path.join(export_dir, "yolov8n_normal.engine"),
    )

    print(f"Model exported to {os.path.join(export_dir, 'yolov8n_normal.engine')}")
