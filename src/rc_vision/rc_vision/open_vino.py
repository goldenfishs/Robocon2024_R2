from ultralytics import YOLO

# Load a YOLOv8n PyTorch model
model = YOLO("/home/ubuntu/r2_algorithm/src/rc_vision/rc_vision/weights/weights2/best.pt")

# Export the model
model.export(format="openvino")  # creates 'yolov8n_openvino_model/'

