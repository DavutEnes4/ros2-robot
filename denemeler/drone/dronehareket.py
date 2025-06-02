from ultralytics import YOLO
import time

model = YOLO("best3.pt")

model.predict(source = "video3.mp4", show = True)

