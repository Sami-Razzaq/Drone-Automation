from pdb import set_trace as st
from ultralytics import YOLO
import matplotlib.pyplot as plt
import cv2

def main():
    # Class 4 = airplane
    model = YOLO('yolov8n.pt')

    # Train the model
    model.train(
        task='detect',
        data='D:/Code/Drone/ML/drone.yaml',  # Path to the YAML file
        epochs=300,  # Number of epochs to train
        batch=4,   # Batch size
        imgsz=320,  # Image size
        pretrained=True,  # Use pretrained weights
        save=True,
        save_period=1,
        val=True,
        device=0,
    )

    # Save the trained model
    model.save('./drone5_yolov8n.pt')

    # results = model.predict("./", classes=[4])

    # for result in results:
    #     result.show()


if __name__ == '__main__':
    main()