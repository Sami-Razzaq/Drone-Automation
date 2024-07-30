from ultralytics import YOLO
import cv2
# def main():
# Class 4 = airplane
# model = YOLO('Trained Models/drone_yolov8n.pt')
# model = YOLO('Trained Models/train3/weights/epoch30.pt')
model = YOLO('drone5_yolov8n.pt')
# model = YOLO('runs/detect/train3/weights/epoch75.pt')

# results = model.predict("drone.mp4", show=True, conf=0.7)

results = model.predict("./pics", conf=0.5)

for result in results:
    result.show()
    # print(result.boxes.xywh)
    # b = (result.boxes.xywh)
# b[0][0] gives x

# if __name__ == '__main__':
#     main()

# cap = cv2.VideoCapture("drone.mp4"/)

# # Loop through the video frames
# while cap.isOpened():
#     # Read a frame from the video
#     success, frame = cap.read()

#     if success:
#         # Run YOLOv8 inference on the frame
#         results = model(frame)

#         # Visualize the results on the frame
#         annotated_frame = results[0].plot(conf=False)

#         # Display the annotated frame
#         cv2.imshow("YOLOv8 Inference", annotated_frame)

#         # Break the loop if 'q' is pressed
#         if cv2.waitKey(1) & 0xFF == ord("q"):
#             break
#     else:
#         # Break the loop if the end of the video is reached
#         break

# # Release the video capture object and close the display window
# cap.release()
# cv2.destroyAllWindows()

# # Change Classes Labels
# import os

# def update_class_number(file_path, old_class, new_class):
#     with open(file_path, 'r') as file:
#         lines = file.readlines()

#     with open(file_path, 'w') as file:
#         for line in lines:
#             parts = line.strip().split()
#             if parts[0] == str(old_class):
#                 parts[0] = str(new_class)
#             file.write(' '.join(parts) + '\n')

# def process_directory(directory_path, old_class, new_class):
#     for root, dirs, files in os.walk(directory_path):
#         for file in files:
#             if file.endswith('.txt'):
#                 print("Found Text file")
#                 file_path = os.path.join(root, file)
#                 update_class_number(file_path, old_class, new_class)

# directory_path = 'D:/Code/Drone/ML/drone_data_yolo_4/labels/train'  # Update this path to your labels directory
# old_class = 4
# new_class = 80
# process_directory(directory_path, old_class, new_class)