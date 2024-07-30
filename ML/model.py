from ultralytics import YOLO
import cv2

model = YOLO('drone3_yolov8n.pt')

# results = model.predict("drone.mp4", show=True, conf=0.5)

results = model.predict("./pics", conf=0.5)

for result in results:
    result.show()
    # print(result.boxes.xywh)
    # b = (result.boxes.xywh)
# b[0][0] gives x

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