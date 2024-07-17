import cv2
dispW=640
dispH=480
flip=2
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
cam=cv2.VideoCapture(camSet)
outVid=cv2.VideoWriter('/home/vit/drone/myCam.avi', cv2.VideoWriter_fourcc(*'XVID'),21,(dispW, dispH))
while True:
    ret, frame= cam.read()
    cv2.imshow('nanoCam', frame)
    outVid.write(frame)
    if cv2.waitKey(1)==ord('q'):
        break
cam.release()
outVid.release()
cv2.destroyAllWindows()