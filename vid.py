import cv2
import time
dispW=640
dispH=480
flip=2
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
cam=cv2.VideoCapture(camSet)
outVid=cv2.VideoWriter('/home/vit/drone/myCam.avi', cv2.VideoWriter_fourcc(*'XVID'),21,(dispW, dispH))
i = 0
total = 0
start = time.time()
while True:
    ret, frame= cam.read()
    cv2.imshow('nanoCam', frame)
    outVid.write(frame)
    #cv2.imwrite(f'./img/{i}.jpg', frame)
    i+=1
    if cv2.waitKey(1)==ord('q'):
        break
end = time.time()
diff = end-start
cam.release()
outVid.release()
cv2.destroyAllWindows()
print("Total time: ", diff)
