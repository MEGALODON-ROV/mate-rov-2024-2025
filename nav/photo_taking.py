import cv2
import time
cam = cv2.VideoCapture(0)
picNum = 0
while True:
    ret,frame = cam.read()
    cv2.imshow('camera', frame)
    fileName = 'captured_image' + str(picNum) + '.jpg'
    cv2.imwrite(fileName, frame)
    picNum = picNum + 1
    time.sleep(1.5)

    if cv2.waitkey(1) == ord('q'):
        break

cam.release()
cv2.destoryAllWindows()