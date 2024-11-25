import cv2

cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Error: Cannot open camera")
else:
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot grab frame")
    else:
        print("Frame captured successfully")
cap.release()
