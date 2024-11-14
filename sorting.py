import cv2

# Draws bounding circle (ball outline), returns center[] and radius
def drawOutlineCircle(frame):
	height, width = frame.shape[:2]
	centerX = width//2
	centerY = height//2
	center = (centerX, centerY)
	radius = 100
	color = (255, 255, 255)
	thickness = 2
	cv2.circle(frame, center, radius, color, thickness) 
	return [center, radius]

cap = cv2.VideoCapture(0)

cv2.namedWindow("window")


while True:
	ret, frame = cap.read()
	dims = drawOutlineCircle(frame)
	cv2.imshow("window", frame)

	if cv2.waitKey(1) & 0xFF==27:
		break


cv2.destroyAllWindows()
cap.release()
