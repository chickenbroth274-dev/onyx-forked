import cv2

# Open DroidCam virtual camera
camera = cv2.VideoCapture(2)

if not camera.isOpened():
    print("ERROR: Could not open DroidCam.")
    exit()

print("DroidCam connected. Press Q to quit.")

while True:
    ret, frame = camera.read()

    if not ret:
        print("ERROR: Could not receive frame.")
        break

    cv2.imshow("DroidCam Phone Feed", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
