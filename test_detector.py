import cv2
import numpy as np

from turret_control.target_detector import detect_concentric_target


# Create a blank white image.
image = np.full((720, 1280, 3), 255, dtype=np.uint8)

# Define target centre and radii.
center = (640, 360)
radii = [40, 80, 120]

# Draw the three concentric target regions.
cv2.circle(image, center, radii[2], (0, 0, 0), -1)
cv2.circle(image, center, radii[1], (255, 255, 255), -1)
cv2.circle(image, center, radii[0], (0, 0, 0), -1)

# Run target detection.
result = detect_concentric_target(image)

if result is None:
    print("Target not detected.")
else:
    print("Target detected!")
    print("Centre:", result["center"])
    print("Radii:", result["radii"])
    print("Score:", result["score"])

    detected_center = tuple(
        int(round(value)) for value in result["center"]
    )

    cv2.circle(image, detected_center, 5, (0, 0, 255), -1)

    cv2.imshow("Target Detection Test", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
