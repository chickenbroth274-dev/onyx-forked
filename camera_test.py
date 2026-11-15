import cv2
import time

camera = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L2)

if not camera.isOpened():
    print("Camera failed to open")
    exit()

frame_count = 0
start_time = time.perf_counter()
last_time = start_time

print("Testing camera capture. Press Ctrl+C to stop.")

try:
    while True:
        ret, frame = camera.read()

        if not ret:
            print("Frame capture failed")
            continue

        frame_count += 1
        now = time.perf_counter()

        if now - start_time >= 5:
            fps = frame_count / (now - start_time)
            interval = (now - last_time) * 1000

            print(
                f"FPS: {fps:.2f} | "
                f"Last interval: {interval:.1f} ms"
            )

            frame_count = 0
            start_time = now

        last_time = now

except KeyboardInterrupt:
    pass

finally:
    camera.release()
