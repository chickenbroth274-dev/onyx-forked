"""ROS 2 node for threaded camera capture and target detection."""

import threading
import time

import cv2
import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

from cv_bridge import CvBridge

from turret_control.target_detector import detect_concentric_target


class CameraNode(Node):
    """Capture frames independently; always process the newest available frame."""

    def __init__(self):
        super().__init__('camera_node')

        # Camera configuration.
        self.declare_parameter('camera_device', '/dev/video2')

        camera_device = self.get_parameter(
            'camera_device'
        ).value

        self.camera = cv2.VideoCapture(
            camera_device,
            cv2.CAP_V4L2
        )

        if not self.camera.isOpened():
            raise RuntimeError(
                f'Could not open camera device {camera_device}'
            )

        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.camera.set(cv2.CAP_PROP_FPS, 15)
        self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.bridge = CvBridge()

        # Thread-safe storage for the latest camera frame.
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.frame_sequence = 0
        self.last_processed_sequence = -1

        # Capture-thread controls and measurements.
        self.stop_event = threading.Event()

        self.capture_time_total = 0.0
        self.capture_samples = 0

        self.capture_thread = threading.Thread(
            target=self.capture_loop,
            name='camera_capture_thread',
            daemon=True
        )

        # ROS publishers.
        self.detection_pub = self.create_publisher(
            Bool,
            '/turret/target_detected',
            10
        )

        self.center_pub = self.create_publisher(
            Float32MultiArray,
            '/turret/target_center',
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/turret/debug_image',
            qos_profile_sensor_data
        )

        # Performance measurements.
        self.measurement_count = 0

        self.timings = {
            'detection': 0.0,
            'resize': 0.0,
            'conversion': 0.0,
            'publish': 0.0,
            'total': 0.0
        }

        # Start camera capture independently.
        self.capture_thread.start()

        # Process the latest available frame at approximately 15 Hz.
        self.timer = self.create_timer(
            1.0 / 15.0,
            self.process_frame
        )

        width = self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)

        self.get_logger().info(
            f'Camera node started using {camera_device}'
        )

        self.get_logger().info(
            f'Camera resolution: {width}x{height}'
        )

        self.get_logger().info(
            'Threaded camera capture enabled'
        )

    def capture_loop(self):
        """Continuously capture frames in a background thread."""

        while not self.stop_event.is_set():

            start = time.perf_counter()

            ret, frame = self.camera.read()

            capture_time = time.perf_counter() - start

            if not ret:
                time.sleep(0.02)
                continue

            with self.frame_lock:

                # Replace the previous frame.
                # Old frames are not queued.
                self.latest_frame = frame
                self.frame_sequence += 1

                self.capture_time_total += capture_time
                self.capture_samples += 1

    def process_frame(self):
        """Process the latest frame and publish ROS messages."""

        frame_start = time.perf_counter()

        # Get the newest available frame.
        with self.frame_lock:

            if self.latest_frame is None:
                return

            if (
                self.frame_sequence
                == self.last_processed_sequence
            ):
                return

            # The capture thread replaces latest_frame with a new array; it
            # does not modify this one. Keeping the reference avoids a full
            # 1280x720 copy before detection.
            frame = self.latest_frame

            self.last_processed_sequence = (
                self.frame_sequence
            )

        # Create an annotated copy only when a ROS image subscriber exists.
        publish_debug_image = self.image_pub.get_subscription_count() > 0
        debug_frame = frame.copy() if publish_debug_image else None

        # 1. Target detection.
        start = time.perf_counter()

        try:
            target = detect_concentric_target(frame)

        except ValueError as error:
            self.get_logger().error(str(error))
            return

        self.timings['detection'] += (
            time.perf_counter() - start
        )

        # 2. Publish detection status.
        detected_msg = Bool()

        if target is not None:

            detected_msg.data = True

            center_x, center_y = target['center']

            center_msg = Float32MultiArray()

            center_msg.data = [
                float(center_x),
                float(center_y)
            ]

            self.center_pub.publish(center_msg)

            # Annotate only when the debug image is actually being consumed.
            if debug_frame is not None:
                cv2.circle(
                    debug_frame,
                    (round(center_x), round(center_y)),
                    5,
                    (0, 255, 0),
                    -1
                )

                cv2.putText(
                    debug_frame,
                    f'Center: ({center_x:.1f}, {center_y:.1f})',
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

        else:

            detected_msg.data = False

            if debug_frame is not None:
                cv2.putText(
                    debug_frame,
                    'Target not detected',
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

        self.detection_pub.publish(detected_msg)

        # Resize, convert and publish only when a debug-image subscriber exists.
        if debug_frame is not None:
            start = time.perf_counter()
            display_frame = cv2.resize(
                debug_frame,
                (640, 360),
                interpolation=cv2.INTER_AREA
            )
            self.timings['resize'] += time.perf_counter() - start

            start = time.perf_counter()
            image_msg = self.bridge.cv2_to_imgmsg(
                display_frame,
                encoding='bgr8'
            )
            self.timings['conversion'] += time.perf_counter() - start

            start = time.perf_counter()
            self.image_pub.publish(image_msg)
            self.timings['publish'] += time.perf_counter() - start

        # Measure total processing time.
        self.timings['total'] += (
            time.perf_counter() - frame_start
        )

        self.measurement_count += 1

        # Log performance every 30 processed frames.
        if self.measurement_count >= 30:

            count = self.measurement_count

            with self.frame_lock:

                if self.capture_samples > 0:
                    average_capture = (
                        self.capture_time_total
                        / self.capture_samples
                        * 1000
                    )
                else:
                    average_capture = 0.0

                self.capture_time_total = 0.0
                self.capture_samples = 0

            self.get_logger().info(
                'Average timings (milliseconds): '
                f'Capture={average_capture:.2f}, '
                f'Detection={self.timings["detection"] * 1000 / count:.2f}, '
                f'Resize={self.timings["resize"] * 1000 / count:.2f}, '
                f'Conversion={self.timings["conversion"] * 1000 / count:.2f}, '
                f'Publish={self.timings["publish"] * 1000 / count:.2f}, '
                f'Total={self.timings["total"] * 1000 / count:.2f}'
            )

            for key in self.timings:
                self.timings[key] = 0.0

            self.measurement_count = 0

    def destroy_node(self):
        """Stop capture thread and release camera resources."""

        self.stop_event.set()

        if hasattr(self, 'capture_thread'):
            self.capture_thread.join(timeout=2.0)

        if hasattr(self, 'camera') and self.camera.isOpened():
            self.camera.release()

        super().destroy_node()


def main(args=None):
    """Start the camera node."""

    rclpy.init(args=args)

    node = None

    try:
        node = CameraNode()
        rclpy.spin(node)

    except RuntimeError as error:
        print(f'Camera error: {error}')

    except KeyboardInterrupt:
        pass

    finally:

        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
