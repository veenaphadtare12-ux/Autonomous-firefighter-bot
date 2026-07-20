"""
=============================================================================
YOLO VISION v3.0 — Pi 5 Native Camera with Picamera2
=============================================================================
- Uses Picamera2 (native Pi 5 libcamera) instead of OpenCV VideoCapture
- Background thread continuously pulls camera frames (prevents buffer lag)
- Threading lock prevents race conditions between camera thread and main loop
- Confidence threshold filters out false positive detections
- Class filter ensures only "Flame" class triggers candle tracking
=============================================================================
"""
import cv2
import threading
import time
import numpy as np
from ultralytics import YOLO
from picamera2 import Picamera2


class Yolov8_Vision:
    """
    Tier 2: The Eyes of the Robot (V3 - Pi 5 Native Camera)
    Uses Picamera2 for frame capture (the ONLY method that works on Pi 5).
    Background thread continuously pulls the newest frame to prevent lag.
    """

    # Configuration
    CONFIDENCE_THRESHOLD = 0.10   # Lowered to 10% because live PREVIEW mode hits ~19% confidence
    FLAME_CLASS_ID       = 0      # Class index for "Flame" in your custom YOLO model
    CAMERA_WIDTH         = 640    # High resolution for long-range candle detection
    CAMERA_HEIGHT        = 640    # Set to 640x640

    # Filtering
    MAX_MISSED_FRAMES    = 2      # Reduced from 8 to 2 for instant response (no 3-second lag)

    def __init__(self, model_path='best.pt', camera_index=0):
        print("Waking up YOLOv8 Vision Brain (Pi 5 Native Camera)...")
        self.model = YOLO(model_path)

        # Open Camera using Picamera2 (Pi 5 native)
        print("Opening Picamera2...")
        self.picam2 = Picamera2()
        # Changed to video configuration to eliminate 300ms capture lag!
        config = self.picam2.create_video_configuration(
            main={"size": (self.CAMERA_WIDTH, self.CAMERA_HEIGHT), "format": "RGB888"}
        )
        self.picam2.configure(config)
        self.picam2.start()
        print("Picamera2 started successfully!")

        # Threading variables
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        
        # Start background thread to continually drain buffer and keep frame fresh
        self.thread = threading.Thread(target=self._update_frame, daemon=True)
        self.thread.start()
        
        # Persistence variables (Anti-Flicker)
        self.missed_frames = 999
        self.last_offset = 0.0
        self.last_dist = 1.0

        # Wait for camera to warm up and first frame to arrive
        time.sleep(2.0)
        print("Vision system ready!")
        
    def _update_frame(self):
        """Continuously pulls the newest frame from the camera to prevent buffer lag."""
        while self.running:
            try:
                # This blocks until a new frame is available, draining the camera buffer!
                new_frame = self.picam2.capture_array("main")
                with self.lock:
                    self.frame = new_frame
            except Exception as e:
                time.sleep(0.01)

    def scan_for_candle(self):
        """
        Instantly captures a fresh frame and runs YOLO result.
        Returns: (x_offset, distance, detected)
            x_offset: -1.0 (candle is far left) to 1.0 (candle is far right)
            distance: 0.0 (candle is touching) to 1.0 (candle is far away)
            detected: True if a candle/flame was found
        """
        with self.lock:
            if self.frame is None:
                return 0.0, 1.0, False
            frame_copy = self.frame.copy()

        # Run YOLO inference on the copied frame (verbose=False hides spam)
        # imgsz=320 processes the image 4x faster than 640, massively boosting FPS
        results = self.model(frame_copy, imgsz=320, verbose=False)

        best_candle = None
        largest_area = 0
        best_confidence = 0.0

        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Filter by confidence threshold
                confidence = float(box.conf[0])
                if confidence < self.CONFIDENCE_THRESHOLD:
                    continue

                # Filter by class — only accept flame detections
                class_id = int(box.cls[0])
                if class_id != self.FLAME_CLASS_ID:
                    continue

                x_center, y_center, w, h = box.xywh[0]
                area = w * h
                if area > largest_area:
                    largest_area = area
                    best_confidence = confidence
                    best_candle = (x_center.item(), y_center.item(), w.item(), h.item())

        if best_candle is not None:
            cx, cy, cw, ch = best_candle
            frame_width = frame_copy.shape[1]
            frame_height = frame_copy.shape[0]

            # X-Offset: -1.0 (left) to 1.0 (right)
            x_offset = (cx - (frame_width / 2)) / (frame_width / 2)

            # Distance estimate: 0.0 (touching) to 1.0 (far away)
            # Based on bounding box height relative to frame height
            distance = 1.0 - (ch / frame_height)
            distance = max(0.0, min(distance, 1.0))

            # Save for persistence
            self.missed_frames = 0
            self.last_offset = x_offset
            self.last_dist = distance
            return x_offset, distance, True

        # YOLO didn't see it this frame. Check persistence filter.
        self.missed_frames += 1
        if self.missed_frames < self.MAX_MISSED_FRAMES:
            # We missed it, but we saw it very recently. Keep returning the last known good value!
            return self.last_offset, self.last_dist, True

        # Truly lost
        return 0.0, 1.0, False

    def get_annotated_frame(self):
        """Returns the image frame with YOLO bounding boxes drawn for livestreaming."""
        with self.lock:
            if self.frame is None:
                return None
            frame_copy = self.frame.copy()

        # imgsz=320 makes the AI process the image 4x faster!
        results = self.model(frame_copy, imgsz=320, verbose=False)
        annotated = results[0].plot()
        # Convert back to BGR so OpenCV streams it with correct colors
        return cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)

    def shutdown(self):
        """Gracefully stops the camera and releases the hardware."""
        print("Shutting down Vision Thread...")
        self.running = False
        try:
            self.picam2.stop()
            self.picam2.close()
        except:
            pass
        print("Vision shutdown complete.")
