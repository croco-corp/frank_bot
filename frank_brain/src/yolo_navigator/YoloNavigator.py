import cv2
import numpy as np
import threading
from execution.components import Navigator, CommunicatorMessage
from yolo_navigator.utils import load_model, run_inference  # Make sure this matches your utils path

class YoloNavigator(Navigator):
    def __init__(self, model_path, camera_index=0):
        super().__init__()
        self._object_detected: bool = False
        self._object_in_leftzone: bool = False
        self._object_in_rightzone: bool = False
        self._direction: CommunicatorMessage = CommunicatorMessage.STOP
        self._direction_lock = threading.Lock()
        self.model = load_model(model_path)
        self.camera_index = camera_index

    def start(self) -> None:
        user_data = _UserAppCallbackClass()
        cap = cv2.VideoCapture(self.camera_index)
        if not cap.isOpened():
            print("Error: Could not open webcam.")
            return

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to capture frame.")
                break

            detections = run_inference(self.model, frame)
            self._process_detections(detections, user_data)

        cap.release()

    def _process_detections(self, detections, user_data):
        self._object_detected = False
        self._object_in_leftzone = False
        self._object_in_rightzone = False

        for detection in detections:
            label = detection['label']
            confidence = detection['confidence']
            bbox = detection['bbox']  # [x_min, y_min, width, height], normalized 0-1

            if confidence > 0.6 and label == user_data.target_object:
                self._object_detected = True
                x_min, y_min, box_width, box_height = bbox
                x_max = x_min + box_width
                y_max = y_min + box_height
                center_x = x_min + (box_width / 2)
                center_y = y_min + (box_height / 2)

                print(f"{label.capitalize()} detected! - I will go straight\n"
                      f"Position: center=({center_x:.2f}, {center_y:.2f})\n"
                      f"Bounds: xmin={x_min:.2f}, ymin={y_min:.2f}, xmax={x_max:.2f}, ymax={y_max:.2f}\n"
                      f"Confidence: {confidence:.2f}\n")

                if (user_data.leftzone_x_min <= center_x <= user_data.leftzone_x_max and 
                    user_data.leftzone_y_min <= center_y <= user_data.leftzone_y_max):
                    self._object_in_leftzone = True
                    print(f"Object is in left zone!\n")
                if (user_data.rightzone_x_min <= center_x <= user_data.rightzone_x_max and 
                    user_data.rightzone_y_min <= center_y <= user_data.rightzone_y_max):
                    self._object_in_rightzone = True
                    print(f"Object is in right zone!\n")

        if self._object_detected:
            user_data.in_zone_frames += 1
            user_data.out_zone_frames = 0

            if user_data.in_zone_frames >= 4 and not user_data.is_it_active:
                user_data.is_it_active = True
                with self._direction_lock:
                    if self._object_in_rightzone:
                        self._direction = CommunicatorMessage.RIGHT
                    elif self._object_in_leftzone:
                        self._direction = CommunicatorMessage.LEFT
                    else:
                        self._direction = CommunicatorMessage.FORWARD
        else:
            user_data.out_zone_frames += 1
            user_data.in_zone_frames = 0

            if user_data.out_zone_frames >= 5 and user_data.is_it_active:
                user_data.is_it_active = False
                print(f"No {user_data.target_object} in target zone - I will stop")
                with self._direction_lock:
                    self._direction = CommunicatorMessage.STOP

    def get_next_step(self) -> CommunicatorMessage:
        with self._direction_lock:
            return self._direction

class _UserAppCallbackClass:
    def __init__(self):
        self.target_object: str = "person"
        self.leftzone_x_min: float = 0.0
        self.leftzone_x_max: float = 0.4
        self.leftzone_y_min: float = 0.0
        self.leftzone_y_max: float = 1.0
        self.rightzone_x_min: float = 0.6
        self.rightzone_x_max: float = 1.0
        self.rightzone_y_min: float = 0.0
        self.rightzone_y_max: float = 1.0
        self.in_zone_frames: int = 0
        self.out_zone_frames: int = 0
        self.is_it_active: bool = False

if __name__ == "__main__":
    navigator = YoloNavigator(model_path="resources/models/yolo-v8n.hef")
    navigator_thread = threading.Thread(target=navigator.start, daemon=True)
    navigator_thread.start()

    import time
    while True:
        print(navigator.get_next_step())
        time.sleep(1)