import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
import numpy as np
import cv2
import hailo
from hailo_apps_infra.hailo_rpi_common import (
    get_caps_from_pad,
    get_numpy_from_buffer,
    app_callback_class,
)
from hailo_apps_infra.detection_pipeline import GStreamerDetectionApp
from execution.components import Navigator, CommunicatorMessage
import threading

class YoloNavigator(Navigator):
    def __init__(self):
        super().__init__()
        self._object_detected: bool = False
        self._object_in_leftzone: bool = False
        self._object_in_rightzone: bool = False
        self._direction: CommunicatorMessage = CommunicatorMessage.STOP
        self._direction_lock = threading.Lock()


    def start(self) -> None:
        user_data = _user_app_callback_class()
        app = GStreamerDetectionApp(self._app_callback, user_data)
        app.run()

    def _app_callback(self, pad, info, user_data) -> Gst.PadProbeReturn:
        buffer = info.get_buffer()
        if buffer is None:
            return Gst.PadProbeReturn.OK

        user_data.increment()

        format, width, height = get_caps_from_pad(pad)
        frame = None
        if user_data.use_frame and format is not None and width is not None and height is not None:
            frame = get_numpy_from_buffer(buffer, format, width, height)

        roi = hailo.get_roi_from_buffer(buffer)
        detections = roi.get_objects_typed(hailo.HAILO_DETECTION)

        for detection in detections:
            label = detection.get_label()
            confidence = detection.get_confidence()
            
            if confidence > 0.6 and label == user_data.target_object:
                self._object_detected = True
                bbox = detection.get_bbox()
                
                x_min = bbox.xmin()
                y_min = bbox.ymin()
                box_width = bbox.width()
                box_height = bbox.height()
                
                x_max = x_min + box_width
                y_max = y_min + box_height
                
                center_x = x_min + (box_width / 2)
                center_y = (y_min + (box_height / 2) - 0.22) * 1.83

                if (user_data.leftzone_x_min <= center_x <= user_data.leftzone_x_max and 
                    (user_data.leftzone_y_min - 0.22) * 1.83 <= center_y <= (user_data.leftzone_y_max - 0.22) * 1.83):
                    self._object_in_leftzone = True
                    with self._direction_lock:
                        self._direction = CommunicatorMessage.LEFT
                elif (user_data.rightzone_x_min <= center_x <= user_data.rightzone_x_max and 
                    (user_data.rightzone_y_min - 0.22) * 1.83 <= center_y <= (user_data.rightzone_y_max - 0.22) * 1.83):
                    self._object_in_rightzone = True
                    with self._direction_lock:
                        self._direction = CommunicatorMessage.RIGHT
                else: 
                    with self._direction_lock:
              
                        self._direction = CommunicatorMessage.FORWARD
        if self._object_detected:
            user_data.in_zone_frames += 1
            user_data.out_zone_frames = 0
            
            if user_data.in_zone_frames >= 4 and not user_data.is_it_active:
                user_data.is_it_active = True
                if self._object_in_rightzone:
                    with self._direction_lock:
                        self._direction = CommunicatorMessage.RIGHT
                elif self._object_in_leftzone:
                    with self._direction_lock: 
                        self._direction = CommunicatorMessage.LEFT
                else:
                    with self._direction_lock:
                        self._direction = CommunicatorMessage.FORWARD
        else:
            user_data.out_zone_frames += 1
            user_data.in_zone_frames = 0
            
            if user_data.out_zone_frames >= 5 and user_data.is_it_active:
                user_data.is_it_active = False
                with self._direction_lock:
                    self._direction = (CommunicatorMessage.STOP)

        self._object_detected = False
        self._object_in_leftzone = False
        self._object_in_rightzone = False
        
        return Gst.PadProbeReturn.OK

    def get_next_step(self) -> CommunicatorMessage:
        with self._direction_lock:
            return self._direction
    
class _user_app_callback_class(app_callback_class):
    def __init__(self):
        super().__init__()
        # Configuration
        self.target_object: str = "person"  # Object type to detect
        
        # Target zone configuration (normalized coordinates 0-1)
        self.leftzone_x_min: float = 0.0  # Left boundary of target zone
        self.leftzone_x_max: float = 0.4  # Right boundary of target zone
        self.leftzone_y_min: float = 0.0  # Top boundary of target zone
        self.leftzone_y_max: float = 1  # Bottom boundary of target zone
        
        self.rightzone_x_min: float = 0.6 # Left boundary of target zone
        self.rightzone_x_max: float = 1  # Right boundary of target zone
        self.rightzone_y_min: float = 0.0  # Top boundary of target zone
        self.rightzone_y_max: float = 1  # Bottom boundary of target zone
        # Debouncing variables
        self.in_zone_frames: int = 0      # Consecutive frames with object in zone
        self.out_zone_frames: int = 0     # Consecutive frames without object in zone
        
        # State tracking
        self.is_it_active: bool = False
