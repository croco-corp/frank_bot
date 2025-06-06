from .YOLO import hailo_detection_tracker

if __name__ == "__main__":
    user_data = hailo_detection_tracker.user_app_callback_class()
    app = hailo_detection_tracker.GStreamerDetectionApp(hailo_detection_tracker.app_callback, user_data)
    app.run()