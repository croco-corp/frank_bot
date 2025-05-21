from yolo_config__ import YOLOConfig
from yolo_config import PersonTracker

def main():
    # yolo_config = YOLOConfig()
    # yolo_config.detect_image()
    # yolo_config.export_model()

    person_tracker = PersonTracker()
    person_tracker.track_person()
    person_tracker.export_model()
    person_tracker.get_movement_suggestion()
    person_tracker.select_person_to_track()
    

if __name__ == "__main__":
    main()