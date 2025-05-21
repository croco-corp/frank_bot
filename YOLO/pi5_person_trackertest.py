from ultralytics import YOLO
import cv2
import numpy as np
import time
import subprocess
import os
import io
import picamera2
from picamera2 import Picamera2
from typing import Optional, Tuple, List
import threading

class PersonTracker:
    def __init__(self, use_hailo=True):
        """
        Initialize the PersonTracker
        
        Args:
            use_hailo: Whether to use Hailo acceleration
        """
        self.use_hailo = use_hailo
        
        if use_hailo:
            if os.path.exists("yolov8n_hailo.onnx"):
                print("Loading Hailo-optimized ONNX model")
                self.model = YOLO("yolov8n_hailo.onnx")
            else:
                print("Hailo model not found, using standard model")
                self.model = YOLO("yolov8n.pt")
        else:
            self.model = YOLO("yolov8n.pt")
        
        self.class_names = self.model.names
        
        self.tracked_person_id = None
        self.last_person_center = None
        self.tracking_lost_frames = 0
        self.MAX_LOST_FRAMES = 30  
        
        self.tracking_color = (0, 255, 0)  
        self.other_person_color = (0, 165, 255)  
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        
        print(f"PersonTracker initialized with {'Hailo acceleration' if use_hailo else 'standard CPU processing'}")
    
    def export_model_for_hailo(self, optimize=True):
        """Export the model to ONNX format for Hailo acceleration"""
        self.model.export(format='onnx', imgsz=640)
        print("Model exported to ONNX format")
        
        if optimize and self.use_hailo:
            # This is a placeholder - you would use Hailo Model Zoo tools here
            # to optimize the ONNX model for the Hailo device
            print("Note: For actual Hailo optimization, you need to use Hailo Dataflow Compiler")
            print("Visit https://hailo.ai/developer-zone/ for the proper tools")
    
    def get_movement_suggestion(self, cx: int, frame_width: int, box_height: int, frame_height: int) -> str:
        """
        Determines movement direction based on person's position
        
        Args:
            cx: x-coordinate of person's center
            frame_width: width of the frame
            box_height: height of the person's bounding box
            frame_height: height of the frame
            
        Returns:
            String with suggested movement
        """
        # Calculate distance approximation (based on bounding box size)
        distance_ratio = box_height / frame_height
        
        # Stop if too close
        if distance_ratio > 0.6:
            return "STOP"
        
        # Determine horizontal position and suggest movement
        if cx < frame_width * 0.45:
            return "TURN RIGHT"
        elif cx > frame_width * 0.55:
            return "TURN LEFT"
        else:
            return "GO STRAIGHT"
    
    def select_person_to_track(self, person_detections: List[dict]) -> Optional[int]:
        """
        Selects which person to track among multiple detections
        
        Args:
            person_detections: List of person detection dictionaries
            
        Returns:
            Index of the person to track or None if no suitable person
        """
        if not person_detections:
            return None
            
        # If we're already tracking someone, find the closest match
        if self.last_person_center is not None:
            last_x, last_y = self.last_person_center
            
            min_distance = float('inf')
            closest_idx = 0
            
            for i, person in enumerate(person_detections):
                cx, cy = person['center']
                dist = ((cx - last_x) ** 2 + (cy - last_y) ** 2) ** 0.5
                
                if dist < min_distance:
                    min_distance = dist
                    closest_idx = i
            
            # If we found a reasonably close match, track that person
            if min_distance < 100:  # Threshold distance in pixels
                return closest_idx
        
        # If no previous tracking or no close match, pick the largest person (closest)
        # as they're likely the subject we want to follow
        largest_idx = 0
        largest_area = 0
        
        for i, person in enumerate(person_detections):
            area = person['box_height'] * person['box_width']
            if area > largest_area:
                largest_area = area
                largest_idx = i
        
        return largest_idx

    def setup_picamera2(self):
        """Set up PiCamera2 for Camera Module v3"""
        try:
            # Initialize Picamera2
            print("Initializing PiCamera2...")
            camera = Picamera2()
            
            # Configure camera
            config = camera.create_preview_configuration(
                main={"size": (640, 480), "format": "RGB888"},
                buffer_count=4
            )
            camera.configure(config)
            
            print("Camera configured. Starting...")
            camera.start()
            print("Camera started successfully")
            
            return camera
        except Exception as e:
            print(f"Error setting up PiCamera2: {e}")
            return None
    
    def track_person(self, video_source: Optional[str] = None):
        """
        Track a single person using YOLOv8 and camera input
        
        Args:
            video_source: Optional path to video file, None uses live camera
        """
        cap = None
        camera = None
        
        # Setup camera 
        if video_source is None:
            print("Setting up camera...")
            
            # Try PiCamera2 first (preferred for Pi 5 with Camera Module v3)
            try:
                camera = self.setup_picamera2()
                if camera is None:
                    raise Exception("Failed to initialize PiCamera2")
                print("Using PiCamera2")
            except Exception as e:
                print(f"PiCamera2 setup failed: {e}")
                print("Trying fallback methods...")
                
                # Try using rpicam-vid with gstreamer
                try:
                    gst_str = (
                        "rpicam-vid -t 0 --width 640 --height 480 --framerate 30 "
                        "--codec mjpeg --inline --nopreview -o - ! "
                        "fdsrc ! jpegdec ! videoconvert ! appsink"
                    )
                    cap = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
                    if not cap.isOpened():
                        raise Exception("GStreamer pipeline failed")
                    print("Using rpicam-vid with GStreamer")
                except Exception as e:
                    print(f"GStreamer method failed: {e}")
                    
                    # Final fallback to standard camera
                    try:
                        print("Trying standard camera capture...")
                        cap = cv2.VideoCapture(0)
                        if not cap.isOpened():
                            raise Exception("Could not open default camera")
                        print("Using standard camera capture")
                    except Exception as e:
                        print(f"All camera methods failed: {e}")
                        return
        else:
            # Use provided video file
            cap = cv2.VideoCapture(video_source)
            if not cap.isOpened():
                print(f"Error: Could not open video source: {video_source}")
                return
        
        # Performance metrics
        frame_count = 0
        start_time = time.time()
        fps = 0
        
        try:
            while True:
                # Get frame either from PiCamera2 or OpenCV capture
                if camera is not None:
                    # Get frame from PiCamera2
                    frame = camera.capture_array()
                    ret = True
                elif cap is not None:
                    # Get frame from OpenCV capture
                    ret, frame = cap.read()
                else:
                    print("No camera available")
                    break
                
                if not ret or frame is None:
                    print("Error: Could not read frame")
                    break
                
                # Get frame dimensions
                frame_height, frame_width = frame.shape[:2]
                
                # Process frame with YOLO
                start_process = time.time()
                results = self.model(frame, classes=[0])  # Only detect people (class 0)
                process_time = time.time() - start_process
                
                # Extract all person detections
                person_detections = []
                
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        confidence = float(box.conf[0])
                        
                        # Only consider high-confidence detections
                        if confidence > 0.5:
                            x1, y1, x2, y2 = map(int, box.xyxy[0])
                            cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                            box_width = x2 - x1
                            box_height = y2 - y1
                            
                            person_detections.append({
                                'confidence': confidence,
                                'box': (x1, y1, x2, y2),
                                'center': (cx, cy),
                                'box_width': box_width,
                                'box_height': box_height
                            })
                
                # Add FPS counter to frame
                frame_count += 1
                if frame_count % 10 == 0:  # Update FPS every 10 frames
                    fps = frame_count / (time.time() - start_time)
                
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), self.font, 0.7, (0, 0, 255), 2)
                
                # Process tracking logic
                if not person_detections:
                    # No people detected
                    self.tracking_lost_frames += 1
                    cv2.putText(frame, "No person detected", (frame_width//2 - 100, 30), 
                                self.font, 0.7, (0, 0, 255), 2)
                    
                    if self.tracking_lost_frames > self.MAX_LOST_FRAMES:
                        self.last_person_center = None  # Reset tracking
                else:
                    # Select person to track
                    track_idx = self.select_person_to_track(person_detections)
                    tracked_person = person_detections[track_idx]
                    
                    # Reset lost frame counter
                    self.tracking_lost_frames = 0
                    
                    # Draw all detected people
                    for i, person in enumerate(person_detections):
                        x1, y1, x2, y2 = person['box']
                        cx, cy = person['center']
                        
                        # Use different color for tracked vs other people
                        color = self.tracking_color if i == track_idx else self.other_person_color
                        thickness = 2 if i == track_idx else 1
                        
                        # Draw bounding box
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                        
                        # Draw center point
                        cv2.circle(frame, (cx, cy), 5, color, -1)
                        
                        # Add confidence label
                        if i == track_idx:
                            cv2.putText(frame, f"Tracked: {person['confidence']:.2f}", 
                                      (x1, y1-10), self.font, 0.5, color, 2)
                    
                    # Update tracking info for the tracked person
                    tracked_x1, tracked_y1, tracked_x2, tracked_y2 = tracked_person['box']
                    tracked_cx, tracked_cy = tracked_person['center']
                    self.last_person_center = (tracked_cx, tracked_cy)
                    
                    # Get and display movement suggestion
                    movement = self.get_movement_suggestion(
                        tracked_cx, 
                        frame_width,
                        tracked_person['box_height'],
                        frame_height
                    )
                    
                    # Display movement suggestion
                    cv2.putText(frame, movement, (frame_width//2 - 60, frame_height - 20),
                              self.font, 0.8, (0, 255, 0), 2)
                
                # Display the frame
                cv2.imshow("Person Tracker", frame)
                
                # Check for key press
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    # Reset tracking
                    self.last_person_center = None
                    self.tracking_lost_frames = 0
                    print("Tracking reset")
        
        except Exception as e:
            print(f"Error during tracking: {e}")
        finally:
            # Clean up resources
            if cap is not None:
                cap.release()
            if camera is not None:
                camera.stop()
                camera.close()
            cv2.destroyAllWindows()
            print(f"Tracking ended. Average FPS: {frame_count/(time.time()-start_time):.1f}")


def main():
    # Create person tracker (set use_hailo=True to enable Hailo acceleration)
    tracker = PersonTracker(use_hailo=False)  # Set to True when Hailo is properly set up
    
    # Export model for Hailo if needed (only do this once)
    # Uncomment this if you need to export the model for Hailo
    tracker.export_model_for_hailo()
    
    # Start tracking
    tracker.track_person()


if __name__ == "__main__":
    main()
