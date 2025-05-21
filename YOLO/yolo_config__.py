from ultralytics import YOLO
import cv2
import numpy as np
from typing import Optional
import subprocess
import os

class YOLOConfig:
    def __init__(self):
        self.model: YOLO = YOLO("yolov8n.pt")
        # Define class names for better output
        self.class_names = self.model.names

    def export_model(self):
        self.model.export(format='onnx',imgsz=640)

    def get_movement_suggestion(self, cx, frame_width):
        """Simple function to suggest movement based on person's position"""
        if cx < frame_width * 0.4:
            return "TURN RIGHT"
        elif cx > frame_width * 0.6:
            return "TURN LEFT"
        else:
            return "GO STRAIGHT"

    def detect_image(self, image_path: Optional[str] = None):
        # If no image path is provided, use the camera
        if image_path is None:
            try:
                # Use rpicam-vid command to capture frames
                cmd = [
                    "rpicam-vid",
                    "-t", "0",  # Run indefinitely
                    "-o", "-",  # Output to stdout
                    "--width", "640",
                    "--height", "480",
                    "--framerate", "30",
                    "--codec", "h264",
                    "--inline",  # Enable inline headers
                    "--nopreview"  # Disable preview window
                ]
                
                print("Starting camera capture...")
                process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                
                # Create a pipe to read the video stream
                cap = cv2.VideoCapture("pipe:0")
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 3)
                
                if not cap.isOpened():
                    print("Error: Could not open camera stream")
                    return

                print("Camera stream opened successfully")
                
                while True:
                    ret, frame = cap.read()
                    if not ret:
                        print("Error: Could not read frame")
                        break
                    
                    # Get frame dimensions
                    frame_height, frame_width = frame.shape[:2]
                    
                    # Run YOLO detection
                    results = self.model(frame)
                    
                    person_detected = False
                    for result in results:
                        boxes = result.boxes
                        for box in boxes:
                            class_id = int(box.cls[0])
                            conf = float(box.conf[0])
                            class_name = self.class_names[class_id]
                            
                            if class_id == 0 and conf > 0.5:  # Person detected
                                person_detected = True
                                x1, y1, x2, y2 = map(int, box.xyxy[0])
                                cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                                
                                # Calculate distance approximation (based on bounding box size)
                                box_height = y2 - y1
                                distance_ratio = box_height / frame_height
                                
                                # Draw detection
                                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,0,255), 2)
                                cv2.circle(frame, (cx,cy), 5, (0,255,0), -1)
                                
                                # Get movement suggestion
                                movement = self.get_movement_suggestion(cx, frame_width)
                                
                                # Print detection info
                                print(f"\nPerson detected!")
                                print(f"Confidence: {conf:.2f}")
                                print(f"Position: Center at ({cx}, {cy})")
                                print(f"Distance ratio: {distance_ratio:.2f}")
                                print(f"Suggested movement: {movement}")
                                
                                if distance_ratio > 0.5:
                                    print("WARNING: Person is too close! Should STOP")
                                
                                # Add text to frame
                                cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1-10),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                                cv2.putText(frame, movement, (cx-50, cy+20),
                                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                            else:
                                print(f"Detected: {class_name} (Confidence: {conf:.2f})")
                    
                    if not person_detected:
                        print("\nNo person detected in frame")
                    
                    cv2.imshow("YOLO", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

            except Exception as e:
                print(f"Error: {e}")
            finally:
                if 'cap' in locals():
                    cap.release()
                if 'process' in locals():
                    process.terminate()
                cv2.destroyAllWindows()
        else:
            # Original code for image file processing
            cap = cv2.VideoCapture(image_path)
            while True:
                ret, frame = cap.read()
                if not ret: 
                    break
                
                results = self.model(frame)

                for result in results:
                    class_ids = result.boxes.cls[0]
                    conf = result.boxes.conf[0]

                    if class_ids == 0 and conf > 0.5:
                        print("Person detected")

                        x1,y1,x2,y2 = result.boxes.xyxy[0]
                        cx, cy = int((x1+x2)/2), int((y1+y2)/2)

                        cv2.rectangle(frame,(x1,y1),(x2,y2),(0,0,255),2)
                        cv2.circle(frame,(cx,cy),5,(0,255,0),-1)

                        cv2.putText(frame,f"Person {conf[0]:.2f}",(cx,cy),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                    
                    cv2.imshow("YOLO",frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

            cap.release()
            cv2.destroyAllWindows()