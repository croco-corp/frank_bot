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

try:
    from hailo_platform import (HEF, Device, VDevice, HailoStreamInterface, 
                               InferVStreams, ConfigureParams)
    HAILO_AVAILABLE = True
    print("Hailo platform imports successful")
except ImportError:
    HAILO_AVAILABLE = False
    print("Hailo platform not available - falling back to CPU/ONNX")

class PersonTracker:
    def __init__(self, use_hailo=True):
        self.use_hailo = use_hailo and HAILO_AVAILABLE
        self.hailo_device = None
        self.hailo_network_group = None
        self.input_vstream_info = None
        self.output_vstream_info = None
        self.input_vstream = None
        self.output_vstream = None
        
        if self.use_hailo:
            success = self._setup_hailo_model()
            if not success:
                print("Hailo setup failed, falling back to YOLO")
                self.use_hailo = False
                self._setup_yolo_model()
        else:
            self._setup_yolo_model()
        
        self.tracked_person_id = None
        self.last_person_center = None
        self.tracking_lost_frames = 0
        self.MAX_LOST_FRAMES = 30  
        
        self.tracking_color = (0, 255, 0)  
        self.other_person_color = (0, 165, 255)  
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        
        print(f"PersonTracker initialized with {'Hailo acceleration' if self.use_hailo else 'YOLO CPU processing'}")
    
    def _setup_hailo_model(self):
        hef_path = "yolov8n.hef"
        
        if not os.path.exists(hef_path):
            print(f"HEF file not found: {hef_path}")
            print("To create HEF file, use Hailo Dataflow Compiler:")
            print("1. Convert ONNX to HAR: hailo parser onnx yolov8n.onnx")
            print("2. Optimize: hailo optimize yolov8n.har")
            print("3. Compile to HEF: hailo compiler yolov8n_optimized.har")
            return False
        
        try:
            self.hailo_device = Device()
            
            hef = HEF(hef_path)
            
            configure_params = ConfigureParams.create_from_hef(hef, interface=HailoStreamInterface.PCIe)
            self.hailo_network_group = VDevice(self.hailo_device).configure(hef, configure_params)[0]
            
            self.input_vstream_info = hef.get_input_vstream_infos()[0]
            self.output_vstream_info = hef.get_output_vstream_infos()
            
            self.input_vstream = InferVStreams.create_from_hef(hef, self.input_vstream_info)
            self.output_vstream = InferVStreams.create_from_hef(hef, self.output_vstream_info)
            
            print(f"Hailo model loaded successfully from {hef_path}")
            print(f"Input shape: {self.input_vstream_info.shape}")
            print(f"Output streams: {len(self.output_vstream_info)}")
            
            return True
            
        except Exception as e:
            print(f"Error setting up Hailo model: {e}")
            return False
    
    def _setup_yolo_model(self):
        if os.path.exists("yolov8n.onnx"):
            print("Loading ONNX model for CPU inference")
            self.model = YOLO("yolov8n.onnx")
        else:
            print("Loading PyTorch model")
            self.model = YOLO("yolov8n.pt")
        
        self.class_names = self.model.names
    
    def _preprocess_for_hailo(self, frame):
        if not self.use_hailo:
            return frame
        
        expected_shape = self.input_vstream_info.shape
        height, width = expected_shape[1], expected_shape[2]
        
        resized = cv2.resize(frame, (width, height))
        
        if len(expected_shape) == 4 and expected_shape[3] == 3:
            resized = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        
        preprocessed = resized.astype(np.float32) / 255.0
        
        preprocessed = np.expand_dims(preprocessed, axis=0)
        
        return preprocessed
    
    def _postprocess_hailo_output(self, raw_outputs, original_shape):
        detections = []
        
        try:
            if len(raw_outputs) == 1:
                output = raw_outputs[0][0]
                
                for detection in output:
                    if len(detection) >= 6:
                        x1, y1, x2, y2, conf, cls = detection[:6]
                        
                        if int(cls) == 0 and conf > 0.5:
                            orig_h, orig_w = original_shape[:2]
                            model_h, model_w = self.input_vstream_info.shape[1:3]
                            
                            x1 = int(x1 * orig_w / model_w)
                            y1 = int(y1 * orig_h / model_h)
                            x2 = int(x2 * orig_w / model_w)
                            y2 = int(y2 * orig_h / model_h)
                            
                            cx, cy = int((x1+x2)/2), int((y1+y2)/2)
                            
                            detections.append({
                                'confidence': float(conf),
                                'box': (x1, y1, x2, y2),
                                'center': (cx, cy),
                                'box_width': x2 - x1,
                                'box_height': y2 - y1
                            })
            
            else:
                print("Multiple output format detected - implement postprocessing")
                
        except Exception as e:
            print(f"Error in Hailo postprocessing: {e}")
        
        return detections
    
    def _run_hailo_inference(self, frame):
        if not self.use_hailo:
            return []
        
        try:
            preprocessed = self._preprocess_for_hailo(frame)
            
            with self.hailo_network_group.activate():
                self.input_vstream.send(preprocessed)
                
                raw_outputs = []
                for output_stream in self.output_vstream:
                    raw_outputs.append(output_stream.recv())
            
            detections = self._postprocess_hailo_output(raw_outputs, frame.shape)
            
            return detections
            
        except Exception as e:
            print(f"Error in Hailo inference: {e}")
            return []
    
    def _run_yolo_inference(self, frame):
        person_detections = []
        
        results = self.model(frame, classes=[0])
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    confidence = float(box.conf[0])
                    
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
        
        return person_detections
    
    def detect_persons(self, frame):
        if self.use_hailo:
            return self._run_hailo_inference(frame)
        else:
            return self._run_yolo_inference(frame)
    
    def get_movement_suggestion(self, cx: int, frame_width: int, box_height: int, frame_height: int) -> str:
        distance_ratio = box_height / frame_height
        
        if distance_ratio > 0.3:
            return "STOP"
        
        if cx < frame_width * 0.45:
            return "TURN RIGHT"
        elif cx > frame_width * 0.55:
            return "TURN LEFT"
        else:
            return "GO STRAIGHT"
    
    def select_person_to_track(self, person_detections: List[dict]) -> Optional[int]:
        if not person_detections:
            return None
            
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
            
            if min_distance < 100:
                return closest_idx
        
        largest_idx = 0
        largest_area = 0
        
        for i, person in enumerate(person_detections):
            area = person['box_height'] * person['box_width']
            if area > largest_area:
                largest_area = area
                largest_idx = i
        
        return largest_idx

    def setup_picamera2(self):
        try:
            print("Initializing PiCamera2...")
            camera = Picamera2()
            
            try:
                camera_info = camera.camera_properties
                print(f"Camera detected: Model {camera_info.get('Model', 'Unknown')}")
            except:
                print("Camera detected (properties not accessible)")
            
            configurations_to_try = [
                lambda: camera.create_still_configuration(
                    main={"size": (640, 480), "format": "RGB888"},
                    buffer_count=4
                ),
                lambda: camera.create_video_configuration(
                    main={"size": (640, 480), "format": "RGB888"},
                    buffer_count=4
                ),
                lambda: camera.create_preview_configuration(
                    main={"size": (640, 480), "format": "RGB888"},
                    buffer_count=4
                )
            ]
            
            config = None
            for i, config_func in enumerate(configurations_to_try):
                try:
                    print(f"Trying configuration method {i+1}...")
                    config = config_func()
                    camera.configure(config)
                    print(f"Configuration {i+1} successful")
                    break
                except Exception as e:
                    print(f"Configuration {i+1} failed: {e}")
                    continue
            
            if config is None:
                raise Exception("All configuration methods failed")
            
            print("Starting camera...")
            camera.start()
            
            print("Waiting for camera to stabilize...")
            time.sleep(2)
            
            try:
                test_frame = camera.capture_array()
                print(f"Test capture successful: {test_frame.shape}")
            except Exception as e:
                print(f"Warning: Test capture failed: {e}")
            
            print("Camera started successfully")
            return camera
            
        except Exception as e:
            print(f"Error setting up PiCamera2: {e}")
            return None

    def track_person(self, video_source: Optional[str] = None):
        cap = None
        camera = None
        
        if video_source is None:
            print("Setting up camera...")
            
            try:
                camera = self.setup_picamera2()
                if camera is None:
                    raise Exception("Failed to initialize PiCamera2")
                print("Using PiCamera2")
            except Exception as e:
                print(f"PiCamera2 setup failed: {e}")
                print("Camera initialization failed. Please check camera connection.")
                return
        else:
            cap = cv2.VideoCapture(video_source)
            if not cap.isOpened():
                print(f"Error: Could not open video source: {video_source}")
                return
        
        frame_count = 0
        start_time = time.time()
        fps = 0
        total_inference_time = 0
        
        try:
            print("Starting tracking loop...")
            print(f"Using {'Hailo accelerator' if self.use_hailo else 'CPU inference'}")
            
            while True:
                if camera is not None:
                    try:
                        frame = camera.capture_array()
                        ret = True
                    except Exception as e:
                        print(f"Frame capture error: {e}")
                        ret = False
                        frame = None
                elif cap is not None:
                    ret, frame = cap.read()
                else:
                    print("No camera available")
                    break
                
                if not ret or frame is None:
                    print("Error: Could not read frame")
                    break
                
                frame_height, frame_width = frame.shape[:2]
                
                start_inference = time.time()
                person_detections = self.detect_persons(frame)
                inference_time = time.time() - start_inference
                total_inference_time += inference_time
                
                frame_count += 1
                if frame_count % 10 == 0:
                    fps = frame_count / (time.time() - start_time)
                    avg_inference = (total_inference_time / frame_count) * 1000
                
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30), self.font, 0.7, (0, 0, 255), 2)
                cv2.putText(frame, f"Inference: {inference_time*1000:.1f}ms", (10, 60), self.font, 0.7, (0, 0, 255), 2)
                cv2.putText(frame, f"Engine: {'Hailo' if self.use_hailo else 'CPU'}", (10, 90), self.font, 0.7, (255, 0, 0), 2)
                
                if not person_detections:
                    self.tracking_lost_frames += 1
                    cv2.putText(frame, "No person detected", (frame_width//2 - 100, 30), 
                                self.font, 0.7, (0, 0, 255), 2)
                    
                    if self.tracking_lost_frames > self.MAX_LOST_FRAMES:
                        self.last_person_center = None
                else:
                    track_idx = self.select_person_to_track(person_detections)
                    if track_idx is not None:
                        tracked_person = person_detections[track_idx]
                        self.tracking_lost_frames = 0
                        
                        for i, person in enumerate(person_detections):
                            x1, y1, x2, y2 = person['box']
                            cx, cy = person['center']
                            
                            color = self.tracking_color if i == track_idx else self.other_person_color
                            thickness = 2 if i == track_idx else 1
                            
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                            cv2.circle(frame, (cx, cy), 5, color, -1)
                            
                            if i == track_idx:
                                cv2.putText(frame, f"Tracked: {person['confidence']:.2f}", 
                                          (x1, y1-10), self.font, 0.5, color, 2)
                        
                        tracked_cx, tracked_cy = tracked_person['center']
                        self.last_person_center = (tracked_cx, tracked_cy)
                        
                        movement = self.get_movement_suggestion(
                            tracked_cx, frame_width,
                            tracked_person['box_height'], frame_height
                        )
                        
                        cv2.putText(frame, movement, (frame_width//2 - 60, frame_height - 20),
                                  self.font, 0.8, (0, 255, 0), 2)
                
                cv2.imshow("Person Tracker", frame)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('r'):
                    self.last_person_center = None
                    self.tracking_lost_frames = 0
                    print("Tracking reset")
        
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        except Exception as e:
            print(f"Error during tracking: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("Cleaning up...")
            if cap is not None:
                cap.release()
            if camera is not None:
                try:
                    camera.stop()
                    camera.close()
                except:
                    pass
            if self.use_hailo and self.hailo_network_group:
                try:
                    self.hailo_network_group.release()
                except:
                    pass
            cv2.destroyAllWindows()
            
            if frame_count > 0:
                avg_fps = frame_count/(time.time()-start_time)
                avg_inference = (total_inference_time / frame_count) * 1000
                print(f"Session stats - Avg FPS: {avg_fps:.1f}, Avg Inference: {avg_inference:.1f}ms")

    def export_model_to_onnx(self):
        if hasattr(self, 'model'):
            print("Exporting YOLO model to ONNX...")
            self.model.export(format='onnx', imgsz=640)
            print("ONNX export complete: yolov8n.onnx")
            print("\nNext steps for Hailo compilation:")
            print("1. hailo parser onnx yolov8n.onnx --hw-arch hailo8")
            print("2. hailo optimize yolov8n.har")
            print("3. hailo compiler yolov8n_optimized.har")
        else:
            print("No YOLO model available for export")

    def __del__(self):
        if self.use_hailo and self.hailo_network_group:
            try:
                self.hailo_network_group.release()
            except:
                pass

def main():
    print("Person Tracker with Hailo Acceleration")
    print("=====================================")
    
    hef_exists = os.path.exists("yolov8n.hef")
    print(f"HEF file available: {hef_exists}")
    
    if not hef_exists:
        print("\nTo enable Hailo acceleration:")
        print("1. Install Hailo Dataflow Compiler")
        print("2. Export ONNX model (will be done automatically)")
        print("3. Compile ONNX to HEF using Hailo tools")
        print("4. Place yolov8n.hef in the same directory\n")
    
    tracker = PersonTracker(use_hailo=hef_exists and HAILO_AVAILABLE)
    
    if not os.path.exists("yolov8n.onnx"):
        tracker.export_model_to_onnx()
    
    tracker.track_person()

if __name__ == "__main__":
    main()
