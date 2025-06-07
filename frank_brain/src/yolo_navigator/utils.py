from hailo import Model
import numpy as np
import cv2

def load_model(model_path: str) -> Model:
    model = Model(model_path)
    model.load()
    return model

def preprocess_image(image: np.ndarray, input_size: tuple) -> np.ndarray:
    image_resized = cv2.resize(image, input_size)
    image_normalized = image_resized / 255.0
    return image_normalized.astype(np.float32)

def postprocess_detections(detections: list, confidence_threshold: float = 0.6) -> list:
    filtered_detections = []
    for detection in detections:
        if detection['confidence'] >= confidence_threshold:
            filtered_detections.append(detection)
    return filtered_detections

def run_inference(model: Model, image: np.ndarray) -> list:
    preprocessed_image = preprocess_image(image, (model.input_shape[1], model.input_shape[2]))
    detections = model.predict(preprocessed_image)
    return postprocess_detections(detections)