#!/bin/bash
# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

echo "Setting up Person Tracker with virtual environment..."
# Activate virtual environment
echo "Activating virtual environment..."
source .venv/bin/activate

# Check if the model exists, if not download it
if [ ! -f "yolov8n.pt" ]; then
    echo "Downloading YOLOv8n model..."
    python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
fi

# Ensure the main Python script exists
if [ ! -f "pi5_person_tracker.py" ]; then
    echo "Creating pi5_person_tracker.py from the pasted code..."
    if [ -f "paste.txt" ]; then
        cp paste.txt pi5_person_tracker.py
        echo "Created pi5_person_tracker.py from paste.txt"
    else
        echo "Warning: pi5_person_tracker.py not found and paste.txt not available!"
        echo "Please ensure your main Python script is named pi5_person_tracker.py"
    fi
fi

# Verify picamera2 is accessible in the virtual environment
echo "Verifying picamera2 is accessible..."
python -c "import picamera2; print('? picamera2 is available')" || echo "?? Warning: picamera2 import failed"

# Run the person tracker script
echo "Launching person tracker..."
python pi5_person_tracker.py

# Deactivate virtual environment when done
deactivate
echo "Done!"
