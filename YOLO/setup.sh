#!/bin/bash
# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

echo "Setting up Person Tracker with virtual environment..."

# Make sure python3-venv is installed
if ! dpkg -l | grep -q python3-venv; then
    echo "Installing python3-venv..."
    sudo apt-get update
    sudo apt-get install -y python3-venv
fi

# Install picamera2 and dependencies system-wide
echo "Installing picamera2 and dependencies system-wide..."
sudo apt-get update
sudo apt-get install -y libcamera-apps python3-libcamera python3-picamera2
sudo apt-get install -y python3-pyqt5 python3-prctl libatlas-base-dev ffmpeg libopenjp2-7

# Check for camera interface
echo "Checking camera interface..."
if command_exists raspi-config; then
    if ! raspi-config nonint get_camera | grep -q "1"; then
        echo "Enabling camera interface..."
        sudo raspi-config nonint do_camera 0
    fi
fi

# Ensure user is in the video group
if ! groups | grep -q "video"; then
    echo "Adding user to video group..."
    sudo usermod -a -G video $USER
    echo "You may need to log out and back in for group changes to take effect"
fi

# Remove old virtual environment if it exists
#if [ -d ".venv" ]; then
#    echo "Removing old virtual environment..."
#    rm -rf .venv
#fi

# Create virtual environment WITH system-site-packages
#echo "Creating virtual environment with system-site-packages..."
#python3 -m venv --system-site-packages .venv

# Activate virtual environment
echo "Activating virtual environment..."
source .venv/bin/activate

# Create requirements.txt with necessary packages (excluding picamera2)
echo "Creating requirements.txt with necessary packages..."
cat > requirements.txt << EOF
ultralytics>=8.0.0
opencv-python>=4.8.0
numpy>=1.24.0
pillow>=10.0.0
matplotlib>=3.7.0
EOF

echo "Installing dependencies from requirements.txt..."
pip install --upgrade pip
pip install -r requirements.txt

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