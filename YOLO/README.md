# Tracker Tutorial
## Installation
Run the following script to automate the installation process:
```bash
./install.sh
```
### Running The Examples
When opening a new terminal session, ensure you have sourced the environment setup script:
```bash
source setup_env.sh
```
### Tracker example
```bash
python hailo_detection_tracker.py
```
To close the application, press `Ctrl+C`.

#### Running with Raspberry Pi Camera input:
```bash
python hailo_detection_tracker.py --input rpi
```

#### Running with USB camera input (webcam):
There are 2 ways:

Specify the argument `--input` to `usb`:
```bash
python hailo_detection_tracker.py --input usb
```

This will automatically detect the available USB camera (if multiple are connected, it will use the first detected).

Second way:

Detect the available camera using this script:
```bash
get-usb-camera
```
Run example using USB camera input - Use the device found by the previous script:
```bash
python hailo_detection_tracker.py --input /dev/video<X>
```

For additional options, execute:
```bash
python basic_pipelines/detection.py --help
```
