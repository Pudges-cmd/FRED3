#!/bin/bash
# Complete Installation Script for Human Detection System Potang ina mo ninya
# Raspberry Pi Zero 2W + Camera V2 + SIM7600 G-H TAKE ME OUT THE GAME REF AYAW KO NA 

set -e
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log() { echo -e "${GREEN}[$(date +'%H:%M:%S')] $1${NC}"; }
warn() { echo -e "${YELLOW}[$(date +'%H:%M:%S')] $1${NC}"; }
error() { echo -e "${RED}[$(date +'%H:%M:%S')] $1${NC}"; }

if [[ $EUID -ne 0 ]]; then
   error "Run with sudo: sudo ./install.sh"
   exit 1
fi

ACTUAL_USER=${SUDO_USER:-$USER}
PROJECT_DIR="/home/$ACTUAL_USER/human-detection-system"

log "Installing Human Detection System for $ACTUAL_USER..."

# Update system
log "Updating system..."
apt update && apt upgrade -y

# Install dependencies
log "Installing dependencies..."
apt install -y python3 python3-pip python3-venv git wget build-essential \
    cmake pkg-config libjpeg-dev libpng-dev libavcodec-dev libavformat-dev \
    libswscale-dev libv4l-dev libatlas-base-dev gfortran python3-opencv

# Enable camera
log "Enabling camera..."
raspi-config nonint do_camera 0

# Enable serial for SIM7600
log "Configuring serial..."
raspi-config nonint do_serial 0
systemctl disable hciuart

# Boot config
if ! grep -q "dtoverlay=disable-bt" /boot/config.txt; then
    echo "dtoverlay=disable-bt" >> /boot/config.txt
fi
if ! grep -q "gpu_mem=128" /boot/config.txt; then
    echo "gpu_mem=128" >> /boot/config.txt
fi

# Create directories
log "Creating project structure..."
mkdir -p "$PROJECT_DIR"/{logs,models,debug_images}
chown -R $ACTUAL_USER:$ACTUAL_USER "$PROJECT_DIR"

# Create virtual environment
log "Setting up Python environment..."
cd "$PROJECT_DIR"
sudo -u $ACTUAL_USER python3 -m venv venv

# Install Python packages
log "Installing Python packages (this takes time)..."
sudo -u $ACTUAL_USER bash -c "
source venv/bin/activate
pip install --upgrade pip
pip install torch==1.13.1 torchvision==0.14.1 --index-url https://download.pytorch.org/whl/cpu
pip install ultralytics==8.0.20
pip install opencv-python==4.6.0.66
pip install pyserial==3.5
pip install numpy==1.21.6
pip install pillow==9.5.0
pip install pyyaml==6.0
"

# Download YOLOv5 model
log "Downloading YOLOv5 model..."
cd "$PROJECT_DIR/models"
if [ ! -f "yolov5s.pt" ]; then
    sudo -u $ACTUAL_USER wget -q https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt
fi

# Create main Python files
log "Creating Python files..."

# main.py
cat > "$PROJECT_DIR/src/main.py" << 'EOF'
#!/usr/bin/env python3
import time
import json
import logging
import signal
import sys
import os
from datetime import datetime
from detector import HumanDetector
from sms_handler import SMSHandler
from camera_handler import CameraHandler

class HumanDetectionSystem:
    def __init__(self, config_path="config/settings.json"):
        self.running = False
        self.config = self.load_config(config_path)
        self.setup_logging()
        self.camera = None
        self.detector = None
        self.sms_handler = None
        self.last_sms_time = 0
        self.detection_count = 0
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        self.logger.info("Human Detection System initialized")
    
    def load_config(self, config_path):
        try:
            with open(config_path, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"Config file {config_path} not found!")
            sys.exit(1)
    
    def setup_logging(self):
        os.makedirs("logs", exist_ok=True)
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('logs/detection.log'),
                logging.StreamHandler(sys.stdout)
            ]
        )
        self.logger = logging.getLogger(__name__)
    
    def initialize_components(self):
        try:
            self.logger.info("Initializing components...")
            self.camera = CameraHandler()
            self.detector = HumanDetector()
            self.sms_handler = SMSHandler(self.config['sms']['phone_numbers'])
            return True
        except Exception as e:
            self.logger.error(f"Failed to initialize: {e}")
            return False
    
    def should_send_sms(self):
        return time.time() - self.last_sms_time >= self.config['sms']['sms_cooldown']
    
    def process_detections(self, detections):
        if len(detections) > 0:
            self.detection_count += 1
            self.logger.info(f"Detected {len(detections)} person(s)")
            
            if self.sms_handler and self.should_send_sms():
                message = f"🚨 HUMAN DETECTED 🚨\n{len(detections)} person(s)\nTime: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
                try:
                    self.sms_handler.send_sms(message)
                    self.last_sms_time = time.time()
                    self.logger.info("SMS sent")
                except Exception as e:
                    self.logger.error(f"SMS failed: {e}")
    
    def run(self):
        if not self.initialize_components():
            return
        
        self.running = True
        self.logger.info("Detection system running...")
        
        try:
            while self.running:
                frame = self.camera.capture_frame()
                if frame is not None:
                    detections = self.detector.detect_humans(frame)
                    self.process_detections(detections)
                time.sleep(self.config['detection']['detection_interval'])
        except Exception as e:
            self.logger.error(f"Error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        self.logger.info("Shutting down...")
        if self.camera: self.camera.cleanup()
        if self.detector: self.detector.cleanup()
        if self.sms_handler: self.sms_handler.cleanup()
    
    def signal_handler(self, signum, frame):
        self.logger.info("Received shutdown signal")
        self.running = False

if __name__ == "__main__":
    system = HumanDetectionSystem()
    system.run()
EOF

# detector.py
cat > "$PROJECT_DIR/src/detector.py" << 'EOF'
#!/usr/bin/env python3
import torch
import numpy as np
import cv2
import logging
import os
import urllib.request

class HumanDetector:
    def __init__(self, model_path="models/yolov5s.pt", confidence=0.5):
        self.model_path = model_path
        self.confidence = confidence
        self.model = None
        self.logger = logging.getLogger(__name__)
        self.PERSON_CLASS_ID = 0
        self._load_model()
    
    def _load_model(self):
        try:
            self.logger.info("Loading YOLOv5 model...")
            self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=self.model_path, force_reload=False, verbose=False)
            self.model.cpu()
            self.model.conf = self.confidence
            self.model.eval()
            self.logger.info("YOLOv5 model loaded")
        except Exception as e:
            self.logger.error(f"Failed to load model: {e}")
            raise
    
    def detect_humans(self, image):
        if self.model is None:
            return []
        
        try:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            with torch.no_grad():
                results = self.model(image_rgb, size=640)
            
            detections = []
            predictions = results.pandas().xyxy[0]
            person_detections = predictions[predictions['class'] == self.PERSON_CLASS_ID]
            
            for _, detection in person_detections.iterrows():
                detections.append({
                    'bbox': [int(detection['xmin']), int(detection['ymin']), int(detection['xmax']), int(detection['ymax'])],
                    'confidence': float(detection['confidence'])
                })
            
            return detections
        except Exception as e:
            self.logger.error(f"Detection error: {e}")
            return []
    
    def cleanup(self):
        if self.model:
            del self.model
            self.model = None

if __name__ == "__main__":
    detector = HumanDetector()
    print("Detector test completed")
    detector.cleanup()
EOF

# sms_handler.py
cat > "$PROJECT_DIR/src/sms_handler.py" << 'EOF'
#!/usr/bin/env python3
import serial
import time
import logging
import re

class SMSHandler:
    def __init__(self, phone_numbers, device_path="/dev/ttyUSB2", baud_rate=115200):
        self.phone_numbers = phone_numbers
        self.device_path = device_path
        self.baud_rate = baud_rate
        self.serial_connection = None
        self.logger = logging.getLogger(__name__)
        self._initialize_connection()
    
    def _initialize_connection(self):
        try:
            self.logger.info(f"Connecting to SIM7600 at {self.device_path}")
            self.serial_connection = serial.Serial(
                port=self.device_path,
                baudrate=self.baud_rate,
                timeout=10
            )
            time.sleep(2)
            if self._setup_module():
                self.logger.info("SIM7600 initialized")
            else:
                raise Exception("SIM7600 setup failed")
        except Exception as e:
            self.logger.error(f"SIM7600 connection failed: {e}")
            self.serial_connection = None
    
    def _send_at_command(self, command, expected="OK", timeout=5.0):
        if not self.serial_connection:
            return False, "No connection"
        
        try:
            self.serial_connection.reset_input_buffer()
            self.serial_connection.write(f"{command}\r\n".encode())
            
            start_time = time.time()
            response = ""
            
            while time.time() - start_time < timeout:
                if self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.read(self.serial_connection.in_waiting)
                    response += data.decode('utf-8', errors='ignore')
                    if expected in response:
                        return True, response.strip()
                    if "ERROR" in response:
                        return False, response.strip()
                time.sleep(0.1)
            
            return False, f"Timeout: {command}"
        except Exception as e:
            return False, str(e)
    
    def _setup_module(self):
        commands = [("AT", "OK"), ("ATE0", "OK"), ("AT+CMGF=1", "OK"), ("AT+CSCS=\"GSM\"", "OK")]
        for cmd, exp in commands:
            success, _ = self._send_at_command(cmd, exp)
            if not success:
                return False
            time.sleep(0.5)
        return True
    
    def send_sms(self, message):
        if not self.phone_numbers or not self.serial_connection:
            return False
        
        success_count = 0
        for number in self.phone_numbers:
            if self._send_sms_to_number(number, message):
                success_count += 1
                self.logger.info(f"SMS sent to {number}")
            time.sleep(1)
        
        return success_count > 0
    
    def _send_sms_to_number(self, phone_number, message):
        try:
            if len(message) > 160:
                message = message[:157] + "..."
            
            success, _ = self._send_at_command(f'AT+CMGS="{phone_number}"', ">", 10)
            if not success:
                return False
            
            self.serial_connection.write((message + chr(26)).encode())
            
            start_time = time.time()
            response = ""
            while time.time() - start_time < 30:
                if self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.read(self.serial_connection.in_waiting)
                    response += data.decode('utf-8', errors='ignore')
                    if "+CMGS:" in response and "OK" in response:
                        return True
                    if "ERROR" in response:
                        return False
                time.sleep(0.5)
            return False
        except Exception as e:
            self.logger.error(f"SMS error: {e}")
            return False
    
    def cleanup(self):
        if self.serial_connection:
            self.serial_connection.close()
            self.serial_connection = None

if __name__ == "__main__":
    handler = SMSHandler(["+1234567890"])
    print("SMS handler test completed")
    handler.cleanup()
EOF

# camera_handler.py
cat > "$PROJECT_DIR/src/camera_handler.py" << 'EOF'
#!/usr/bin/env python3
import cv2
import numpy as np
import logging
import time

class CameraHandler:
    def __init__(self, resolution=(640, 480), framerate=2):
        self.resolution = resolution
        self.framerate = framerate
        self.camera = None
        self.logger = logging.getLogger(__name__)
        self.is_initialized = False
        self._initialize_camera()
    
    def _initialize_camera(self):
        try:
            self.logger.info("Initializing camera...")
            self.camera = cv2.VideoCapture(0)
            
            if not self.camera.isOpened():
                raise Exception("Camera not found")
            
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            self.camera.set(cv2.CAP_PROP_FPS, self.framerate)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            if self._test_capture():
                self.is_initialized = True
                self.logger.info("Camera initialized successfully")
            else:
                raise Exception("Camera test failed")
        except Exception as e:
            self.logger.error(f"Camera initialization failed: {e}")
            self.cleanup()
            raise
    
    def _test_capture(self):
        try:
            for i in range(3):
                ret, frame = self.camera.read()
                if ret and frame is not None and frame.shape[0] > 0:
                    return True
                time.sleep(0.5)
            return False
        except Exception:
            return False
    
    def capture_frame(self):
        if not self.is_initialized or not self.camera:
            return None
        
        try:
            ret, frame = self.camera.read()
            if ret and frame is not None and frame.shape[0] > 0:
                return frame
            return None
        except Exception as e:
            self.logger.error(f"Capture error: {e}")
            return None
    
    def cleanup(self):
        if self.camera:
            self.camera.release()
            self.camera = None
            self.is_initialized = False

if __name__ == "__main__":
    camera = CameraHandler()
    frame = camera.capture_frame()
    if frame is not None:
        print(f"Camera test successful: {frame.shape}")
    else:
        print("Camera test failed")
    camera.cleanup()
EOF

# Create config file
log "Creating configuration..."
cat > "$PROJECT_DIR/config/settings.json" << 'EOF'
{
  "detection": {
    "detection_confidence": 0.5,
    "detection_interval": 2.0
  },
  "sms": {
    "phone_numbers": [
      "+1234567890"
    ],
    "sms_device_path": "/dev/ttyUSB2",
    "sms_cooldown": 300
  },
  "camera": {
    "resolution": [640, 480],
    "framerate": 2
  }
}
EOF

# Create script files
log "Creating control scripts..."
mkdir -p "$PROJECT_DIR/scripts"

# start.sh
cat > "$PROJECT_DIR/scripts/start.sh" << 'EOF'
#!/bin/bash
cd "$(dirname "$0")/.."
source venv/bin/activate
echo "Starting human detection system..."
python src/main.py
EOF

# stop.sh
cat > "$PROJECT_DIR/scripts/stop.sh" << 'EOF'
#!/bin/bash
pkill -f "python.*main.py"
echo "Human detection system stopped"
EOF

# status.sh
cat > "$PROJECT_DIR/scripts/status.sh" << 'EOF'
#!/bin/bash
if pgrep -f "python.*main.py" > /dev/null; then
    echo "✅ Running"
else
    echo "❌ Stopped"
fi
echo "Camera: $(vcgencmd get_camera)"
echo "SIM7600: $(ls /dev/ttyUSB* 2>/dev/null || echo 'Not found')"
if [ -f ../logs/detection.log ]; then
    echo "Recent logs:"
    tail -3 ../logs/detection.log
fi
EOF

chmod +x "$PROJECT_DIR/scripts"/*.sh

# Create systemd service
log "Creating service..."
cat > /etc/systemd/system/human-detection.service << EOF
[Unit]
Description=Human Detection System
After=network.target

[Service]
Type=simple
User=$ACTUAL_USER
WorkingDirectory=$PROJECT_DIR
Environment=PATH=$PROJECT_DIR/venv/bin
ExecStart=$PROJECT_DIR/venv/bin/python $PROJECT_DIR/src/main.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
chown -R $ACTUAL_USER:$ACTUAL_USER "$PROJECT_DIR"

log "✅ INSTALLATION COMPLETE!"
echo ""
echo "🔧 NEXT STEPS:"
echo "1. sudo reboot"
echo "2. cd $PROJECT_DIR"  
echo "3. nano config/settings.json  # ADD YOUR PHONE NUMBER"
echo "4. ./scripts/start.sh"
echo ""
echo "🚀 Auto-start on boot: sudo systemctl enable human-detection"
echo "📜 Logs: tail -f logs/detection.log"
echo ""
echo "⚡ FRED3 WILL START AUTOMATICALLY ON BOOT AFTER ENABLING SERVICE!"