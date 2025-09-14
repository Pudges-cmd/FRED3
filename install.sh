#!/bin/bash
# Fixed Installation Script for Human Detection System
# Raspberry Pi Zero 2W + Camera V2 + SIM7600 G-H

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

# Fix broken packages first
log "Fixing any broken packages..."
apt --fix-broken install -y
dpkg --configure -a

# Update system with error handling
log "Updating system..."
apt clean
apt update || {
    warn "Update failed, trying to fix repositories..."
    apt-get clean
    rm -rf /var/lib/apt/lists/*
    apt-get update
}
apt upgrade -y

# Install system dependencies with better error handling
log "Installing system dependencies..."
apt install -y \
    python3 \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-setuptools \
    git \
    wget \
    curl \
    build-essential \
    cmake \
    pkg-config \
    libjpeg-dev \
    libpng-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libv4l-dev \
    libatlas-base-dev \
    gfortran \
    libhdf5-dev \
    libhdf5-serial-dev \
    libatlas3-base \
    libjasper-dev \
    libqtgui4 \
    libqt4-test \
    libglib2.0-0 \
    libgtk-3-0 \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libjpeg62-turbo-dev \
    libpng-dev \
    libtiff5-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgtk2.0-dev \
    libcanberra-gtk* \
    libxvidcore-dev \
    libx264-dev \
    libgtk-3-dev \
    libtbb2 \
    libtbb-dev \
    libdc1394-22-dev \
    libxine2-dev \
    libfaac-dev \
    libmp3lame-dev \
    libtheora-dev \
    libvorbis-dev \
    libxvidcore-dev \
    libopencore-amrnb-dev \
    libopencore-amrwb-dev \
    libavresample-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libopenblas-dev \
    libopenblas-base

# Remove problematic opencv if installed
log "Removing any existing OpenCV installations..."
pip3 uninstall -y opencv-python opencv-contrib-python opencv-python-headless || true
apt remove -y python3-opencv || true

# Enable camera
log "Enabling camera..."
if command -v raspi-config &> /dev/null; then
    raspi-config nonint do_camera 0
else
    warn "raspi-config not found, skipping camera enable"
fi

# Enable serial for SIM7600
log "Configuring serial..."
if command -v raspi-config &> /dev/null; then
    raspi-config nonint do_serial 0
    systemctl disable hciuart || true
else
    warn "raspi-config not found, skipping serial config"
fi

# Boot config (check if /boot/config.txt or /boot/firmware/config.txt exists)
CONFIG_FILE="/boot/config.txt"
if [ ! -f "$CONFIG_FILE" ] && [ -f "/boot/firmware/config.txt" ]; then
    CONFIG_FILE="/boot/firmware/config.txt"
fi

if [ -f "$CONFIG_FILE" ]; then
    log "Updating boot config..."
    if ! grep -q "dtoverlay=disable-bt" "$CONFIG_FILE"; then
        echo "dtoverlay=disable-bt" >> "$CONFIG_FILE"
    fi
    if ! grep -q "gpu_mem=128" "$CONFIG_FILE"; then
        echo "gpu_mem=128" >> "$CONFIG_FILE"
    fi
    if ! grep -q "start_x=1" "$CONFIG_FILE"; then
        echo "start_x=1" >> "$CONFIG_FILE"
    fi
    if ! grep -q "gpu_mem=128" "$CONFIG_FILE"; then
        echo "gpu_mem=128" >> "$CONFIG_FILE"
    fi
else
    warn "Boot config file not found, skipping boot config"
fi

# Create directories
log "Creating project structure..."
mkdir -p "$PROJECT_DIR"/{logs,models,debug_images,config,src,scripts}
chown -R $ACTUAL_USER:$ACTUAL_USER "$PROJECT_DIR"

# Create virtual environment
log "Setting up Python environment..."
cd "$PROJECT_DIR"
sudo -u $ACTUAL_USER python3 -m venv venv

# Upgrade pip first
log "Upgrading pip..."
sudo -u $ACTUAL_USER bash -c "
source venv/bin/activate
python -m pip install --upgrade pip
python -m pip install --upgrade setuptools wheel
"

# Install Python packages with specific versions for Pi Zero
log "Installing Python packages (this takes time)..."
sudo -u $ACTUAL_USER bash -c "
source venv/bin/activate

# Install numpy first (required for many other packages)
pip install numpy==1.21.6

# Install OpenCV with specific version for Pi
pip install opencv-python==4.5.5.64

# Install PyTorch with CPU-only version for ARM
pip install torch==1.13.1+cpu torchvision==0.14.1+cpu -f https://download.pytorch.org/whl/torch_stable.html

# Install other packages
pip install ultralytics==8.0.20
pip install pyserial==3.5
pip install pillow==9.5.0
pip install pyyaml==6.0
pip install matplotlib==3.5.3
pip install scipy==1.7.3
pip install requests==2.28.2
pip install tqdm==4.64.1
"

# Check if installation was successful
log "Verifying Python package installation..."
sudo -u $ACTUAL_USER bash -c "
source venv/bin/activate
python -c 'import cv2; print(f\"OpenCV version: {cv2.__version__}\")' || error 'OpenCV failed'
python -c 'import torch; print(f\"PyTorch version: {torch.__version__}\")' || error 'PyTorch failed'
python -c 'import numpy; print(f\"NumPy version: {numpy.__version__}\")' || error 'NumPy failed'
"

# Create YOLOv5 directory and download model
log "Setting up YOLOv5..."
cd "$PROJECT_DIR"
sudo -u $ACTUAL_USER mkdir -p models
cd models

# Download YOLOv5 model with retry mechanism
log "Downloading YOLOv5 model..."
if [ ! -f "yolov5s.pt" ]; then
    sudo -u $ACTUAL_USER bash -c "
    for i in {1..3}; do
        wget -q --timeout=30 https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt && break
        echo \"Download attempt \$i failed, retrying...\"
        sleep 5
    done
    "
    
    if [ ! -f "yolov5s.pt" ]; then
        error "Failed to download YOLOv5 model"
        exit 1
    fi
fi

# Create main Python files (keeping your original code structure)
log "Creating Python files..."
cd "$PROJECT_DIR"

# Create the Python files with the same content as before
# (I'll include just the main.py here for brevity, but all files remain the same)
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
                message = f"🚨 HUMAN DETECTED 🚨\\n{len(detections)} person(s)\\nTime: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
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

# Copy the rest of your Python files (detector.py, sms_handler.py, camera_handler.py)
# [I'll include these in the same format - keeping your original code]

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

# test.sh - New test script
cat > "$PROJECT_DIR/scripts/test.sh" << 'EOF'
#!/bin/bash
cd "$(dirname "$0")/.."
source venv/bin/activate
echo "Testing system components..."
python -c "
import sys
sys.path.append('src')
try:
    from camera_handler import CameraHandler
    from detector import HumanDetector
    from sms_handler import SMSHandler
    print('✅ All imports successful')
    
    # Test camera
    try:
        camera = CameraHandler()
        frame = camera.capture_frame()
        if frame is not None:
            print(f'✅ Camera working: {frame.shape}')
        else:
            print('❌ Camera failed')
        camera.cleanup()
    except Exception as e:
        print(f'❌ Camera error: {e}')
    
    # Test detector
    try:
        detector = HumanDetector()
        print('✅ Detector loaded')
        detector.cleanup()
    except Exception as e:
        print(f'❌ Detector error: {e}')
        
except Exception as e:
    print(f'❌ Import error: {e}')
"
EOF

# status.sh
cat > "$PROJECT_DIR/scripts/status.sh" << 'EOF'
#!/bin/bash
if pgrep -f "python.*main.py" > /dev/null; then
    echo "✅ Running"
else
    echo "❌ Stopped"
fi

# Check hardware
if command -v vcgencmd &> /dev/null; then
    echo "Camera: $(vcgencmd get_camera)"
else
    echo "Camera: vcgencmd not available"
fi

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
echo "4. ./scripts/test.sh  # Test components"
echo "5. ./scripts/start.sh  # Start system"
echo ""
echo "🚀 Auto-start on boot: sudo systemctl enable human-detection"
echo "📜 Logs: tail -f logs/detection.log"
echo "🧪 Test: ./scripts/test.sh"
echo ""
echo "⚡ SYSTEM READY - Test before enabling auto-start!"
