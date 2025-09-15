#!/bin/bash
# UPDATED Installation Script for Human Detection System
# Raspberry Pi Zero 2W + Camera V2 + SIM7600 G-H
# FIXED: Dependencies updated for 2024/2025 compatibility

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

log "Installing FRED3 Human Detection System for $ACTUAL_USER..."

# Fix broken packages first
log "Fixing any broken packages..."
apt --fix-broken install -y || true
dpkg --configure -a || true

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

# Install UPDATED system dependencies 
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
    libgtk-3-0 \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    libjpeg62-turbo-dev \
    libtiff5-dev \
    libgtk2.0-dev \
    libxvidcore-dev \
    libx264-dev \
    libtbb2 \
    libtbb-dev \
    libdc1394-22-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libopenblas-dev \
    libopenblas-base \
    ffmpeg \
    libsm6 \
    libxext6 \
    libfontconfig1 \
    libxrender1 \
    libgl1-mesa-glx

# Remove problematic opencv if installed
log "Removing any existing OpenCV installations..."
pip3 uninstall -y opencv-python opencv-contrib-python opencv-python-headless || true
apt remove -y python3-opencv || true

# Enable camera
log "Enabling camera..."
if command -v raspi-config &> /dev/null; then
    raspi-config nonint do_camera 0
else
    warn "raspi-config not found, manually enable camera"
fi

# Enable serial for SIM7600
log "Configuring serial..."
if command -v raspi-config &> /dev/null; then
    raspi-config nonint do_serial 0
    systemctl disable hciuart || true
else
    warn "raspi-config not found, skipping serial config"
fi

# Boot config - handle different Pi OS versions
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
    if ! grep -q "camera_auto_detect=1" "$CONFIG_FILE"; then
        echo "camera_auto_detect=1" >> "$CONFIG_FILE"
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
python -m pip install --upgrade pip setuptools wheel
"

# Install UPDATED Python packages for better ARM compatibility
log "Installing Python packages with improved ARM support..."
sudo -u $ACTUAL_USER bash -c "
source venv/bin/activate

# Install numpy first - CRITICAL foundation
pip install numpy==1.24.3

# Use opencv-python-headless for better Pi compatibility
pip install opencv-python-headless==4.8.0.76

# Try modern PyTorch first, fallback to alternative if needed
if pip install torch==2.0.1 torchvision==0.15.2 --index-url https://download.pytorch.org/whl/cpu; then
    echo '✅ PyTorch 2.0 installed successfully'
    # Install ultralytics with PyTorch
    pip install ultralytics==8.0.20
else
    echo '⚠️ PyTorch failed, using MediaPipe as alternative'
    pip install mediapipe==0.10.7
fi

# Install other required packages
pip install pyserial==3.5
pip install pillow==10.0.1  
pip install pyyaml==6.0
pip install requests==2.31.0
pip install tqdm==4.66.1

# Optional: Install matplotlib for debugging
pip install matplotlib==3.7.2 || echo 'matplotlib failed, skipping'
"

# Verify installation
log "Verifying Python package installation..."
sudo -u $ACTUAL_USER bash -c "
source venv/bin/activate
python -c 'import cv2; print(f\"✅ OpenCV version: {cv2.__version__}\")' || echo '❌ OpenCV failed'
python -c 'import numpy; print(f\"✅ NumPy version: {numpy.__version__}\")' || echo '❌ NumPy failed'

# Check if PyTorch is available
if python -c 'import torch; print(f\"✅ PyTorch version: {torch.__version__}\")' 2>/dev/null; then
    echo 'Will use YOLOv5 for detection'
else
    echo 'Will use MediaPipe for detection (PyTorch not available)'
fi
"

# Set up AI model
log "Setting up detection model..."
cd "$PROJECT_DIR"
sudo -u $ACTUAL_USER mkdir -p models

# Try to download YOLOv5 model if PyTorch is available
sudo -u $ACTUAL_USER bash -c "
source venv/bin/activate
if python -c 'import torch' 2>/dev/null; then
    cd models
    if [ ! -f 'yolov5s.pt' ]; then
        echo 'Downloading YOLOv5 model...'
        for i in {1..3}; do
            wget -q --timeout=60 https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt && break
            echo \"Download attempt \$i failed, retrying...\"
            sleep 5
        done
        
        if [ ! -f 'yolov5s.pt' ]; then
            echo '❌ Failed to download YOLOv5 model'
        else
            echo '✅ YOLOv5 model downloaded'
        fi
    fi
else
    echo 'Skipping YOLOv5 download (using MediaPipe instead)'
fi
"

# Create UPDATED Python files with fallback detection
log "Creating Python application files..."
cd "$PROJECT_DIR"

# Create main.py with updated logic
cat > "$PROJECT_DIR/src/main.py" << 'EOF'
#!/usr/bin/env python3
import time
import json
import logging
import signal
import sys
import os
from datetime import datetime

# Try importing detection modules
try:
    from detector import HumanDetector
    DETECTOR_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Detector import failed: {e}")
    DETECTOR_AVAILABLE = False

try:
    from sms_handler import SMSHandler
    SMS_AVAILABLE = True
except ImportError as e:
    print(f"Warning: SMS handler import failed: {e}")
    SMS_AVAILABLE = False

try:
    from camera_handler import CameraHandler
    CAMERA_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Camera handler import failed: {e}")
    CAMERA_AVAILABLE = False

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
        self.logger.info("FRED3 Human Detection System initialized")
    
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
            handlers
