# GStreamer Setup for Perception

The perception node uses GStreamer for video capture when running with video files or when `camera.backend: gstreamer` is set. Standard `opencv-python` from pip does **not** include GStreamer support.

## 1. Install OpenCV with GStreamer (Python)

```bash
# Remove standard OpenCV (conflicts with custom build)
pip uninstall opencv-python opencv-python-headless -y

# Install OpenCV built with GStreamer support
pip install opencv-python-custom-gst
```

## 2. Install GStreamer System Packages (Linux)

```bash
# Ubuntu / Debian
sudo apt update
sudo apt install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav
```

## 3. Verify GStreamer Support

```bash
python -c "import cv2; info=cv2.getBuildInformation(); print('GStreamer: YES' in info)"
# Should print True
```

## 4. Test Video Capture

```bash
python -u -m src.main --source "video:$(pwd)/tests/test_run.mp4" --comms stdout --no-gui
```

## Docker

The Docker image uses `python3-opencv` from Debian apt, which is built with GStreamer. No extra steps needed when running in Docker.
