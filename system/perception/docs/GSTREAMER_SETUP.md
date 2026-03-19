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
python -u -m src.main --config configs/test-video-blue.yaml --source "video:$(pwd)/tests/test_run.mp4" --comms stdout --no-gui
```

## Docker

The Docker image uses `python3-opencv` from Debian apt, which is built with GStreamer. No extra steps needed when running in Docker.

## UDP Stream Input

Perception now accepts direct stream URIs for `camera.source`, including `udp://`, `tcp://`, `rtsp://`, `http://`, and `https://`.

Example config:

```yaml
camera:
  source: udp://0.0.0.0:5000
  backend: gstreamer

debug_stream:
  enabled: true
  host: 0.0.0.0
  port: 8081
```

For a Docker container running on your laptop, publish both ports:

```bash
docker run --rm \
  -p 5000:5000/udp \
  -p 8081:8081 \
  perception \
  python3 -m src.main --mode production --config configs/docker-udp.yaml
```

Then open [http://localhost:8081](http://localhost:8081) to view the ROI overlay and predicted vector in a browser.
