#!/usr/bin/env python3
"""
Minimal red-tape finder for Pi Camera Module 2 (Picamera2 + OpenCV).

Pipeline (standard OpenCV, see docs):
  BGR frame → HSV → inRange (red has two hue bands) → findContours → largest contour
  → boundingRect (left/right edges) + moments (centroid)

  - Contours: https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html
  - Moments / centroid: https://docs.opencv.org/4.x/d3/dc0/group__imgproc__shape.html#ga556a180f043780e241de946b872a4920

Run (Pi with display):
    python test_perception_pi.py

Keys:  q quit   s save frame   [ ] ROI   - = min area
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np
from picamera2 import Picamera2

# Red in HSV: hue wraps at 0/179 — two ranges (tune on your lighting).
RED_LO1 = np.array([0, 100, 70], np.uint8)
RED_HI1 = np.array([12, 255, 255], np.uint8)
RED_LO2 = np.array([168, 100, 70], np.uint8)
RED_HI2 = np.array([179, 255, 255], np.uint8)


def find_tape(frame_bgr: np.ndarray, roi_top_ratio: float, min_area: float):
    """
    Returns dict with roi_y, bbox (full-frame x,y,w,h) or None, centroid (cx,cy) or None.
    """
    h, w = frame_bgr.shape[:2]
    roi_y = int(h * roi_top_ratio)
    roi = frame_bgr[roi_y:, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.bitwise_or(
        cv2.inRange(hsv, RED_LO1, RED_HI1),
        cv2.inRange(hsv, RED_LO2, RED_HI2),
    )
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return {"roi_y": roi_y, "bbox": None, "centroid": None, "mask": mask}

    cnt = max(contours, key=cv2.contourArea)
    if cv2.contourArea(cnt) < min_area:
        return {"roi_y": roi_y, "bbox": None, "centroid": None, "mask": mask}

    x, y, bw, bh = cv2.boundingRect(cnt)
    bbox = (x, roi_y + y, bw, bh)

    m = cv2.moments(cnt)
    if m["m00"] <= 1e-6:
        cx, cy = x + bw // 2, roi_y + y + bh // 2
    else:
        cx = int(m["m10"] / m["m00"])
        cy = int(m["m01"] / m["m00"]) + roi_y

    return {
        "roi_y": roi_y,
        "bbox": bbox,
        "centroid": (cx, cy),
        "mask": mask,
    }


def draw(frame_bgr: np.ndarray, d: dict) -> np.ndarray:
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    roi_y = d["roi_y"]

    # Light mask tint (optional visual only)
    tint = np.zeros_like(out)
    tint[roi_y:, :, 2] = np.where(d["mask"] > 0, 120, 0)  # red channel in BGR
    cv2.addWeighted(tint, 0.25, out, 0.75, 0, out)

    cv2.line(out, (0, roi_y), (w - 1, roi_y), (160, 160, 160), 1)
    cv2.line(out, (w // 2, roi_y), (w // 2, h - 1), (255, 255, 255), 1)

    if d["bbox"] is None or d["centroid"] is None:
        cv2.putText(out, "no tape", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return out

    x, y, bw, bh = d["bbox"]
    cx, cy = d["centroid"]

    # Edges of tape (vertical sides of bounding box)
    cv2.line(out, (x, y), (x + bw, y), (0, 255, 0), 2)
    cv2.line(out, (x, y + bh), (x + bw, y + bh), (0, 255, 0), 2)
    cv2.line(out, (x, y), (x, y + bh), (0, 255, 0), 2)
    cv2.line(out, (x + bw, y), (x + bw, y + bh), (0, 255, 0), 2)

    # Center of the tape (centroid)
    cv2.drawMarker(out, (cx, cy), (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=16, thickness=2)

    err = (cx - w / 2.0) / (w / 2.0)
    cv2.putText(
        out,
        f"cx={cx}  err={err:+.2f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
    )
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Simple red tape: contour + bbox + centroid")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--roi", type=float, default=0.5, help="Fraction of frame height to skip at top")
    ap.add_argument("--min-area", type=float, default=80.0, help="Min contour area (pixels)")
    args = ap.parse_args()

    cam = Picamera2()
    cam.configure(
        cam.create_preview_configuration(
            main={"size": (args.width, args.height), "format": "RGB888"},
            buffer_count=2,
        )
    )
    cam.start()

    roi_ratio = args.roi
    min_area = args.min_area
    print("q=quit  s=save  [/]=ROI  -/==min-area")

    try:
        while True:
            rgb = cam.capture_array("main")
            frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

            d = find_tape(frame, roi_ratio, min_area)
            vis = draw(frame, d)

            cv2.imshow("tape", vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                cv2.imwrite("saved_frame.jpg", frame)
                print("saved saved_frame.jpg")
            elif key == ord("["):
                roi_ratio = max(0.0, roi_ratio - 0.05)
                print(f"roi={roi_ratio:.2f}")
            elif key == ord("]"):
                roi_ratio = min(0.95, roi_ratio + 0.05)
                print(f"roi={roi_ratio:.2f}")
            elif key == ord("-"):
                min_area = max(10.0, min_area - 50)
                print(f"min_area={min_area:.0f}")
            elif key == ord("="):
                min_area += 50
                print(f"min_area={min_area:.0f}")
    finally:
        cam.stop()
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
