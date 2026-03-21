#!/usr/bin/env python3
"""
Perception tuner — Pi Camera Module 2 version.

Run on the Pi with a display (VNC or HDMI):
    python test_perception_pi.py

Controls:
    q           quit
    s           save current frame as 'saved_frame.jpg'
    [ / ]       decrease / increase roi_top_ratio by 0.05
    - / =       decrease / increase red_min_area by 50

What is shown:
    Grey line    : ROI boundary
    White line   : frame centre (zero error target)
    Green rect   : bounding box of detected red contour
    Green lines  : left/right tape edges
    Red dot      : centroid the robot steers toward
    Orange line  : error — centre to centroid
    Red overlay  : HSV red mask (semi-transparent)
    Yellow nums  : contour areas (tune red_min_area until only tape shows)
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np
from picamera2 import Picamera2


# ── HSV ranges (must match local/perception.py) ───────────────────────────────

RED_LO1 = np.array([  0, 120,  80], np.uint8)
RED_HI1 = np.array([ 10, 255, 255], np.uint8)
RED_LO2 = np.array([170, 120,  80], np.uint8)
RED_HI2 = np.array([179, 255, 255], np.uint8)

BLUE_LO = np.array([100, 150, 100], np.uint8)
BLUE_HI = np.array([130, 255, 255], np.uint8)


# ── Detection ─────────────────────────────────────────────────────────────────

def detect(frame, roi_top_ratio, red_min_area, blue_min_area, t_ratio):
    h, w  = frame.shape[:2]
    roi_y = int(h * roi_top_ratio)
    roi   = frame[roi_y:, :]
    hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv, RED_LO1, RED_HI1),
        cv2.inRange(hsv, RED_LO2, RED_HI2),
    )
    kernel   = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  kernel, iterations=1)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    blue_mask = cv2.inRange(hsv, BLUE_LO, BLUE_HI)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_found = False
    red_error = 0.0
    t_junction = False
    bbox = None
    cx_px = w // 2
    cy_px = roi_y + (h - roi_y) // 2

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area    = cv2.contourArea(largest)
        if area >= red_min_area:
            red_found = True
            bx, by, bw, bh = cv2.boundingRect(largest)
            bbox       = (bx, roi_y + by, bw, bh)
            t_junction = (bw / w) > t_ratio
            M = cv2.moments(largest)
            if M["m00"] > 1e-6:
                cx_raw = M["m10"] / M["m00"]
                red_error = max(-1.0, min(1.0, (cx_raw - w / 2.0) / (w / 2.0)))
                cx_px = int(cx_raw)
                cy_px = roi_y + by + bh // 2

    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_found = any(cv2.contourArea(c) >= blue_min_area for c in blue_contours)

    return {
        "red_found":    red_found,
        "red_error":    red_error,
        "blue_found":   blue_found,
        "t_junction":   t_junction,
        "roi_y":        roi_y,
        "bbox":         bbox,
        "cx_px":        cx_px if red_found else None,
        "cy_px":        cy_px if red_found else None,
        "red_mask":     red_mask,
        "all_contours": [(cv2.contourArea(c), cv2.boundingRect(c)) for c in contours],
    }


# ── Overlay drawing ───────────────────────────────────────────────────────────

def draw(frame, d):
    out  = frame.copy()
    h, w = out.shape[:2]
    roi_y = d["roi_y"]

    red_full = np.zeros((h, w), np.uint8)
    red_full[roi_y:, :] = d["red_mask"]
    overlay = out.copy()
    overlay[red_full > 0] = (0, 0, 180)
    cv2.addWeighted(overlay, 0.35, out, 0.65, 0, out)

    cv2.line(out, (0, roi_y), (w - 1, roi_y), (180, 180, 180), 1)
    cv2.line(out, (w // 2, roi_y), (w // 2, h - 1), (255, 255, 255), 2)

    if d["red_found"] and d["bbox"]:
        bx, by, bw, bh = d["bbox"]
        cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        cv2.line(out, (bx,      roi_y), (bx,      h - 1), (0, 200, 0), 1)
        cv2.line(out, (bx + bw, roi_y), (bx + bw, h - 1), (0, 200, 0), 1)
        cx, cy = d["cx_px"], d["cy_px"]
        cv2.line(out, (w // 2, cy), (cx, cy), (0, 140, 255), 2)
        cv2.circle(out, (cx, cy), 8, (0, 0, 255), -1)
        cv2.circle(out, (cx, cy), 8, (255, 255, 255), 2)

    for area, (bx, by, bw, bh) in d["all_contours"][:6]:
        fy = roi_y + by
        cv2.putText(out, f"{int(area)}", (bx, max(fy - 4, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 0), 1, cv2.LINE_AA)

    lines = [
        f"red={'Y' if d['red_found'] else 'N'}  err={d['red_error']:+.3f}",
        f"blue={'Y' if d['blue_found'] else 'N'}  T={'Y' if d['t_junction'] else 'N'}",
    ]
    for i, txt in enumerate(lines):
        cv2.putText(out, txt, (10, 28 + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)

    cv2.putText(out,
                f"roi={d['roi_y']/h:.2f}  [/]=ROI  -/==red_area",
                (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
    return out


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--width",         type=int,   default=640)
    ap.add_argument("--height",        type=int,   default=480)
    ap.add_argument("--roi",           type=float, default=0.5)
    ap.add_argument("--red-min-area",  type=float, default=80.0)
    ap.add_argument("--blue-min-area", type=float, default=1500.0)
    ap.add_argument("--t-ratio",       type=float, default=0.5)
    args = ap.parse_args()

    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(
        main={"size": (args.width, args.height), "format": "BGR888"},
        buffer_count=2,
    ))
    cam.start()

    roi_ratio    = args.roi
    red_min_area = args.red_min_area

    print("Controls: q=quit  s=save  [/]=ROI  -/==red area")

    while True:
        frame = cam.capture_array("main")
        d     = detect(frame, roi_ratio, red_min_area, args.blue_min_area, args.t_ratio)
        vis   = draw(frame, d)

        cv2.imshow("perception test", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            cv2.imwrite("saved_frame.jpg", frame)
            print("Saved saved_frame.jpg")
        elif key == ord("["):
            roi_ratio = max(0.0, roi_ratio - 0.05)
            print(f"roi_top_ratio = {roi_ratio:.2f}")
        elif key == ord("]"):
            roi_ratio = min(0.95, roi_ratio + 0.05)
            print(f"roi_top_ratio = {roi_ratio:.2f}")
        elif key == ord("-"):
            red_min_area = max(10, red_min_area - 50)
            print(f"red_min_area = {red_min_area:.0f}")
        elif key == ord("="):
            red_min_area += 50
            print(f"red_min_area = {red_min_area:.0f}")

    cam.stop()
    cam.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
