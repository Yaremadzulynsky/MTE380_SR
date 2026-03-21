#!/usr/bin/env python3
"""
Red / blue tape test — Pi Camera Module 2 (Picamera2 + OpenCV).

Why “red and blue look switched”
--------------------------------
The main stream is often labeled ``RGB888``, but on many Pis the numpy buffer is
**BGR channel order**. OpenCV’s ``BGR2HSV`` expects BGR; ``RGB2HSV`` expects RGB.
Using the wrong pair swaps R/B in hue space (red tape can read as blue).

Default here is ``--input bgr``, matching ``local/perception.py`` (``camera_channel_order="bgr"``).
Use ``--input rgb`` only if your buffer is true RGB order.

Pipeline
--------
  capture_array → HSV with the *matching* COLOR_*2HSV for your buffer
  → inRange (red + blue masks) → contours → bbox + centroid

Keys:  q quit   s save frame   [ ] ROI   - = min areas   c cycle input order
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np
from picamera2 import Picamera2

# Red: two hue bands (wraps at 0/179). Blue: single wedge. Tune under your lights.
RED_LO1 = np.array([0, 100, 70], np.uint8)
RED_HI1 = np.array([12, 255, 255], np.uint8)
RED_LO2 = np.array([168, 100, 70], np.uint8)
RED_HI2 = np.array([179, 255, 255], np.uint8)

BLUE_LO = np.array([95, 120, 70], np.uint8)
BLUE_HI = np.array([135, 255, 255], np.uint8)


def raw_to_bgr_and_hsv_roi(
    raw: np.ndarray,
    roi_top_ratio: float,
    input_order: str,
) -> tuple[np.ndarray, np.ndarray, int]:
    """
    raw: Picamera2 main buffer (RGB888 → RGB order, or set input_order='bgr' if not).

    Returns:
      bgr_full — BGR for drawing / imshow
      hsv_roi  — HSV of bottom ROI only (for detection)
      roi_y    — first row index of ROI in full frame
    """
    h, w = raw.shape[:2]
    roi_y = int(h * roi_top_ratio)
    roi_raw = raw[roi_y:, :]

    if input_order == "rgb":
        bgr_full = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        hsv_roi = cv2.cvtColor(roi_raw, cv2.COLOR_RGB2HSV)
    else:
        # Buffer already BGR (rare; use if red/blue still wrong with rgb)
        bgr_full = raw
        hsv_roi = cv2.cvtColor(roi_raw, cv2.COLOR_BGR2HSV)

    return bgr_full, hsv_roi, roi_y


def detect(
    hsv_roi: np.ndarray,
    full_w: int,
    roi_y: int,
    red_min_area: float,
    blue_min_area: float,
    t_ratio: float,
):
    red_mask = cv2.bitwise_or(
        cv2.inRange(hsv_roi, RED_LO1, RED_HI1),
        cv2.inRange(hsv_roi, RED_LO2, RED_HI2),
    )
    kernel = np.ones((5, 5), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    blue_mask = cv2.inRange(hsv_roi, BLUE_LO, BLUE_HI)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel, iterations=1)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    red_found = False
    red_error = 0.0
    t_junction = False
    bbox = None
    cx_px = full_w // 2
    cy_px = roi_y + hsv_roi.shape[0] // 2

    if contours:
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area >= red_min_area:
            red_found = True
            bx, by, bw, bh = cv2.boundingRect(largest)
            bbox = (bx, roi_y + by, bw, bh)
            t_junction = (bw / full_w) > t_ratio
            M = cv2.moments(largest)
            if M["m00"] > 1e-6:
                cx_raw = M["m10"] / M["m00"]
                red_error = max(-1.0, min(1.0, (cx_raw - full_w / 2.0) / (full_w / 2.0)))
                cx_px = int(cx_raw)
                cy_px = roi_y + int(M["m01"] / M["m00"])

    blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_found = any(cv2.contourArea(c) >= blue_min_area for c in blue_contours)

    # Pad masks to full height for overlay
    h_full = roi_y + hsv_roi.shape[0]
    red_full = np.zeros((h_full, full_w), np.uint8)
    blue_full = np.zeros((h_full, full_w), np.uint8)
    red_full[roi_y:, :] = red_mask
    blue_full[roi_y:, :] = blue_mask

    return {
        "red_found": red_found,
        "red_error": red_error,
        "blue_found": blue_found,
        "t_junction": t_junction,
        "roi_y": roi_y,
        "bbox": bbox,
        "cx_px": cx_px if red_found else None,
        "cy_px": cy_px if red_found else None,
        "red_mask_full": red_full,
        "blue_mask_full": blue_full,
        "all_contours": [(cv2.contourArea(c), cv2.boundingRect(c)) for c in contours],
    }


def draw(frame_bgr: np.ndarray, d: dict) -> np.ndarray:
    out = frame_bgr.copy()
    h, w = out.shape[:2]
    roi_y = d["roi_y"]

    overlay = out.copy()
    overlay[d["red_mask_full"] > 0] = (0, 0, 180)
    cv2.addWeighted(overlay, 0.3, out, 0.7, 0, out)

    cv2.line(out, (0, roi_y), (w - 1, roi_y), (180, 180, 180), 1)
    cv2.line(out, (w // 2, roi_y), (w // 2, h - 1), (255, 255, 255), 2)

    if d["red_found"] and d["bbox"]:
        bx, by, bw, bh = d["bbox"]
        cv2.rectangle(out, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        cv2.line(out, (bx, roi_y), (bx, h - 1), (0, 200, 0), 1)
        cv2.line(out, (bx + bw, roi_y), (bx + bw, h - 1), (0, 200, 0), 1)
        cx, cy = d["cx_px"], d["cy_px"]
        cv2.line(out, (w // 2, cy), (cx, cy), (0, 140, 255), 2)
        cv2.circle(out, (cx, cy), 8, (0, 0, 255), -1)

    for area, (bx, by, bw, bh) in d["all_contours"][:6]:
        fy = roi_y + by
        cv2.putText(
            out,
            f"{int(area)}",
            (bx, max(fy - 4, 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (200, 200, 0),
            1,
            cv2.LINE_AA,
        )

    lines = [
        f"red={'Y' if d['red_found'] else 'N'}  err={d['red_error']:+.3f}",
        f"blue={'Y' if d['blue_found'] else 'N'}  T={'Y' if d['t_junction'] else 'N'}",
    ]
    for i, txt in enumerate(lines):
        cv2.putText(out, txt, (10, 28 + i * 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--roi", type=float, default=0.5)
    ap.add_argument("--red-min-area", type=float, default=80.0)
    ap.add_argument("--blue-min-area", type=float, default=1500.0)
    ap.add_argument("--t-ratio", type=float, default=0.5)
    ap.add_argument(
        "--input",
        choices=("rgb", "bgr"),
        default="bgr",
        help="Channel order of capture_array (Pi: default bgr, matches local/perception.py).",
    )
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--debug-every", type=int, default=15)
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
    red_min_area = args.red_min_area
    input_order = args.input

    print(
        "q=quit  s=save  [/]=ROI  -/==red  c=input(rgb/bgr)  "
        f"input={input_order}",
    )

    frame_i = 0
    try:
        while True:
            frame_i += 1
            raw = cam.capture_array("main")
            bgr, hsv_roi, roi_y = raw_to_bgr_and_hsv_roi(raw, roi_ratio, input_order)
            h, w = bgr.shape[:2]

            d = detect(
                hsv_roi,
                w,
                roi_y,
                red_min_area,
                args.blue_min_area,
                args.t_ratio,
            )
            vis = draw(bgr, d)

            if args.debug and frame_i % max(1, args.debug_every) == 0:
                rp = int(np.count_nonzero(d["red_mask_full"]))
                bp = int(np.count_nonzero(d["blue_mask_full"]))
                print(
                    f"[{frame_i}] input={input_order}  red={d['red_found']} blue={d['blue_found']}  "
                    f"red_px={rp} blue_px={bp}",
                    flush=True,
                )

            cv2.imshow("perception test", vis)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                cv2.imwrite("saved_frame.jpg", bgr)
                print("saved saved_frame.jpg")
            elif key == ord("["):
                roi_ratio = max(0.0, roi_ratio - 0.05)
                print(f"roi={roi_ratio:.2f}")
            elif key == ord("]"):
                roi_ratio = min(0.95, roi_ratio + 0.05)
                print(f"roi={roi_ratio:.2f}")
            elif key == ord("-"):
                red_min_area = max(10.0, red_min_area - 50)
                print(f"red_min_area={red_min_area:.0f}")
            elif key == ord("="):
                red_min_area += 50
                print(f"red_min_area={red_min_area:.0f}")
            elif key == ord("c"):
                input_order = "bgr" if input_order == "rgb" else "rgb"
                print(f"input_order={input_order}  (HSV from {'RGB2HSV' if input_order=='rgb' else 'BGR2HSV'})")
    finally:
        cam.stop()
        cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
