from __future__ import annotations

import cv2
from cv2.typing import MatLike
import numpy as np

def detect_line_geometry(
    jpeg_bytes: bytes,
    obstacle_far_roi_ratio: float = 0.45,
    obstacle_min_area_px: float = 200.0,
    obstacle_px_per_cm: float = 8.0,
    obstacle_distance_cm: float | None = None,
    camera_diag_fov_deg: float = 75.0,
    threshold: float = 100.0,
) -> tuple[
    float | None,
    float | None,
    float | None,
    float,
    float | None,
    float | None,
    float | None,
    MatLike,
]:
    """
    Detect line direction and curvature from a top-down camera frame.

    Returns:
        angle (rad)      : heading direction of the line
        curvature        : curvature of fitted polynomial
        offset           : horizontal offset from image center in range [-1, 1]
        confidence       : line detection confidence in range [0, 1]
        obstacle_width_cm: estimated obstacle width in cm (None if unavailable)
        obstacle_x_norm  : obstacle center in range [-1, 1] (None if unavailable)
        obstacle_frame_width_cm: estimated image frame width in cm (None if unavailable)
        debug_image       : visualization image
    """

    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    if frame is None:
        raise ValueError("Failed to decode JPEG bytes into an image")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if threshold <= 0.0:
        _, binary_full = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    else:
        _, binary_full = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)

    h, w = binary_full.shape
    debug = frame.copy()

    # Far-field obstacle probe (upper ROI).
    obstacle_width_cm: float | None = None
    obstacle_x_norm: float | None = None
    obstacle_frame_width_cm: float | None = None

    far_h = int(np.clip(h * obstacle_far_roi_ratio, 0, h))
    obstacle_mask = np.zeros_like(binary_full)
    obstacle_mask[int(0.3 * h):far_h, :] = 255
    obstacle_binary = cv2.bitwise_and(binary_full, binary_full, mask=obstacle_mask)

    obstacle_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    obstacle_binary = cv2.morphologyEx(obstacle_binary, cv2.MORPH_OPEN, obstacle_kernel)

    obs_contours, _ = cv2.findContours(obstacle_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if obs_contours:
        obs = max(obs_contours, key=cv2.contourArea)
        obs_area = float(cv2.contourArea(obs))
        if obs_area >= obstacle_min_area_px:
            ox, oy, ow, oh = cv2.boundingRect(obs)
            dynamic_px_per_cm: float | None = None
            if (
                obstacle_distance_cm is not None
                and obstacle_distance_cm > 0.0
                and camera_diag_fov_deg > 0.0
            ):
                diag_px = float(np.hypot(w, h))
                half_diag_fov_rad = float(np.deg2rad(camera_diag_fov_deg * 0.5))
                tan_half = float(np.tan(half_diag_fov_rad))
                if tan_half > 1e-6:
                    focal_px = (0.5 * diag_px) / tan_half
                    dynamic_px_per_cm = focal_px / float(obstacle_distance_cm)

            px_per_cm = float(obstacle_px_per_cm)
            if dynamic_px_per_cm is not None and dynamic_px_per_cm > 1e-6:
                px_per_cm = dynamic_px_per_cm
            if px_per_cm > 1e-6:
                obstacle_frame_width_cm = float(w) / px_per_cm

            if px_per_cm > 1e-6:
                obstacle_width_cm = float(ow) / px_per_cm
            obstacle_x_norm = float(np.clip(((ox + (ow * 0.5)) - (w * 0.5)) / (w * 0.5), -1.0, 1.0))
            _ = cv2.rectangle(debug, (ox, oy), (ox + ow, oy + oh), (0, 0, 255), 2)
            _ = cv2.putText(
                debug,
                f"obs_w={obstacle_width_cm:.1f}cm x={obstacle_x_norm:.2f}" if obstacle_width_cm is not None else f"obs_x={obstacle_x_norm:.2f}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

    # Bottom ROI for lane tracking.
    lane_mask = np.zeros_like(binary_full)
    lane_mask[int(h * 0.45):h, 0:w] = 255
    binary = cv2.bitwise_and(binary_full, binary_full, mask=lane_mask)

    # Remove noise
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if len(contours) == 0:
        return None, None, None, 0.0, obstacle_width_cm, obstacle_x_norm, obstacle_frame_width_cm, debug

    # Use the largest contour as the line
    contour = max(contours, key=cv2.contourArea)

    points = contour[:,0,:]

    if len(points) < 3:
        return None, None, None, 0.0, obstacle_width_cm, obstacle_x_norm, obstacle_frame_width_cm, debug

    x = points[:,0]
    y = points[:,1]

    # Fit polynomial (x as function of y)
    coeffs = np.polyfit(y, x, 2)
    a, b, c = coeffs

    # Heading angle (slope at bottom of image)
    lookahead_ratio = 0.85 
    y_eval = frame.shape[0] * lookahead_ratio
    slope = 2*a*y_eval + b
    angle = np.arctan(slope)

    # Curvature calculation
    curvature = abs(2*a) / ((1 + slope**2)**1.5)

    # Offset from image center
    x_line = a*y_eval**2 + b*y_eval + c
    center = frame.shape[1] / 2
    offset = float(np.clip((x_line - center) / center, -1.0, 1.0))

    # Confidence estimate from contour quality + fit quality
    h, w = frame.shape[:2]
    frame_area = float(h * w)
    contour_area = float(cv2.contourArea(contour))
    area_score = float(np.clip(contour_area / (0.12 * frame_area), 0.0, 1.0))

    y_span = float(y.max() - y.min())
    vertical_coverage = float(np.clip(y_span / max(1.0, float(h)), 0.0, 1.0))

    x_fit = a * y**2 + b * y + c
    rmse = float(np.sqrt(np.mean((x - x_fit) ** 2)))
    fit_score = float(np.clip(1.0 - (rmse / max(1.0, 0.2 * w)), 0.0, 1.0))

    point_score = float(np.clip(len(points) / 250.0, 0.0, 1.0))
    confidence = float(np.clip(
        0.35 * area_score +
        0.30 * vertical_coverage +
        0.25 * fit_score +
        0.10 * point_score,
        0.0,
        1.0,
    ))

    for yi in range(0, frame.shape[0], 5):
        xi = int(a * yi**2 + b * yi + c)
        if 0 <= xi < frame.shape[1]:
            _ = cv2.circle(debug, (xi, yi), 2, (0, 255, 0), -1)

    _ = cv2.drawContours(debug, [contour], -1, (255,0,0), 2)
    _ = cv2.putText(
        debug,
        f"slope: {float(slope):.3f}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    return angle, curvature, offset, confidence, obstacle_width_cm, obstacle_x_norm, obstacle_frame_width_cm, debug


def detect_line_geometry_canny(jpeg_bytes: bytes):
    """
    Detect line direction and curvature from a front-facing camera frame
    using Canny + Hough transform.

    Returns:
        angle (rad)
        curvature
        offset (-1 to 1)
        confidence (0 to 1)
        debug_image
    """

    import numpy as np
    import cv2

    # Decode image
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)

    if frame is None:
        raise ValueError("Failed to decode JPEG bytes into an image")

    h, w = frame.shape[:2]

    # --- 1. Preprocessing (Canny) ---
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # --- 2. Region of Interest (bottom part of image) ---
    mask = np.zeros_like(edges)

    polygon = np.array([[
        (0, h),
        (w, h),
        (w, int(h * 0.45)),
        (0, int(h * 0.45))
    ]], dtype=np.int32)

    cv2.fillPoly(mask, polygon, 255)
    edges = cv2.bitwise_and(edges, mask)

    # --- 3. Hough Transform ---
    lines = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=50,
        minLineLength=50,
        maxLineGap=20
    )

    # --- 4. Convert lines → points ---
    points = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Skip near-horizontal lines (noise)
            if abs(y2 - y1) < 10:
                continue

            num_samples = 20
            for t in np.linspace(0, 1, num_samples):
                x = int(x1 * (1 - t) + x2 * t)
                y = int(y1 * (1 - t) + y2 * t)
                points.append((x, y))

    if len(points) < 3:
        return None, None, None, 0.0, frame

    points = np.array(points)
    x = points[:, 0]
    y = points[:, 1]

    # --- 5. Polynomial fit (same as your original) ---
    coeffs = np.polyfit(y, x, 2)
    a, b, c = coeffs

    # Heading angle (slope at bottom)
    y_eval = h
    slope = 2 * a * y_eval + b
    angle = float(np.arctan(slope))

    # Curvature
    curvature = float(abs(2 * a) / ((1 + slope**2) ** 1.5))

    # Offset from center
    x_line = a * y_eval**2 + b * y_eval + c
    center = w / 2
    offset = float(np.clip((x_line - center) / center, -1.0, 1.0))

    # --- 6. Confidence (adapted) ---
    num_lines = 0 if lines is None else len(lines)
    line_score = float(np.clip(num_lines / 10.0, 0.0, 1.0))

    y_span = float(y.max() - y.min())
    vertical_coverage = float(np.clip(y_span / h, 0.0, 1.0))

    x_fit = a * y**2 + b * y + c
    rmse = float(np.sqrt(np.mean((x - x_fit) ** 2)))
    fit_score = float(np.clip(1.0 - (rmse / max(1.0, 0.2 * w)), 0.0, 1.0))

    point_score = float(np.clip(len(points) / 300.0, 0.0, 1.0))

    confidence = float(np.clip(
        0.4 * line_score +
        0.3 * vertical_coverage +
        0.2 * fit_score +
        0.1 * point_score,
        0.0,
        1.0
    ))

    # --- 7. Debug visualization ---
    debug = frame.copy()

    # Draw Hough lines
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(debug, (x1, y1), (x2, y2), (255, 0, 0), 2)

    # Draw fitted curve
    for yi in range(0, h, 5):
        xi = int(a * yi**2 + b * yi + c)
        if 0 <= xi < w:
            cv2.circle(debug, (xi, yi), 2, (0, 255, 0), -1)

    return angle, curvature, offset, confidence, debug