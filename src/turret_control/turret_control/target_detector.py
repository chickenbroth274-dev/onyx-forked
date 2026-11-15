"""Detect concentric circular targets using OpenCV."""

import cv2
import numpy as np


def _remove_duplicate_circles(circles):
    """Remove duplicate circle candidates."""

    circles = sorted(
        circles,
        key=lambda c: (c[3], c[2])
    )

    unique = []

    for circle in circles:

        cx, cy, radius, axis_ratio = circle

        duplicate_index = None

        for index, existing in enumerate(unique):

            ex, ey, er, existing_ratio = existing

            center_distance = np.hypot(
                cx - ex,
                cy - ey
            )

            radius_difference = abs(radius - er)

            center_threshold = max(
                3.0,
                0.06 * min(radius, er)
            )

            radius_threshold = max(
                3.0,
                0.07 * min(radius, er)
            )

            if (
                center_distance < center_threshold
                and radius_difference < radius_threshold
            ):
                duplicate_index = index
                break

        if duplicate_index is None:

            unique.append(circle)

        else:

            # Keep the more circular candidate.
            if axis_ratio < unique[duplicate_index][3]:
                unique[duplicate_index] = circle

    return unique


def _detect_strict_target(
    image,
    min_radius=5,
    center_tolerance=0.12,
    ratio_tolerance=0.25,
):
    """
    Detect three approximately concentric circular boundaries.

    Parameters
    ----------
    image : numpy.ndarray
        Input BGR or grayscale image.

    min_radius : float
        Minimum accepted circle radius in pixels.

    center_tolerance : float
        Maximum centre deviation relative to the largest radius.

    ratio_tolerance : float
        Allowed relative error in the expected 1:2:3 radius ratio.

    Returns
    -------
    dict or None
        Dictionary containing center, radii and score.
    """

    # --------------------------------------------------
    # 1. Validate image
    # --------------------------------------------------

    if image is None or image.size == 0:
        raise ValueError("Input image is empty.")

    if image.ndim == 2:

        gray = image

    elif image.ndim == 3 and image.shape[2] == 3:

        gray = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2GRAY
        )

    else:

        raise ValueError(
            "Input must be grayscale or a 3-channel BGR image."
        )

    if min_radius <= 0:
        raise ValueError("min_radius must be positive.")

    if center_tolerance < 0 or ratio_tolerance < 0:
        raise ValueError("Tolerances must be non-negative.")

    # --------------------------------------------------
    # 2. Preprocessing
    # --------------------------------------------------

    blurred = cv2.GaussianBlur(
        gray,
        (5, 5),
        1.5
    )

    edges = cv2.Canny(
        blurred,
        50,
        150
    )

    contours, _ = cv2.findContours(
        edges,
        cv2.RETR_LIST,
        cv2.CHAIN_APPROX_SIMPLE
    )

    # --------------------------------------------------
    # 3. Extract circular candidates
    # --------------------------------------------------

    circles = []

    minimum_area = np.pi * min_radius ** 2

    for contour in contours:

        if len(contour) < 5:
            continue

        area = cv2.contourArea(contour)

        if area < minimum_area:
            continue

        ellipse = cv2.fitEllipse(contour)

        (cx, cy), (axis_a, axis_b), _ = ellipse

        if axis_a <= 0 or axis_b <= 0:
            continue

        radius = (axis_a + axis_b) / 4.0

        if radius < min_radius:
            continue

        axis_ratio = max(axis_a, axis_b) / min(axis_a, axis_b)

        # Reject highly elliptical contours.
        if axis_ratio > 1.5:
            continue

        circles.append(
            (
                float(cx),
                float(cy),
                float(radius),
                float(axis_ratio)
            )
        )

    if len(circles) < 3:
        return None

    # --------------------------------------------------
    # 4. Remove duplicate candidates
    # --------------------------------------------------

    circles = _remove_duplicate_circles(circles)

    if len(circles) < 3:
        return None

    # Sort by radius.
    circles.sort(key=lambda c: c[2])

    radii = np.array(
        [c[2] for c in circles],
        dtype=float
    )

    centers = np.array(
        [[c[0], c[1]] for c in circles],
        dtype=float
    )

    axis_ratios = np.array(
        [c[3] for c in circles],
        dtype=float
    )

    candidate_count = len(circles)

    best_candidate = None
    best_score = float("inf")

    # --------------------------------------------------
    # 5. Search using each candidate as the outer circle
    # --------------------------------------------------

    for outer_index in range(candidate_count):

        outer_radius = radii[outer_index]

        outer_center = centers[outer_index]

        # Expected inner and middle radii.
        expected_small = outer_radius / 3.0
        expected_middle = outer_radius * 2.0 / 3.0

        # Radius tolerance ranges.
        small_min = expected_small * (1.0 - ratio_tolerance)
        small_max = expected_small * (1.0 + ratio_tolerance)

        middle_min = expected_middle * (1.0 - ratio_tolerance)
        middle_max = expected_middle * (1.0 + ratio_tolerance)

        # Candidate small circles.
        small_indices = np.flatnonzero(
            (radii >= small_min)
            & (radii <= small_max)
            & (np.arange(candidate_count) < outer_index)
        )

        # Candidate middle circles.
        middle_indices = np.flatnonzero(
            (radii >= middle_min)
            & (radii <= middle_max)
            & (np.arange(candidate_count) < outer_index)
        )

        if len(small_indices) == 0:
            continue

        if len(middle_indices) == 0:
            continue

        # --------------------------------------------------
        # 6. Filter by centre distance
        # --------------------------------------------------

        small_distances = np.linalg.norm(
            centers[small_indices] - outer_center,
            axis=1
        )

        middle_distances = np.linalg.norm(
            centers[middle_indices] - outer_center,
            axis=1
        )

        max_center_distance = center_tolerance * outer_radius

        small_indices = small_indices[
            small_distances <= max_center_distance
        ]

        middle_indices = middle_indices[
            middle_distances <= max_center_distance
        ]

        if len(small_indices) == 0:
            continue

        if len(middle_indices) == 0:
            continue

        # --------------------------------------------------
        # 7. Evaluate only promising triples
        # --------------------------------------------------

        for small_index in small_indices:

            for middle_index in middle_indices:

                selected_radii = np.array(
                    [
                        radii[small_index],
                        radii[middle_index],
                        outer_radius
                    ]
                )

                selected_centers = np.array(
                    [
                        centers[small_index],
                        centers[middle_index],
                        outer_center
                    ]
                )

                # Calculate common centre.
                common_center = np.mean(
                    selected_centers,
                    axis=0
                )

                # Calculate centre deviations.
                center_errors = np.linalg.norm(
                    selected_centers - common_center,
                    axis=1
                )

                if np.max(center_errors) > max_center_distance:
                    continue

                # Check expected radius ratios.
                measured_ratios = (
                    selected_radii / selected_radii[0]
                )

                expected_ratios = np.array(
                    [1.0, 2.0, 3.0]
                )

                ratio_errors = (
                    np.abs(
                        measured_ratios - expected_ratios
                    ) / expected_ratios
                )

                if np.max(ratio_errors) > ratio_tolerance:
                    continue

                # Prefer circular contours.
                selected_axis_ratios = np.array(
                    [
                        axis_ratios[small_index],
                        axis_ratios[middle_index],
                        axis_ratios[outer_index]
                    ]
                )

                shape_penalty = np.mean(
                    selected_axis_ratios - 1.0
                )

                # Final candidate score.
                score = (
                    np.mean(ratio_errors)
                    + np.mean(center_errors) / outer_radius
                    + 0.1 * shape_penalty
                )

                if score < best_score:

                    best_score = score

                    best_candidate = {
                        "center": (
                            float(common_center[0]),
                            float(common_center[1])
                        ),

                        "radii": tuple(
                            float(r)
                            for r in selected_radii
                        ),

                        "score": float(score)
                    }

    return best_candidate


# ---------------------------------------------------------------------------
# Perspective-tolerant fallback for phone-screen and oblique bullseye images.
# The strict 1:2:3 detector above remains the first choice for the official
# three-disc target. This fallback is only used when the strict detector fails.
# ---------------------------------------------------------------------------

def _fallback_ellipse_candidates(gray, min_radius=4.0, max_axis_ratio=2.6):
    """Extract nested ellipse candidates while allowing oblique viewpoints."""
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.5)
    edges = cv2.Canny(blurred, 50, 150)
    contours, _ = cv2.findContours(
        edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []
    minimum_area = max(35.0, np.pi * (0.45 * min_radius) ** 2)

    for contour in contours:
        if len(contour) < 5 or cv2.contourArea(contour) < minimum_area:
            continue

        (cx, cy), (axis_a, axis_b), angle = cv2.fitEllipse(contour)
        if axis_a <= 0.0 or axis_b <= 0.0:
            continue

        major = max(axis_a, axis_b)
        minor = min(axis_a, axis_b)
        axis_ratio = major / minor
        if axis_ratio > max_axis_ratio:
            continue

        radius = 0.5 * np.sqrt(axis_a * axis_b)
        if radius < min_radius:
            continue

        points = contour.reshape(-1, 2).astype(np.float32)
        ellipse_angle = np.deg2rad(angle)
        cos_a, sin_a = np.cos(ellipse_angle), np.sin(ellipse_angle)
        dx, dy = points[:, 0] - cx, points[:, 1] - cy
        u = cos_a * dx + sin_a * dy
        v = -sin_a * dx + cos_a * dy
        a, b = max(axis_a / 2.0, 1.0), max(axis_b / 2.0, 1.0)
        normalized_radius = np.sqrt((u / a) ** 2 + (v / b) ** 2)
        residual = float(np.median(np.abs(normalized_radius - 1.0)))
        quality = float(1.0 / (1.0 + residual + abs(axis_ratio - 1.0) * 0.15))

        candidates.append({
            "center": (float(cx), float(cy)),
            "radius": float(radius),
            "axis_ratio": float(axis_ratio),
            "angle": float(angle),
            "quality": quality,
            "area": float(cv2.contourArea(contour)),
        })

    candidates.sort(key=lambda item: item["radius"])
    unique = []
    for candidate in candidates:
        duplicate_index = None
        for index, existing in enumerate(unique):
            distance = np.hypot(
                candidate["center"][0] - existing["center"][0],
                candidate["center"][1] - existing["center"][1],
            )
            radius_delta = abs(candidate["radius"] - existing["radius"])
            threshold = max(4.0, 0.055 * min(candidate["radius"], existing["radius"]))
            if distance <= threshold and radius_delta <= threshold:
                duplicate_index = index
                break
        if duplicate_index is None:
            unique.append(candidate)
        elif candidate["quality"] > unique[duplicate_index]["quality"]:
            unique[duplicate_index] = candidate

    return unique


def _find_color_center(mask, predicted_center, search_radius, max_component_area):
    """Find a compact saturated red/yellow/green region near the ellipse centre."""
    cx, cy = predicted_center
    height, width = mask.shape
    x0 = max(0, int(cx - search_radius))
    x1 = min(width, int(cx + search_radius + 1))
    y0 = max(0, int(cy - search_radius))
    y1 = min(height, int(cy + search_radius + 1))
    if x1 <= x0 or y1 <= y0:
        return None

    roi = mask[y0:y1, x0:x1].copy()
    yy, xx = np.ogrid[y0:y1, x0:x1]
    roi[((xx - cx) ** 2 + (yy - cy) ** 2) > search_radius ** 2] = 0
    roi = cv2.morphologyEx(
        roi, cv2.MORPH_OPEN, np.ones((3, 3), dtype=np.uint8)
    )

    count, labels, stats, centroids = cv2.connectedComponentsWithStats(
        roi, connectivity=8
    )
    options = []
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < 20 or area > max_component_area:
            continue
        bx, by, bw, bh, _ = stats[label]
        if bw <= 0 or bh <= 0:
            continue
        if max(bw, bh) / min(bw, bh) > 2.8:
            continue
        px, py = centroids[label]
        px += x0
        py += y0
        distance = float(np.hypot(px - cx, py - cy))
        # Strongly prefer a compact patch close to the predicted bullseye centre.
        score = distance / max(search_radius, 1.0) + 0.015 * abs(np.log(max(area, 1)))
        options.append((score, float(px), float(py), area))

    if not options:
        return None
    _, px, py, area = min(options, key=lambda item: item[0])
    return px, py, float(np.sqrt(area / np.pi))


def _detect_perspective_bullseye(image, min_radius=4.0):
    """Detect nested ellipses without assuming a 1:2:3 size ratio."""
    if image.ndim == 2:
        gray = image
        bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        bgr = image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    candidates = _fallback_ellipse_candidates(gray, min_radius=min_radius)
    if len(candidates) < 2:
        return None

    # Precompute a colour mask once; avoid repeated HSV conversions per pair.
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    valid_hue = (hue <= 90) | (hue >= 170)
    color_mask = (valid_hue & (saturation >= 75) & (value >= 55)).astype(np.uint8) * 255

    # The candidate search is bounded for consistent processing time.
    if len(candidates) > 45:
        candidates = sorted(
            candidates,
            key=lambda item: item["quality"] * (1.0 + np.log1p(item["area"])),
            reverse=True,
        )[:45]
        candidates.sort(key=lambda item: item["radius"])

    best = None
    best_score = float("inf")
    count = len(candidates)

    for outer_index in range(1, count):
        outer = candidates[outer_index]
        for inner_index in range(outer_index):
            inner = candidates[inner_index]
            separation = outer["radius"] / inner["radius"]
            if separation < 1.12 or separation > 3.5:
                continue

            base_center = (
                (inner["center"][0] + outer["center"][0]) / 2.0,
                (inner["center"][1] + outer["center"][1]) / 2.0,
            )
            center_distance = float(np.hypot(
                inner["center"][0] - outer["center"][0],
                inner["center"][1] - outer["center"][1],
            ))
            if center_distance > 0.22 * outer["radius"]:
                continue
            if abs(inner["axis_ratio"] - outer["axis_ratio"]) > 0.65:
                continue
            angle_delta = abs((inner["angle"] - outer["angle"] + 90.0) % 180.0 - 90.0)
            if angle_delta > 28.0:
                continue

            pair_score = (
                center_distance / max(outer["radius"], 1.0)
                + 0.35 * abs(inner["axis_ratio"] - outer["axis_ratio"])
                + angle_delta / 180.0
                + 0.25 * (2.0 - inner["quality"] - outer["quality"])
            )

            # Prefer a third matching ellipse when present.
            selected = None
            for middle_index in range(inner_index + 1, outer_index):
                middle = candidates[middle_index]
                if middle["radius"] / inner["radius"] < 1.12:
                    continue
                if outer["radius"] / middle["radius"] < 1.12:
                    continue
                group = [inner, middle, outer]
                group_centers = np.asarray([g["center"] for g in group], dtype=float)
                common = np.mean(group_centers, axis=0)
                errors = np.linalg.norm(group_centers - common, axis=1)
                ratios = [g["axis_ratio"] for g in group]
                angles = [g["angle"] for g in group]
                angle_spread = max(
                    abs((a - angles[0] + 90.0) % 180.0 - 90.0) for a in angles
                )
                if np.max(errors) > 0.22 * outer["radius"]:
                    continue
                if max(ratios) - min(ratios) > 0.65 or angle_spread > 28.0:
                    continue
                score = (
                    float(np.mean(errors) / max(outer["radius"], 1.0))
                    + 0.25 * (max(ratios) - min(ratios))
                    + angle_spread / 180.0
                    + 0.25 * (3.0 - sum(g["quality"] for g in group))
                )
                if selected is None or score < selected[0]:
                    selected = (score, group, common)

            if selected is not None:
                score, group, common = selected
                result = {
                    "center": (float(common[0]), float(common[1])),
                    "radii": tuple(float(g["radius"]) for g in group),
                    "score": float(score),
                }
            else:
                color_center = _find_color_center(
                    color_mask,
                    base_center,
                    search_radius=0.22 * outer["radius"],
                    max_component_area=max(50.0, 0.30 * np.pi * inner["radius"] ** 2),
                )
                if color_center is None:
                    continue
                px, py, color_radius = color_center
                result = {
                    "center": (float(px), float(py)),
                    "radii": (float(color_radius), float(inner["radius"]), float(outer["radius"])),
                    "score": float(pair_score + 0.10),
                }

            if result["score"] < best_score:
                best_score = result["score"]
                best = result

    return best


def detect_concentric_target(
    image,
    min_radius=5,
    center_tolerance=0.12,
    ratio_tolerance=0.25,
):
    """Detect a concentric target; fall back to perspective-tolerant ellipses.

    The original strict 1:2:3 circle detector is retained as the primary path.
    If it fails, a half-resolution fallback accepts nested ellipses and can use
    a compact coloured centre patch when only two ring edges remain visible.
    Output coordinates and radii are converted back to the input image scale.
    """
    strict_result = _detect_strict_target(
        image,
        min_radius=min_radius,
        center_tolerance=center_tolerance,
        ratio_tolerance=ratio_tolerance,
    )
    if strict_result is not None:
        return strict_result

    if image is None or image.size == 0:
        raise ValueError("Input image is empty.")

    original_height, original_width = image.shape[:2]
    scale = min(1.0, 640.0 / float(original_width))
    if scale < 1.0:
        new_size = (
            max(1, int(round(original_width * scale))),
            max(1, int(round(original_height * scale))),
        )
        reduced = cv2.resize(image, new_size, interpolation=cv2.INTER_AREA)
    else:
        reduced = image

    fallback_result = _detect_perspective_bullseye(
        reduced,
        min_radius=max(3.5, min_radius * scale),
    )
    if fallback_result is None:
        return None

    if scale < 1.0:
        fallback_result["center"] = (
            float(fallback_result["center"][0] / scale),
            float(fallback_result["center"][1] / scale),
        )
        fallback_result["radii"] = tuple(
            float(radius / scale) for radius in fallback_result["radii"]
        )
    return fallback_result
