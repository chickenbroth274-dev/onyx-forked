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


def detect_concentric_target(
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
