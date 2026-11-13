"""Utilities for calculating target centres in camera images."""


def calculate_center(x, y, width, height):
    """
    Calculate the centre of a bounding box.

    Parameters
    ----------
    x : float
        Left coordinate of the bounding box.
    y : float
        Top coordinate of the bounding box.
    width : float
        Width of the bounding box.
    height : float
        Height of the bounding box.

    Returns
    -------
    tuple
        Centre x and y coordinates.

    Raises
    ------
    ValueError
        If width or height is not positive.

    """
    if width <= 0 or height <= 0:
        raise ValueError("Width and height must be positive.")

    center_x = x + width / 2.0
    center_y = y + height / 2.0

    return center_x, center_y


def calculate_error(center_x, center_y, image_width, image_height):
    """
    Calculate target displacement from the image centre.

    Parameters
    ----------
    center_x : float
        Target centre x coordinate.
    center_y : float
        Target centre y coordinate.
    image_width : int
        Image width in pixels.
    image_height : int
        Image height in pixels.

    Returns
    -------
    tuple
        Horizontal and vertical pixel errors.

    Raises
    ------
    ValueError
        If image dimensions are not positive.

    """
    if image_width <= 0 or image_height <= 0:
        raise ValueError("Image dimensions must be positive.")

    image_center_x = image_width / 2.0
    image_center_y = image_height / 2.0

    error_x = center_x - image_center_x
    error_y = center_y - image_center_y

    return error_x, error_y
