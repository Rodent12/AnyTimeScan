import cv2
import numpy as np


def calibrate_camera(images):
    # Assuming all images have the same size
    image_size = images[0].shape[:2]

    # Define chessboard pattern parameters (modify based on your calibration pattern)
    pattern_size = (8, 5)  # Number of inner corners in your calibration pattern
    obj_points = []  # 3D points in the real world
    img_points = []  # 2D points in the image plane

    # Generate calibration pattern coordinates
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    i= 1
    for img in images:
        i+=1
        print(i)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            obj_points.append(objp)
            img_points.append(corners)

    # Perform camera calibration
    _, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)

    return camera_matrix, dist_coeffs