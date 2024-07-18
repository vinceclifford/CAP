import cv2
import numpy as np
import glob
import sys


def calibrate_camera(path):
    chess_board_size = (9, 6)
    frame_size = (1920, 1080)

    # Standard criteria from opencv 
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((chess_board_size[0] * chess_board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chess_board_size[0], 0:chess_board_size[1]].T.reshape(-1, 2)

    object_points = []
    image_points = []
    images_used_for_calibration = 0

    images = glob.glob(path + '/*.png')
    for image in images:

        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, chess_board_size, None)
        if ret == True:
            images_used_for_calibration += 1
            object_points.append(objp)
            corners_sub_pixel = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            image_points.append(corners)

            cv2.drawChessboardCorners(img, chess_board_size, corners_sub_pixel, ret)

            cv2.imshow('img', img)
            cv2.waitKey(2000)

    if images_used_for_calibration == 0:
        sys.exit("No calibration images were provided")
    ret, camera_matrix, distortion, rotation_vector, translation_vector = cv2.calibrateCamera(object_points,
                                                                                              image_points, frame_size,
                                                                                              None, None)

    print(f"Camera calibrated: {ret}")
    print(f"Camera Matrix: {camera_matrix}")
    print(f"Distortion Paramters: {distortion}")
    print(f"Rotation Vector: {rotation_vector}")
    print(f"Translation Vector: {translation_vector}")

    return camera_matrix, distortion, rotation_vector, translation_vector
