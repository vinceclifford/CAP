import cv2
import numpy as np
import glob 

def calibrate_camera(path):

    chess_board_size = (9, 6)
    frame_size = (1920, 1080)

    # Standard criteria from opencv 
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((chess_board_size[0] * chess_board_size[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:chess_board_size[0], 0:chess_board_size[1]].T.reshape(-1,2)

    object_points = []
    image_points = []

    images = glob.glob(path + '/calibration_images/*.png')
    counter = 0 
    for image in images: 
       
        img = cv2.imread(image)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, chess_board_size, None)
        if ret == True: 
            object_points.append(objp)
            corners_sub_pixel = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            image_points.append(corners)

            cv2.drawChessboardCorners(img, chess_board_size, corners_sub_pixel, ret)
            cv2.imwrite(path + f'/calibration_images/chessboard{counter}.png', img)
            counter += 1
            
            cv2.imshow('img', img)
            cv2.waitKey(1000)

    ret, camera_matrix, distortion, rotation_vector, translation_vector = cv2.calibrateCamera(object_points, image_points, frame_size, None, None)

    print(f"Camera calibrated: {ret}")
    print(f"Camera Matrix: {camera_matrix}")
    print(f"Distortion Paramters: {distortion}")
    print(f"Rotation Vector: {rotation_vector}")
    print(f"Translation Vector: {translation_vector}")
    
    img = cv2.imread(path + '/calibration_images/calibration_image_2.png')
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion, (w, h), 1, (w, h))

    dst = cv2.undistort(img, camera_matrix, distortion, None, new_camera_matrix)

    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, distortion, None, new_camera_matrix, (w,h), 5)
    dst = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

    x,y,w,h = roi 
    dst = dst[y:y+h, x:x+w]
    cv2.imwrite(path + '/calibration_images/undistorted_2.png', dst)

calibrate_camera("/home/mapsmain/bachelor_thesis_vincent/CAP")