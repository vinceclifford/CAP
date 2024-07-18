import cv2
import numpy as np

ARUCO_DICT = {
    "DICT_4x4_50": cv2.aruco.DICT_4X4_50
}


def coordinate_estimation_continous(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    # id number 4 and 5 represent the goal and robot respectively

    distance_map = {}
    gray_image = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, _ = detector.detectMarkers(gray_image)
    robot = None
    obstacles = {}

    if ids is not None:
        # Draw the detected markers on the frame
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            marker_id = ids[i][0]
            center = np.mean(corners[i][0], axis=0)
            x, y = center[0], center[1]
            distance_map[marker_id] = np.array((x.item(), y.item()))

            # Draw a circle at the center of the marker
            cv2.circle(frame, (int(x), int(y)), radius=5, color=(0, 0, 255), thickness=-1)

            marker_points = np.array([[-0.01, 0.01, 0], [0.01, 0.01, 0], [0.01, -0.01, 0], [-0.01, -0.01, 0]],
                                     dtype=np.float32)
            # Estimate the pose of the marker
            success, rvec, tvec = cv2.solvePnP(marker_points, corners[i][0], matrix_coefficients,
                                               distortion_coefficients)

        # Check if all required markers are detected
        required_ids = [0, 1, 2, 3]
        if all(id in distance_map for id in required_ids):

            camera_coordinates = np.array([distance_map[id] for id in required_ids], dtype='float32')
            real_world_coords = np.array([[0, 0], [1155, 0], [1155, 1415], [0, 1415]], dtype='float32')

            # Compute the homography matrix
            H, status = cv2.findHomography(camera_coordinates, real_world_coords)

            for key, value in distance_map.items():
                # We exclude the 4th obstacles because we assume that the goal is static
                if key in {0, 1, 2, 3, 4}:
                    continue

                point_camera = np.array([distance_map[key]], dtype='float32').reshape(-1, 1, 2)
                point_or = cv2.perspectiveTransform(point_camera, H)
                if key == 5:
                    robot = int(point_or[0][0][0] / 2), int(point_or[0][0][1] / 2)
                else:
                    obstacles[key] = (int(point_or[0][0][0] / 2), int(point_or[0][0][1] / 2))

    return obstacles, robot, frame


def coordinate_estimation_first(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    # id number 4 and 5 represent the goal and robot respectively
    distance_map = {}
    gray_image = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)

    corners, ids, _ = detector.detectMarkers(gray_image)
    target = None
    robot = None
    obstacles = {}

    if ids is not None:
        # Draw the detected markers on the frame
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            marker_id = ids[i][0]
            center = np.mean(corners[i][0], axis=0)
            x, y = center[0], center[1]
            distance_map[marker_id] = np.array((x.item(), y.item()))

            # Draw a circle at the center of the marker
            cv2.circle(frame, (int(x), int(y)), radius=5, color=(0, 0, 255), thickness=-1)

            marker_points = np.array([[-0.01, 0.01, 0], [0.01, 0.01, 0], [0.01, -0.01, 0], [-0.01, -0.01, 0]],
                                     dtype=np.float32)
            # Estimate the pose of the marker
            success, rvec, tvec = cv2.solvePnP(marker_points, corners[i][0], matrix_coefficients,
                                               distortion_coefficients)

        # Check if all required markers are detected
        required_ids = [0, 1, 2, 3]
        if all(id in distance_map for id in required_ids):

            camera_coordinates = np.array([distance_map[id] for id in required_ids], dtype='float32')
            real_world_coords = np.array([[0, 0], [1155, 0], [1155, 1415], [0, 1415]], dtype='float32')

            # Compute the homography matrix
            H, status = cv2.findHomography(camera_coordinates, real_world_coords)

            for key, value in distance_map.items():
                if key in {0, 1, 2, 3}:
                    continue

                point_camera = np.array([distance_map[key]], dtype='float32').reshape(-1, 1, 2)
                point_or = cv2.perspectiveTransform(point_camera, H)
                if key == 4:
                    target = (int(point_or[0][0][0] / 2), int(point_or[0][0][1] / 2))
                elif key == 5:
                    robot = (int(point_or[0][0][0] / 2), int(point_or[0][0][1] / 2))
                else:
                    obstacles[key] = (int(point_or[0][0][0] / 2), int(point_or[0][0][1] / 2))

    return obstacles, robot, target, frame
