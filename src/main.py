import pygame
import time
import shutil
import cv2
import numpy as np
from src.dijkstra import dijkstra_on_nparray_with_dictionary_without_detach
from visualization_engine import draw_robot, draw_obstacles, visualizing_dijkstra, check_validity_of_obstacles, \
    GREY, BLACK, RED, PURPLE, visualization_heat_map_tensor
from environments.environment_1 import obstacles, agent, target, SCREEN_WIDTH, SCREEN_HEIGHT
from src.math_tensor import calculate_total_repulsive_field_value, calculate_attraction_field_value_tensor
from coordinate_estimation import coordinate_estimation_continous, coordinate_estimation_first, ARUCO_DICT
from src.camera_calibration import calibrate_camera
from src.get_calibration_images import get_images
from src.staticcircle import StaticCircle
from src.robot import Robot


def main_pathplanning():
    check_validity_of_obstacles(obstacles, SCREEN_WIDTH, SCREEN_HEIGHT)
    pygame.init()
    pygame.display.set_caption("Environment")
    screen = pygame.display.set_mode((SCREEN_WIDTH + 5, SCREEN_HEIGHT + 5))
    screen.fill(GREY)

    if SCREEN_WIDTH <= 0:
        exit("A non positive Screen width was entered!")

    draw_robot(screen, agent, BLACK, 5)
    draw_robot(screen, target, RED, 10)
    print(obstacles)
    draw_obstacles(screen, obstacles, PURPLE)
    pygame.display.flip()

    s_time = time.time()
    tensor = calculate_total_repulsive_field_value(obstacles, target, SCREEN_WIDTH, SCREEN_HEIGHT)
    path, _ = dijkstra_on_nparray_with_dictionary_without_detach(tensor, agent.vector, target.vector)
    f_time = time.time()
    visualizing_dijkstra(screen, path)
    diff = f_time - s_time
    print(f"The time it took is {diff} seconds")
    visualization_heat_map_tensor(tensor)

    while True:
        done = False
        while not done:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True


def update_cache(obstacle_classes_map, other_map, frame_number, velocity_history_length=0):
    # We update all the obstacles that we have gotten in the last frame
    for key, coordinates in other_map.items():
        # Case that we have already seen the obstacles
        if key in obstacle_classes_map:
            circle_object, seen_cache, velocity_cache = obstacle_classes_map[key]
            # We have to throw out the latest element in the velocity history if we have too much data
            if len(velocity_cache) >= velocity_history_length:
                velocity_cache.pop(0)
            velocity_cache.append((coordinates[0], coordinates[1]))
            circle_object.vector = (coordinates[0], coordinates[1])
            obstacle_classes_map[key] = (circle_object, 1, velocity_cache)
        # Case that we have not seen the object so far
        else:
            obstacle_classes_map[key] = (
            StaticCircle(coordinates[0], coordinates[1], 30, 3, 25), 1, [(coordinates[0], coordinates[1])])

    difference_set = obstacle_classes_map.keys() - other_map.keys()

    # We update all the frames that we have not seen in the current pulled frame
    for key in difference_set:
        circle, last_seen, velocity_cache = obstacle_classes_map[key]
        if last_seen < frame_number:
            obstacle_classes_map[key] = circle, last_seen << 1, velocity_cache
        else:
            del obstacle_classes_map[key]

    obstacle_classes_list = []
    for key, value in obstacle_classes_map.items():
        circle_object, lifetime, velocity_cache = value
        obstacle_classes_list.append(circle_object)

    return obstacle_classes_list


def first_camera_frame(width, height, frame_number, velocity_history=0):
    pygame.init()
    pygame.display.set_caption("Environment")
    screen = pygame.display.set_mode((height + 5, width + 5))

    path_to_calibration_images = get_images()
    camera_matrix, distortion, _, _ = calibrate_camera(path_to_calibration_images)
    shutil.rmtree(path_to_calibration_images)

    aruco_type = "DICT_4x4_50"

    cam = cv2.VideoCapture(0)
    target = None
    robot = None

    # obstacles maps id to coordinates in the current frame. obstacle_classes_map maps ids to StaticCircle as well as
    # last time seen. obstacles: int -> (int,int) obstacle_classes_map: int -> (StaticCircle, int, list). The list
    # contain the last coordinates of the obstacle such that we can determine the average velocity
    obstacles = {}
    obstacle_classes_map = {}
    while target is None and robot is None:
        ret, image_ocv = cam.read()
        image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)
        obstacles, r, t, output = \
            coordinate_estimation_first(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion, scaling_factor)
        if target is None:
            target = t
        if robot is None:
            robot = r
        print(f"Fetching locations: Robot: {robot}, target {target}")

    obstacle_classes_list = update_cache(obstacle_classes_map, obstacles, frame_number, velocity_history)

    target_object = StaticCircle(target[0], target[1], 30, 3)
    print(target)
    robot_object = Robot(robot[0], robot[1])
    attraction_tensor = calculate_attraction_field_value_tensor(target_object, height, width)
    print("Came back from attraction tensor calculation")

    tensor = (
        calculate_total_repulsive_field_value(obstacle_classes_list, target_object, height, width,
                                              attraction_repulsive_tensor=attraction_tensor))

    path_orig, _ = dijkstra_on_nparray_with_dictionary_without_detach(tensor, robot_object.vector, target_object.vector)
    print(f"The robot has got position {robot_object.vector}")
    print(f"The target has got position {target_object.vector}")
    print(f"The size of the tensor is {tensor.shape}")

    pygame.draw.lines(screen, BLACK, False, path_orig, 2)
    pygame.display.flip()
    return (cam, aruco_type, camera_matrix, distortion, obstacle_classes_map, target_object, attraction_tensor, screen,
            path_orig, robot_object)


def main_realtime_detection_and_continuous_planning(width, height, frame_history, scaling_factor):
    if frame_history > 32:
        exit("An invalid frame history value was given")
    frame_number = 2 ** frame_history

    (cam, aruco_type, camera_matrix, distortion, obstacle_classes_map, target_object, attraction_tensor, screen, path,
     robot_object) = (first_camera_frame(width, height, frame_number))

    pressed_key = ' '
    done = False
    first = time.time()
    while pressed_key != ord('q') and not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        ret, image_ocv = cam.read()
        image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)

        obstacles_iteration, robot, output = (
            coordinate_estimation_continous(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion,
                                            scaling_factor))

        obstacle_classes_list = update_cache(obstacle_classes_map, obstacles_iteration, frame_number)

        if robot is not None:
            robot_object = Robot(robot[0], robot[1])
        tensor_iteration = (
            calculate_total_repulsive_field_value(obstacle_classes_list, target_object, height, width,
                                                  attraction_repulsive_tensor=attraction_tensor))

        path_iteration, _ = dijkstra_on_nparray_with_dictionary_without_detach(tensor_iteration, robot_object.vector,
                                                                               target_object.vector)

        path_iteration = moving_average_smooth(path_iteration, 25)
        second = time.time()
        print(f"It took {first - second} seconds")
        first = second
        screen.fill(GREY)
        pygame.draw.lines(screen, BLACK, False, path_iteration, 2)
        draw_obstacles(screen, obstacle_classes_list, PURPLE)
        draw_robot(screen, robot_object, BLACK, 5)
        draw_robot(screen, target_object, RED, 10)
        pygame.draw.circle(screen, BLACK, (0, 0), 5)
        pygame.draw.circle(screen, BLACK, (1273, 0), 5)
        pygame.draw.circle(screen, BLACK, (1273, 1010), 5)
        pygame.draw.circle(screen, BLACK, (0, 1010), 5)

        pygame.display.flip()

        cv2.imshow("Image", output)
        pressed_key = cv2.waitKey(1)

    cam.release()
    cv2.destroyAllWindows()


def main_realtime_detection_and_lazy_planning(width, height, frame_history, scaling_factor):
    if frame_history > 32:
        exit("An invalid frame history value was given")
    frame_number = 2 ** frame_history

    (cam, aruco_type, camera_matrix, distortion, obstacle_classes_map, target_object, attraction_tensor, screen, path,
     robot_object) = (first_camera_frame(width, height, frame_number))

    pressed_key = ' '
    done = False
    first = time.time()
    while pressed_key != ord('q') and not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        ret, image_ocv = cam.read()
        image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)

        obstacles_iteration, robot, output = (
            coordinate_estimation_continous(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion,
                                            scaling_factor))

        obstacle_classes_list = update_cache(obstacle_classes_map, obstacles_iteration, frame_number)

        if obstacles_collide_with_path(obstacle_classes_list, path, 5):
            if robot is not None:
                robot_object = Robot(robot[0], robot[1])
            tensor_iteration = (
                calculate_total_repulsive_field_value(obstacle_classes_list, target_object, height, width,
                                                      attraction_repulsive_tensor=attraction_tensor))

            path, _ = dijkstra_on_nparray_with_dictionary_without_detach(tensor_iteration,
                                                                         robot_object.vector,
                                                                         target_object.vector)
            path = moving_average_smooth(path, 25)

        second = time.time()
        print(f"One update took {first - second} seconds")
        first = second
        screen.fill(GREY)
        draw_obstacles(screen, obstacle_classes_list, PURPLE)
        pygame.draw.lines(screen, BLACK, False, path, 2)
        draw_robot(screen, robot_object, BLACK, 5)
        draw_robot(screen, target_object, RED, 10)

        pygame.display.flip()

        cv2.imshow("Image", output)
        pressed_key = cv2.waitKey(1)

    cam.release()
    cv2.destroyAllWindows()


def main_realtime_detection_and_lazy_guessing_planning(width, height, frame_seen_history, scaling_factor,
                                                       velocity_history=30):
    if frame_seen_history > 32:
        exit("An invalid frame history value was given")
    frame_number = 2 ** frame_seen_history

    (cam, aruco_type, camera_matrix, distortion, obstacle_classes_map, target_object, attraction_tensor, screen, path,
     robot_object) = (first_camera_frame(width, height, frame_number, velocity_history))

    pressed_key = ' '
    done = False
    first = time.time()
    while pressed_key != ord('q') and not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True

        ret, image_ocv = cam.read()
        image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)

        obstacles_iteration, robot, output = (
            coordinate_estimation_continous(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion,
                                            scaling_factor))

        obstacle_classes_list = update_cache(obstacle_classes_map, obstacles_iteration, frame_number, velocity_history)
        collision, recalculate = obstacles_collide_with_path_with_waiting(obstacle_classes_map, path, 5)

        if not collision:
            if len(path) <= 1:
                exit("The goal was reached and trajectory planning is over")
            path = path[1:]
            robot_object.vector = (int(path[0][0]), int(path[0][1]))

        if collision and recalculate:
            #if robot is not None:
            #robot_object = Robot(robot[0], robot[1])
            tensor_iteration = (
                calculate_total_repulsive_field_value(obstacle_classes_list, target_object, height, width,
                                                      attraction_repulsive_tensor=attraction_tensor))

            path, _ = dijkstra_on_nparray_with_dictionary_without_detach(tensor_iteration,
                                                                         robot_object.vector,
                                                                         target_object.vector)
            path = moving_average_smooth(path, 100)

        second = time.time()
        print(f"One update took {first - second} seconds")
        first = second
        screen.fill(GREY)
        draw_obstacles(screen, obstacle_classes_list, PURPLE)
        pygame.draw.lines(screen, BLACK, False, path, 2)
        pygame.draw.circle(screen, BLACK, (0, 0), 5)
        pygame.draw.circle(screen, BLACK, (1273, 0), 5)
        pygame.draw.circle(screen, BLACK, (1273, 1010), 5)
        pygame.draw.circle(screen, BLACK, (0, 1010), 5)
        draw_robot(screen, robot_object, BLACK, 5)
        draw_robot(screen, target_object, RED, 10)

        pygame.display.flip()

        cv2.imshow("Image", output)
        pressed_key = cv2.waitKey(1)

    cam.release()
    cv2.destroyAllWindows()


def obstacles_collide_with_path_with_waiting(obstacles_list_map, path, robot_radius):
    minimum_one_collision = False
    for key, value in obstacles_list_map.items():
        obstacle, cache, velocity_history = value
        for coordinate in path:
            collision, recalculation = obstacle.collides_even_with_waiting(coordinate, robot_radius, velocity_history)
            if collision:
                minimum_one_collision = True
                if recalculation:
                    return True, True
    return minimum_one_collision, False


def obstacles_collide_with_path(obstacle_list, path, robot_radius):
    first = time.time()
    for coordinate in path:
        for obstacle in obstacle_list:
            if obstacle.collides_with_point(coordinate, robot_radius):
                print("We are colliding")
                return True
    second = time.time()
    print(f"It took {second - first} seconds to check if path collides")


def main_realtime_detection_without_visualization(width, height):
    path_to_calibration_images = get_images()
    camera_matrix, distortion, _, _ = calibrate_camera(path_to_calibration_images)
    shutil.rmtree(path_to_calibration_images)

    aruco_type = "DICT_4x4_50"

    cam = cv2.VideoCapture(0)

    ret, image_ocv = cam.read()
    image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)
    target = None
    robot = None
    obstacles = []
    while target is None and robot is None:
        obstacles, r, t, output = \
            coordinate_estimation_first(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion, 4 / 5)
        target = t if target is None else target
        robot = r if robot is None else robot

    obstacle_classes = []
    for obstacle in obstacles:
        obstacle_classes.append(StaticCircle(obstacle[0], obstacle[1], 30, 3, 25))

    target_object = StaticCircle(target[0], target[1], 30, 3)
    robot_object = Robot(robot[0], robot[1])
    attraction_tensor = calculate_attraction_field_value_tensor(target_object, height, width)

    tensor = (
        calculate_total_repulsive_field_value(obstacle_classes, target_object, height, width,
                                              attraction_repulsive_tensor=attraction_tensor))

    path_orig, _ = dijkstra_on_nparray_with_dictionary_without_detach(tensor, robot_object.vector,
                                                                      target_object.vector)

    pressed_key = ' '
    first = time.time()
    while pressed_key != ord('q'):

        ret, image_ocv = cam.read()
        image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)

        obstacles_iteration, robot, output = (
            coordinate_estimation_continous(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion, 4 / 5))
        obstacles_classes_iteration = []
        for obstacle in obstacles_iteration:
            obstacles_classes_iteration.append(StaticCircle(obstacle[0], obstacle[1], 30, 3, 25))

        if robot is not None:
            robot_object = Robot(robot[0], robot[1])

        tensor_iteration = (
            calculate_total_repulsive_field_value(obstacles_classes_iteration, target_object, height, width,
                                                  attraction_repulsive_tensor=attraction_tensor))

        path_iteration, _ = dijkstra_on_nparray_with_dictionary_without_detach(tensor_iteration,
                                                                               robot_object.vector,
                                                                               target_object.vector)

        second = time.time()
        print(f"It took {first - second} seconds")
        first = second

        cv2.imshow("Image", output)
        pressed_key = cv2.waitKey(1)

    cam.release()
    cv2.destroyAllWindows()


def moving_average_smooth(path, window_size):
    path_array = np.array(path)
    n = len(path_array)
    smoothed_path = []

    for i in range(n):
        # Calculate dynamic window size
        current_window_size = min(window_size, i + 1, n - i)
        half_window = current_window_size // 2

        # Calculate the window range
        start = max(0, i - half_window)
        end = min(n, i + half_window + 1)

        # Calculate the average of the points within the window
        window_points = path_array[start:end]
        smoothed_point = np.mean(window_points, axis=0)
        smoothed_path.append(tuple(smoothed_point))

    return smoothed_path


if __name__ == "__main__":
    print(cv2.__version__)
    scaling_factor = 7/8
    main_realtime_detection_and_lazy_guessing_planning(int(1155 * scaling_factor), int(1455 * scaling_factor),
                                                       8, scaling_factor)
