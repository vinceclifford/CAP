import pygame
import time
import shutil
import cv2
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


def update_cache(obstacle_classes_map, other_map, frame_number):
    for key, coordinates in other_map.items():
        obstacle_classes_map[key] = (StaticCircle(coordinates[0], coordinates[1], 30, 3, 25), 1)

    difference_set = obstacle_classes_map.keys() - other_map.keys()

    for key in difference_set:
        circle, last_seen = obstacle_classes_map[key]
        if last_seen < frame_number:
            obstacle_classes_map[key] = circle, last_seen << 1
        else:
            del obstacle_classes_map[key]

    obstacle_classes_list = []
    for key, value in obstacle_classes_map.items():
        circle_object, lifetime = value
        obstacle_classes_list.append(circle_object)

    return obstacle_classes_list


def main_realtime_detection(width, height, frame_history):
    if frame_history > 32:
        exit("An invalid frame history value was given")
    frame_number = 2 ** frame_history
    pygame.init()
    pygame.display.set_caption("Environment")
    screen = pygame.display.set_mode((height + 5, width + 5))

    path_to_calibration_images = get_images()
    camera_matrix, distortion, _, _ = calibrate_camera(path_to_calibration_images)
    shutil.rmtree(path_to_calibration_images)

    aruco_type = "DICT_4x4_50"

    cam = cv2.VideoCapture(0)

    ret, image_ocv = cam.read()
    image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)
    target = None
    robot = None

    # obstacles maps id to coordinates in the current frame. obstacle_classes_map maps ids to StaticCircle as well as last
    # time seen.
    # obstacles: int -> (int,int)
    # obstacle_classes_map: int -> (StaticCircle, int)
    obstacles = {}
    obstacle_classes_map = {}
    while target is None and robot is None:
        obstacles, r, t, output = \
            coordinate_estimation_first(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion)
        target = t if target is None else target
        robot = r if robot is None else robot
        print(f"Fetching locations: Robot: {robot}, target{target}")

    obstacle_classes_list = update_cache(obstacle_classes_map, obstacles, frame_number)

    target_object = StaticCircle(target[0], target[1], 30, 3)
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
            coordinate_estimation_continous(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion))

        obstacle_classes_list = update_cache(obstacle_classes_map, obstacles_iteration, frame_number)

        if robot is not None:
            robot_object = Robot(robot[0], robot[1])
        tensor_iteration = (
            calculate_total_repulsive_field_value(obstacle_classes_list, target_object, height, width,
                                                  attraction_repulsive_tensor=attraction_tensor))

        path_iteration, _ = dijkstra_on_nparray_with_dictionary_without_detach(tensor_iteration, robot_object.vector,
                                                                               target_object.vector)

        second = time.time()
        print(f"It took {first - second} seconds")
        first = second
        screen.fill(GREY)
        pygame.draw.lines(screen, BLACK, False, path_iteration, 2)
        draw_obstacles(screen, obstacle_classes_list, PURPLE)
        draw_robot(screen, robot_object, BLACK, 5)
        draw_robot(screen, target_object, RED, 10)

        pygame.display.flip()

        cv2.imshow("Image", output)
        pressed_key = cv2.waitKey(1)

    cam.release()
    cv2.destroyAllWindows()


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
            coordinate_estimation_first(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion)
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
            coordinate_estimation_continous(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion))
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


if __name__ == "__main__":
    print(cv2.__version__)
    main_realtime_detection(int(1455 / 2), int(1155 / 2), 8)
