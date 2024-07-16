import pygame
import time
import shutil
import pyzed.sl as sl
import cv2
from dijkstra import dijkstra_on_nparray_with_dictionary_without_detach
from visualization_engine import draw_robot, draw_obstacles, visualizing_dijkstra, check_validity_of_obstacles, \
    GREY, BLACK, RED, PURPLE, visualization_heat_map_tensor
#from environments.environment_15 import obstacles, agent, target, SCREEN_WIDTH, SCREEN_HEIGHT
from math_tensor import calculate_total_repulsive_field_value
from coordinate_estimation import coordinate_estimation, ARUCO_DICT
from camera_calibration import calibrate_camera
from get_calibration_images import get_images
from staticcircle import StaticCircle


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


def main_realtime_detection(height, width): 
    pygame.init() 
    pygame.display.set_caption("Environment")
    screen = pygame.display.set_mode((height + 5, width + 5))
    
    path_to_calibration_images = get_images()
    camera_matrix, distortion, _, _ = calibrate_camera(path_to_calibration_images)
    shutil.rmtree(path_to_calibration_images) 

    aruco_type = "DICT_4X4_100"
    
    zed = sl.Camera() 

    input_type = sl.InputType()
    init = sl.InitParameters(input_t=input_type)
    init.camera_resolution = sl.RESOLUTION.HD1080
    init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init.coordinate_units = sl.UNIT.MILLIMETER

    err = zed.open(init)
    if err != sl.ERROR_CODE.SUCCESS: 
        print(repr(err))
        zed.close()
        exit(1)

    runtime_parameters = sl.RuntimeParameters()

    image_size = zed.get_camera_information().camera_configuration.resolution

    image_zed = sl.Mat(image_size.width, image_size.height, sl.MAT_TYPE.U8_C4)

    pressed_key = ' '
    done = False 
    while pressed_key != ord('q') and not done: 
        for event in pygame.event.get(): 
            if event.type == pygame.QUIT:
                    done = True

        err = zed.grab(runtime_parameters)
        if err == sl.ERROR_CODE.SUCCESS: 
            zed.retrieve_image(image_zed, sl.VIEW.LEFT)
            image_ocv = image_zed.get_data()

            image_ocv = cv2.cvtColor(image_ocv, cv2.COLOR_RGBA2RGB)

            obstacles, output = coordinate_estimation(image_ocv, ARUCO_DICT[aruco_type], camera_matrix, distortion)
            obstacles_classes = []  
            print(obstacles)
            for obstacle in obstacles: 
                obstacles_classes.append(StaticCircle(obstacle[0], obstacle[1], 50, 3, 2))

            screen.fill(GREY)   
            pygame.display.flip()
            draw_obstacles(screen, obstacles_classes, PURPLE)
            pygame.display.flip()


            cv2.imshow("Image", output)
            pressed_key = cv2.waitKey(5)

    cv2.destroyAllWindows()
    zed.close()

main_realtime_detection(873, 1036)