import pygame
import time
from dijkstra import dijkstra_on_nparray_with_dictionary_without_detach
from visualization_engine import draw_robot, draw_obstacles, visualizing_dijkstra, check_validity_of_obstacles, \
    SCREEN_WIDTH, SCREEN_HEIGHT, GREY, BLACK, RED, PURPLE, visualization_heat_map_tensor
from environments.environment_1 import obstacles, agent, target
from math_tensor import calculate_total_repulsive_field_value


def main():
    check_validity_of_obstacles(obstacles)
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


if __name__ == "__main__":
    main()
