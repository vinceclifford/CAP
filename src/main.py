import pygame
import time
from dijkstra import dijkstra_on_nparray_with_dictionary_without_detach
from visualization_engine import draw_robot, draw_obstacles, visualizing_dijkstra, check_validity_of_obstacles, \
    SCREEN_WIDTH, SCREEN_HEIGHT, GREY, BLACK, RED, PURPLE, visualization_heat_map_tensor
from environments.environment_7 import obstacles, agent, target
from math_tensor import calculate_total_repulsive_field_value


def main():
    check_validity_of_obstacles(obstacles)
    pygame.init()
    pygame.display.set_caption("Environment")
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    screen.fill(GREY)
    clock = pygame.time.Clock()

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


"""
def main_2():
    obstacle_1 = Static_Polygon([(100, 100), (150, 150), (200, 100), (150, 50)], 5, 3, no_interference=2)
    obstacle_2 = Static_Polygon([(260, 10), (260, 300), (300, 20)], 5, 3, no_interference=2)

    tensor = torch.arange(0, 800 + 1, dtype=torch.float32)
    tensor = tensor.unsqueeze(0).repeat(640 + 1, 1)
    tensor = fill_infinite_polygon_repulsive_field_value([obstacle_2], tensor)
    visualization_heat_map_tensor(tensor)"""

"""
def main_2():
    obstacle_1 = Static_Polygon([(10, 10), (10, 610), (10, 610)], 5, 3, no_interference=2)
    obstacle_2 = Static_Polygon([(130, 10), (130, 610), (130, 610)], 5, 3, no_interference=2)
    obstacle_3 = Static_Polygon([(250, 10), (250, 610), (250, 610)], 5, 3, no_interference=2)
    obstacle_4 = Static_Polygon([(10, 10), (250, 10), (250, 10)], 5, 3, no_interference=2)
    obstacle_5 = Static_Polygon([(260, 10), (260, 610), (260, 610)], 5, 3, no_interference=2)
    obstacle_6 = Static_Polygon([(400, 10), (400, 610), (400, 610)], 5, 3, no_interference=2)
    obstacle_7 = Static_Polygon([(260, 300), (400, 300), (400, 300)], 5, 3, no_interference=2)
    obstacle_8 = Static_Polygon([(260, 10), (400, 10), (400, 10)], 5, 3, no_interference=2)
    obstacle_9 = Static_Polygon([(410, 10), (600, 10), (600, 10)], 5, 3, no_interference=2)
    obstacle_10 = Static_Polygon([(410, 10), (410, 610), (410, 610)], 5, 3, no_interference=2)
    obstacle_11 = Static_Polygon([(410, 300), (600, 300), (600, 300)], 5, 3, no_interference=2)
    obstacle_12 = Static_Polygon([(600, 10), (600, 300), (600, 300)], 5, 3, no_interference=2)
    obstacle_13 = Static_Polygon([(610, 10), (790, 10), (790, 10)], 5, 3, no_interference=2)
    obstacle_14 = Static_Polygon([(610, 10), (610, 300), (610, 300)], 5, 3, no_interference=2)
    obstacle_15 = Static_Polygon([(610, 300), (790, 300), (790, 300)], 5, 3, no_interference=2)
    obstacle_16 = Static_Polygon([(790, 300), (790, 630), (790, 630)], 5, 3, no_interference=2)
    obstacle_17 = Static_Polygon([(790, 630), (10, 630), (10, 630)], 5, 3, no_interference=2)

    obstacles = [
        obstacle_1,
        obstacle_2,
        obstacle_3,
        obstacle_4,
        obstacle_5,
        obstacle_6,
        obstacle_7,
        obstacle_8,
        obstacle_9,
        obstacle_10,
        obstacle_11,
        obstacle_12,
        obstacle_13,
        obstacle_14,
        obstacle_15,
        obstacle_16,
        obstacle_17
    ]

    s_time = time.time()

    res = calculate_polygon_repulsive_field_value_tensor(obstacles, 800, 640)
    e_time = time.time()

    print(f"It took {e_time - s_time} seconds")
    print(res)
"""

if __name__ == "__main__":
    main()
