from src.dijkstra import dijkstra_on_tensor_with_tensor, dijkstra_on_tensor_with_dictionary, \
    dijkstra_on_graph_with_dictionary, dijkstra_on_tensor_with_array, dijkstra_on_nparray_with_array, \
    dijkstra_on_nparray_with_dictionary, dijkstra_on_nparray_with_two_array, \
    dijkstra_on_nparray_with_dictionary_without_detach
from timeit import timeit
import torch

"""Got the idea of wrappers from here: https://stackoverflow.com/questions/43489341/python-function-undefined."""


def wrapper(func, *args, **kwargs):
    def wrapped():
        return func(*args, **kwargs)

    return wrapped


def benchmark(tensor, start, goal, number_iteration):
    print(f"Benchmarking on {number_iteration} rounds")
    wrapped_tensor_with_tensor = wrapper(dijkstra_on_tensor_with_tensor, tensor, start, goal)
    wrapped_tensor_with_dictionary = wrapper(dijkstra_on_tensor_with_dictionary, tensor, start, goal)
    wrapped_tensor_with_array = wrapper(dijkstra_on_tensor_with_array, tensor, start, goal)
    wrapped_nparray_with_dictionary_with_detach = wrapper(dijkstra_on_nparray_with_dictionary, tensor, start, goal)
    wrapped_nparray_with_dictionary_without_detach = wrapper(dijkstra_on_nparray_with_dictionary_without_detach, tensor,
                                                             start, goal)
    wrapped_nparray_with_array = wrapper(dijkstra_on_nparray_with_array, tensor, start, goal)
    wrapped_nparray_with_two_array = wrapper(dijkstra_on_nparray_with_two_array, tensor, start, goal)
    wrapped_graph = wrapper(dijkstra_on_graph_with_dictionary, tensor, start, goal)

    time_tensor_with_tensor = timeit(wrapped_tensor_with_tensor, number=number_iteration)
    print(f"Time taken for tensor dijkstra-algorithm with tensor: {time_tensor_with_tensor} seconds")

    time_tensor_with_dictionary = timeit(wrapped_tensor_with_dictionary, number=number_iteration)
    print(f"Time taken for tensor dijkstra-algorithm with dictionary: {time_tensor_with_dictionary} seconds")

    time_tensor_with_array = timeit(wrapped_tensor_with_array, number=number_iteration)
    print(f"Time taken for tensor dijkstra-algorithm with dictionary: {time_tensor_with_array} seconds")

    time_nparray_with_dictionary = timeit(wrapped_nparray_with_dictionary_with_detach, number=number_iteration)
    print(
        f"Time taken for array dijkstra-algorithm with dictionary with detach: {time_nparray_with_dictionary} seconds")

    time_nparray_with_dictionary_without_detach = timeit(wrapped_nparray_with_dictionary_without_detach,
                                                         number=number_iteration)
    print(
        f"Time taken for array dijkstra-algorithm with dictionary without detach: {time_nparray_with_dictionary_without_detach} seconds")

    time_nparray_with_array = timeit(wrapped_nparray_with_array, number=number_iteration)
    print(f"Time taken for array dijkstra-algorithm with array: {time_nparray_with_array} seconds")

    time_nparray_with_two_array = timeit(wrapped_nparray_with_two_array, number=number_iteration)
    print(f"Time taken for array dijkstra-algorithm with two array: {time_nparray_with_two_array} seconds")

    time_graph = timeit(wrapped_graph, number=number_iteration)
    print(f"Time taken for graph dijkstra-algorithm with dictionary: {time_graph} seconds")


def small_graph():
    t = torch.tensor([[1, 1, 1, 1], [5, 4, 1, 1], [7, 9, 1, 1]])
    benchmark(t, (0, 0), (2, 2), 100)


def random_medium_graph():
    t = torch.rand(640, 800).abs()
    benchmark(t, (0, 0), (690, 420),  3)


if __name__ == "__main__":
    random_medium_graph()
