import unittest
import torch
from src.dijkstra import dijkstra_on_tensor_with_tensor


class Test_Dijkstra_Tensor_With_Tensor(unittest.TestCase):

    def test_dijkstra_on_tensor_with_dictionary_01(self):
        potential_field_value_tensor = torch.tensor([[1, 1, 1], [5, 4, 1], [7, 9, 1]])
        path, _ = dijkstra_on_tensor_with_tensor(potential_field_value_tensor, (0, 0), (2, 2))
        solution = ((0, 0), (1, 0), (2, 0), (2, 1), (2, 2))

        if len(solution) != len(path):
            self.fail("The length of your path and the solution path do not match")

        for index, coordinates in enumerate(path):
            if coordinates[0] != solution[index][0] or coordinates[1] != solution[index][1]:
                self.fail(f"The path entries do not match. You are taking a suboptimal path. The path should be "
                          f"{solution}, but you took {path}")

    def test_dijkstra_on_tensor_with_dictionary_02(self):
        potential_field_value_tensor = torch.tensor([[1, 1, 1, 1], [5, 4, 1, 1], [7, 9, 1, 1]])
        path, _ = dijkstra_on_tensor_with_tensor(potential_field_value_tensor, (0, 0), (2, 2))
        solution = ((0, 0), (1, 0), (2, 0), (2, 1), (2, 2))

        if len(solution) != len(path):
            self.fail("The length of your path and the solution path do not match")

        for index, coordinates in enumerate(path):
            if coordinates[0] != solution[index][0] or coordinates[1] != solution[index][1]:
                self.fail(
                    f"The path entries do not match. You are taking a suboptimal path. The path should be {solution}, "
                    f"but you took {path}")


if __name__ == "__main__":
    unittest.main()
