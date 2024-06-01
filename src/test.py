import torch

# Example tensors
i_s_chunked = [torch.randn(641, 801, 2)]
to_be_intertwined = [torch.randn(641, 801, 3)]

# Concatenate tensors along the second dimension
catted = torch.cat((i_s_chunked[0], to_be_intertwined[0]), dim=2)

print(catted.size())  # Output: torch.Size([641, 801, 5])