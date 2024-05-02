import torch as tp 

t1 = tp.tensor([5.,6.])
t2 = tp.tensor([6.,7.])

squared = tp.sqrt(tp.sum((t1-t2) ** 2))
print(squared.item())
