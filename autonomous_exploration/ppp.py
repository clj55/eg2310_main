
import numpy as np
# def fGroups(groups):
#     print("FGROUPS")
#     print(groups.items())
#     sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
#     return sorted_groups
#     top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]    
#     return top_five_groups


# groups = {1: [(6, 34)], 2: [(17, 24), (17, 25), (18, 26), (18, 27), (19, 28), (19, 29), (19, 30), (20, 31), (20, 32), (20, 33), (21, 34), (21, 35)]}
# print(fGroups(groups))

array = np.array([[1,2,3],
                  [4,5,6]])
print(array.shape)
# [(2, [(17, 24), (17, 25), (18, 26), (18, 27), (19, 28), (19, 29), (19, 30), (20, 31), (20, 32), (20, 33), (21, 34), (21, 35)])]
