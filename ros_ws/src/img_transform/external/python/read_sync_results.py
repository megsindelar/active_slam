import numpy as np
import matplotlib.pyplot as plt


# raw_data = np.loadtxt("/home/msun/workspace/spider/C++/build/test_sesync.txt")
raw_data = np.loadtxt("/home/msun/workspace/spider/C++/build/test_cplslam.txt")
print(raw_data.shape)

num_poses = int(raw_data.shape[1] / 3)
print("num of poses: " , num_poses)

fig, ax = plt.subplots(1, 1, tight_layout=True)

poses = np.zeros((num_poses, 2))
for i in range(num_poses):
    j = num_poses + i + 1

    se2 = np.zeros((3, 3))
    se2[0:2, 2] = raw_data[0:2, i]
    se2[0:2, 0:2] = raw_data[0:2, j:j+2]

    poses[i] = raw_data[0:2, i]
    # print(se2)

ax.plot(poses[:,0], poses[:,1], '-o')
plt.show()
plt.close()
