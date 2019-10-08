import os
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

sns.set_style("darkgrid")

#################################
#           test                #
#################################

# x = np.array([i for i in range(1,1000)])
# y = np.sin(x)
# z = np.log(y)

# fig = plt.figure()
# ax = Axes3D(fig)

# ax.set_xlabel("X")
# ax.set_ylabel("Y")
# ax.set_zlabel("Z")

# ax.plot(x,y,z,marker="o")

# plt.show()

#################################
#         end test              #
#################################




log_path = "../log"
filename = "mylog_107191.csv"
csv_path = os.path.join(log_path,filename)

house_shape = [ [[-0.5,-0.5],[-0.5,10]], [[-0.5,15],[10,10]  ],
                [[15,15],    [10,-0.5]], [[15,-0.5],[-0.5,-0.5]],
                [[1,1],    [0.5,9]    ], [[1,3.5],      [9,9]  ],
                [[3.5,3.5],    [9,0.5]], [[3.5,1], [0.5,0.5]],
                [[5,5],  [10,5]       ], [[5,10],     [5,5]    ],
                [[10,10],     [10,5]   ], [[5,5], [-0.5,4]    ],
                [[5,10],     [4,4]   ], [[10,10],   [4,-0.5]] ]

csv_file = pd.read_csv(csv_path,header=2)
pos_x = csv_file["pos.x"]
pos_y = csv_file["pos.y"]
linadm = csv_file["linadm"]
linsafe = csv_file["linsafe"]
angadm = csv_file["angadm"]
andsafe = csv_file["angsafe"]
vel_h_cost = csv_file["vel_h_cost"]
angl_h_cost = csv_file["ang_h_cost"]
cost = csv_file["cost"]
cal_vel_v = csv_file["cal_vel.v"]
val_vel_w = csv_file["cal_val.w"]


fig = plt.figure()
ax = Axes3D(fig)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("cost")

ax.plot(pos_x,pos_y,cost,marker="o")

plt.show()
plt.close()

fig2 = plt.figure()
plt.plot(pos_x,pos_y)

for i in range(len(house_shape)):
    plt.plot(house_shape[i][0],house_shape[i][1],'k-', lw=4)



plt.show()
plt.close()