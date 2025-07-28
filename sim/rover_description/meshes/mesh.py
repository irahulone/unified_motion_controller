import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from stl import mesh
import matplotlib.pyplot as plt

# STLファイルを読み込む
your_mesh = mesh.Mesh.from_file('pioneer_3dx.stl')

# Figureを作成
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# メッシュの座標を取得
ax.add_collection3d(Poly3DCollection(your_mesh.vectors, alpha=0.5, edgecolor='k'))

# 軸のスケールを調整
scale = your_mesh.points.flatten()
ax.auto_scale_xyz(scale, scale, scale)

# 表示
plt.show()
