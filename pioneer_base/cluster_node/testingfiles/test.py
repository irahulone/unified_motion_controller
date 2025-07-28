from Cluster import Cluster
import numpy as np
import math
clust = Cluster()
clust = Cluster(KPgains=[0.25]*9, KVgains=[0.25]*9)
r = np.array([[3], [6], [0], [1], [1], [0], [5], [1], [0]])
rd = np.array([[0], [0], [0], [0], [0], [0], [0], [0], [0]])
print(clust.getVelocityCommand(r, rd))
