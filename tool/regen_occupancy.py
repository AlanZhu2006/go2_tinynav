"""Regenerate a LingBot map's occupancy with TinyNav's raycaster (proper free-space carving), in the
gravity-aligned frame (Z=up correct). Run on the robot (ROS env). Overwrites occupancy_grid/meta/sdf."""
import sys, numpy as np, cv2
from tinynav.core.build_map_node import generate_occupancy_map, TinyNavDB
M=sys.argv[1]
P=np.load(f"{M}/poses.npy",allow_pickle=True).item()
P={int(k):np.asarray(v,np.float64) for k,v in P.items()}
K=np.load(f"{M}/intrinsics.npy").astype(np.float64)
db=TinyNavDB(M, is_scratch=False)
grid,origin,img2d,sdf=generate_occupancy_map(P, db, K, 0.0, resolution=0.1, step=10)
np.save(f"{M}/occupancy_grid.npy",grid)
np.save(f"{M}/occupancy_meta.npy",np.array([origin[0],origin[1],origin[2],0.1],np.float32))
np.save(f"{M}/sdf_map.npy",sdf)
cv2.imwrite(f"{M}/occupancy_2d_image.png",img2d)
f=np.max(grid,2); print(f"RAYCAST occupancy {grid.shape} | free {int((f==1).sum())} occupied {int((f==2).sum())} unknown {int((f==0).sum())}")
db.close()
