from registration import *

A = load_cloud("clouds/bunny/bun000.ply")
B = load_cloud("clouds/bunny/bun045.ply")

R, t, n, final_error = icp(A, B, min_error=0.001)

reg = np.matmul(A, R) + t

draw_clouds([B, reg])
