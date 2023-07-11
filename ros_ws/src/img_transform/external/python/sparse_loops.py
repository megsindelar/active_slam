import jax
import jax.numpy as np
import numpy as onp
from jax import vmap, grad, jit
from jax.lax import scan
import matplotlib.pyplot as plt


# read in g2o files
def read_g2o(fname):
    est_vertices = []
    edges = []
    measurements = []
    info_matrices = []
    unctys = []
    num_poses = 0
    with open(fname) as fh:
        for line in fh.readlines(): # read in every line
            content = line.split(' ')
            if(content[0] == 'EDGE_SE2'): # if it's an edge
                # indices of the edge
                idx1 = int(content[1])
                idx2 = int(content[2])
                if(idx2 > 1000 or idx1 > 1000):
                    continue
                if(idx2 == idx1 + 1):
                    num_poses = idx2
                edge = onp.array([idx1, idx2], dtype=int)
                edges.append(edge)

                # measurement of the edge
                dx = float(content[3])
                dy = float(content[4])
                dth = float(content[5])
                measurement = onp.array([dx, dy, dth])
                measurements.append(measurement.copy())

                # information matrix of the edge
                info_mat = onp.array([
                    [float(content[6]), float(content[7]), float(content[8])],
                    [float(content[7]), float(content[8]), float(content[9])],
                    [float(content[8]), float(content[9]), float(content[10])]
                ])
                info_matrices.append(info_mat.copy())
                unctys.append(np.linalg.det(info_mat))

    edges = np.array(edges)
    measurements = np.array(measurements)
    info_matrices = np.array(info_matrices)
    unctys = np.array(unctys)
    return num_poses, edges, measurements, info_matrices, unctys


num_poses, edges, measurements, info_matrices, unctys = read_g2o('../data/2D/intel.g2o')
print(num_poses)
print(edges.shape)
print(measurements.shape)
print(info_matrices.shape)
print(unctys.shape)


full_graph_mat = onp.zeros((num_poses+1, num_poses+1))
for i in range(edges.shape[0]):
    ed = edges[i]
    uncty = unctys[i]
    full_graph_mat[ed[0], ed[1]] = uncty
full_graph_mat = np.array(full_graph_mat)
edges = edges * 1.0


from tinygp import kernels
from tinygp import GaussianProcess


def build_gp(theta, X):
    return GaussianProcess(
        kernels.ExpSquared(theta),
        X
    )


def neg_log_ll(theta, X, y):
    gp = build_gp(theta, X)
    return -gp.condition(y)


theta_init = 1.0
obj = jax.jit(jax.value_and_grad(neg_log_ll))
print(f"Initial negative log likelihood: {obj(theta_init, edges, unctys)[0]}")


from scipy.optimize import minimize


soln = minimize(obj, theta_init, jac=True, args=(edges, unctys))
print(f"Final converged negative likelihood: {soln.fun}")


import fls


ks = fls.config_k2(100, 0.0, num_poses*1.0)
ck = fls.get_coefficients(edges, unctys, ks, 0.0, num_poses*1.0)
print(ck.shape)

ss, X, Y = fls.config_ss2(0.0, 1.0*num_poses, 200)
print(ss.shape)
gp = build_gp(soln.x, edges)
gp.condition(unctys)
mu = gp.predict(ss)
print(mu.shape)

dist_recon = fls.fk(ss, ks, 0.0).T @ ck
dist_recon = dist_recon.reshape(X.shape)

cmap = 'Greys'
fig, ax = plt.subplots(1, 2, tight_layout=False)
# ax.contourf(X, Y, mu.reshape(X.shape), cmap='hot', levels=50)
# ax.imshow(mu.reshape(X.shape), origin='lower')
ax[0].pcolormesh(mu.reshape(X.shape).T, cmap=cmap)
ax[0].set_aspect('equal')
ax[1].pcolormesh(full_graph_mat, cmap=cmap)
# ax[1].pcolormesh(dist_recon, cmap=cmap)
ax[1].set_aspect('equal')
plt.show()
plt.close()
