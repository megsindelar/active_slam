"""
Library for functional least square implemented in JAX

References:
    [1] Mathew, George, and Igor Mezić. "Metrics for ergodicity and design of ergodic dynamics for multi-agent systems." Physica D: Nonlinear Phenomena 240.4-5 (2011): 432-442.
    [2] Miller, Lauren. Optimal ergodic control for active search and information acquisition. Diss. Northwestern University, 2015.
    [3] Abraham, Ian. Optimal Experimental Learning and Infinite Linear Embeddings. Diss. Northwestern University, 2020.
"""

import jax
import jax.random as rnd
import jax.numpy as np
from jax import jit, vmap, grad
from jax.lax import scan, cond
from jax.ops import index, index_add, index_update
from jax.scipy.linalg import expm # TODO: reimplement expm for SE(2/3)
from jax.scipy.optimize import minimize as jmin

import numpy as onp
from scipy.optimize import minimize as smin

from functools import partial
import time
# import plotly.graph_objects as go
# import plotly.express as px
# from IPython.display import clear_output
# import matplotlib.pyplot as plt


__author__ = "Muchen Sun"
__copyright__ = "Copyright 2021, Muchen Sun"
__credits__ = ["Ian Abraham", "Todd Murphey"]
__license__ = "GPLv3"
__version__ = "1.1.0"
__maintainer__ = "Muchen Sun"
__email__ = "muchen@u.northwestern.edu"
__status__ = "Alpha"


######################################
# Utility functions
######################################
def test_installation():
    print("package has been successfully installed.")
    print("test local import")


def prngkey(seed=None):
    if(seed is None):
        seed = int( time.time() * 1000 % 1000 )
    print('seed: ', seed)
    return rnd.PRNGKey(seed)


def config_ss2(L1, L2, num):
    dim = 2
    _grid = np.meshgrid(*[
        np.linspace(L1, L2, num)
        for i in range(dim)
    ])
    _ss = np.array([
        _grid[i].ravel() for i in range(dim)
    ]).T

    _ret = _ss, *_grid
    return _ret


def config_ss3(L1, L2, num):
    dim = 3
    _grid = np.meshgrid(*[
        np.linspace(L1, L2, num)
        for i in range(dim)
    ])
    _ss = np.array([
        _grid[i].ravel() for i in range(dim)
    ]).T

    _ret = _ss, *_grid
    return _ret


def config_ss(*specs):
    """Specifying search space.

    Keyword arguments:
    specs -- Specification for each dimension, in format of: (low, high, num)
    """
    dim = len(specs)
    _grid = np.meshgrid(*[
        np.linspace(specs[i][0], specs[i][1], specs[i][2])
        for i in range(dim)
    ])
    _ss = np.array([
        _grid[i].ravel() for i in range(dim)
    ]).T

    _ret = _ss, *_grid
    return _ret


@jit
def cumprod(res, el):
    res = res * el
    return res, res


######################################
# SE(2,3) Operations
######################################
@jit
def action2(pt, mat):
    return mat[0:2,0:2] @ pt + mat[0:2,2]


@jit
def actions2(pts, mat):
    return vmap(action2, in_axes=(0,None))(pts, mat)


@jit
def se3inv(mat):
    mat = index_update(mat, index[0:3, 0:3], mat[0:3,0:3].T)
    mat = index_update(mat, index[0:3, 3], -np.dot(mat[0:3, 0:3], mat[0:3, 3]))
    return mat


@jit
def se3exp(vec):
    return expm(np.array([
        [0.0, -vec[5], vec[4], vec[0]],
        [vec[5], 0.0, -vec[3], vec[1]],
        [-vec[4], vec[3], 0.0, vec[2]],
        [0.0, 0.0, 0.0, 0.0]
    ]))


@jit
def action3(pt, mat):
    return mat[0:3,0:3] @ pt + mat[0:3,3]


@jit
def actions3(pts, mat):
    return vmap(action3, in_axes=(0,None))(pts, mat)


@jit
def daction3(pt, mat):
    R = mat[0:3, 0:3]
    skew_sym_p = np.array([
        [0.0, -pt[2], pt[1]],
        [pt[2], 0.0, -pt[0]],
        [-pt[1], pt[0], 0.0]
    ])
    rmat = np.dot(-R, skew_sym_p)
    return np.array([
        [R[0,0], R[0,1], R[0,2], rmat[0,0], rmat[0,1], rmat[0,2]],
        [R[1,0], R[1,1], R[1,2], rmat[1,0], rmat[1,1], rmat[1,2]],
        [R[2,0], R[2,1], R[2,2], rmat[2,0], rmat[2,1], rmat[2,2]]
    ])


@jit
def dactions3(pts, mat):
    return vmap(daction3, in_axes=(0,None))(pts, mat)


######################################
# SO(2,3) Operations
######################################
@jit
def so2exp(theta):
    return np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta),  np.cos(theta)]
    ])


@jit
def so2log(R):
    return np.arctan(R[1,0] / R[0,0])


@jit
def so3exp(vec):
    th = np.linalg.norm(vec)
    w = vec / th
    skew_sym_w = np.array([
        [0.0, -w[2], w[1]],
        [w[2], 0.0, -w[0]],
        [-w[1], w[0], 0.0]
    ])
    return np.eye(3) + \
           np.sin(th) * skew_sym_w + \
           (1 - np.cos(th)) * skew_sym_w @ skew_sym_w


@jit
def rotate(pt, mat):
    return mat @ pt


@jit
def rotates(pts, mat):
    return vmap(rotate, in_axes=(0,None))(pts, mat)


@jit
def drotate2(pt, mat):
    skew_sym = np.array([[0, -1], [1, 0]])
    return np.dot(np.dot(mat, skew_sym), pt)


@jit
def drotates2(pts, mat):
    return vmap(drotate2, in_axes=(0,None))(pts, mat)


@jit
def drotate3(pt, mat):
    skew_sym_p = np.array([
        [0.0, -pt[2], pt[1]],
        [pt[2], 0.0, -pt[0]],
        [-pt[1], pt[0], 0.0]
    ])
    return np.dot(-mat, skew_sym_p)


@jit
def drotates3(pts, mat):
    return vmap(drotate3, in_axes=(0,None))(pts, mat)


######################################
# FLS functions
######################################
def config_k(*specs):
    """Specifying Fourier coefficient indices.

    Keyword arguments:
    specs -- Specification for each dimension, in format of: (num_k, L), number of coefficients and length of the dimension (L = high - low, from config_ss)
    """
    dim = len(specs)
    _ks = np.meshgrid(*[np.arange(0, specs[i][0], step=1) / specs[i][1] for i in range(dim)])
    _k = np.array([
        _ks[i].ravel() for i in range(dim)
    ]).T

    return _k


def config_k3(numK, L1, L2):
    dim = 3
    dL = L2 - L1
    _ks = np.meshgrid(*[np.arange(0, numK, step=1)/dL for i in range(dim)])
    _k = np.array([
        _ks[i].ravel() for i in range(dim)
    ]).T

    return _k


def config_k2(numK, L1, L2):
    dim = 2
    dL = L2 - L1
    _ks = np.meshgrid(*[np.arange(0, numK, step=1)/dL for i in range(dim)])
    _k = np.array([
        _ks[i].ravel() for i in range(dim)
    ]).T

    return _k


def config_k1(numK, L1, L2):
    dim = 1
    dL = L2 - L1
    _ks = np.meshgrid(*[np.arange(0, numK, step=1)/dL for i in range(dim)])
    _k = np.array([
        _ks[i].ravel() for i in range(dim)
    ]).T

    return _k


@jit
def get_hk(k, L1, L2):
    _hk1 = (2.0*k*L1 + np.sin(2.0*k*L1)) / (4.0*k)
    _hk2 = (2.0*k*L2 + np.sin(2.0*k*L2)) / (4.0*k)
    _hk = _hk2 - _hk1
    _hk = np.nan_to_num(_hk, nan=1.0)
    return np.sqrt(np.prod(_hk, axis=1))


@jit
def get_lamk(k, L1, L2):
    dim = k.shape[1]
    return 1 / (1.0 + np.linalg.norm(k, axis=1) ** 2)


@jit
def fk_unit(x, k, L1):
    return np.prod(np.cos((x-L1) * k * np.pi))


@jit
def dfk2_unit(x, k, L1, L2):
    return np.array([-np.sin((x[0]-L1)*k[0]*np.pi) * np.cos((x[1]-L1)*k[1]*np.pi) * k[0]*np.pi,
                     -np.sin((x[1]-L1)*k[1]*np.pi) * np.cos((x[0]-L1)*k[0]*np.pi) * k[1]*np.pi]) / get_hk(np.array([k]), L1, L2)


@jit
def dfk3_unit(x, k, L1, L2):
    return np.array([-np.sin((x[0]-L1)*k[0]*np.pi) * np.cos((x[1]-L1)*k[1]*np.pi) * np.cos((x[2]-L1)*k[2]*np.pi) * k[0]*np.pi,
                     -np.sin((x[1]-L1)*k[1]*np.pi) * np.cos((x[0]-L1)*k[0]*np.pi) * np.cos((x[2]-L1)*k[2]*np.pi) * k[1]*np.pi,
                     -np.sin((x[2]-L1)*k[2]*np.pi) * np.cos((x[0]-L1)*k[0]*np.pi) * np.cos((x[1]-L1)*k[1]*np.pi) * k[2]*np.pi]) / get_hk(np.array([k]), L1, L2)


@jit
def fk(x, k, L1):
    return vmap(vmap(fk_unit, in_axes=(0,None,None)), in_axes=(None,0,None))(x, k, L1)


@jit
def dfk2(x, k, L1, L2):
    return vmap(vmap(dfk2_unit, in_axes=(0,None,None,None)), in_axes=(None,0,None,None))(x, k, L1, L2) / (x.shape[0])


@jit
def dfk3(x, k, L1, L2):
    return vmap(vmap(dfk3_unit, in_axes=(0,None,None,None)), in_axes=(None,0,None,None))(x, k, L1, L2) / (x.shape[0])


@jit
def get_coefficients(x, w, k, L1, L2):
    return np.mean(fk(x, k, L1) * w, axis=1) / get_hk(k, L1, L2)


@jit
def get_sample_coef(x, k, hk, L1):
    return np.mean(fk(x, k, L1), axis=1) / hk


@jit
def get_distr_coef(ss, p, k, hk, L1):
    return np.mean(fk(ss, k, L1) * p, axis=1) / hk


@jit
def fnorm(coef1, coef2, lamk):
    return np.sum(lamk @ np.square(coef1-coef2))


@jit
def fnorm_L1(coef1, coef2, lamk):
    return np.linalg.norm(lamk * (coef1-coef2), ord=1)


@jit
def fnorm_inf(coef1, coef2, lamk):
    return np.linalg.norm(lamk * (coef1-coef2), ord=np.inf)


@jit
def fnorm_pts2(pts, coef1, k, hk, lamk, L1):
    coef = get_sample_coef(pts, k, hk, L1)
    return fnorm(coef1, coef, lamk)


@jit
def d_fnorm_coef1(coef1, coef2, lamk):
    return lamk * 2.0 * (coef1 - coef2)


@jit
def d_fnorm_coef2(coef1, coef2, lamk):
    return lamk * 2.0 * (coef2 - coef1)


@jit
def d_fnorm2_pts1(pts1, pts2, k, hk, lamk, L1, L2):
    coef1 = get_sample_coef(pts1, k, hk, L1)
    coef2 = get_sample_coef(pts2, k, hk, L1)
    return d_fnorm_coef1(coef1, coef2, lamk) @ dfk2(pts1, k, L1, L2).reshape(k.shape[0], -1)


@jit
def d_fnorm2_pts2(pts1, pts2, k, hk, lamk, L1, L2):
    coef1 = get_sample_coef(pts1, k, hk, L1)
    coef2 = get_sample_coef(pts2, k, hk, L1)
    return d_fnorm_coef2(coef1, coef2, lamk) @ dfk2(pts2, k, L1, L2).reshape(k.shape[0], -1)


@jit
def d_fnorm3_pts1(pts1, pts2, k, hk, lamk, L1, L2):
    coef1 = get_sample_coef(pts1, k, hk, L1)
    coef2 = get_sample_coef(pts2, k, hk, L1)
    return d_fnorm_coef1(coef1, coef2, lamk) @ dfk3(pts1, k, L1, L2).reshape(k.shape[0], -1)


@jit
def d_fnorm3_pts2(pts1, pts2, k, hk, lamk, L1, L2):
    coef1 = get_sample_coef(pts1, k, hk, L1)
    coef2 = get_sample_coef(pts2, k, hk, L1)
    return d_fnorm_coef2(coef1, coef2, lamk) @ dfk3(pts2, k, L1, L2).reshape(k.shape[0], -1)


######################################
# Armijo/Backtracking Line Search
######################################
@jit
def step(xk, a, pk):
    return xk + a * pk


@partial(jit, static_argnums=(0,1,))
def armijo_step(_step_fun, _obj_fun, _a, _c, _xk, _pk, _objk, _pk_norm):
    _xk_new = _step_fun(_xk, -_a, _pk)
    _cond = _objk - _obj_fun(_xk_new) - _c * _a * _pk_norm**2
    return _cond


@partial(jit, static_argnums=(0,1,))
def armijo_steps(_step_fun, _obj_fun, _alist, _c, _xk, _pk, _objk, _pk_norm):
    return vmap(armijo_step, in_axes=(None, None, 0, None, None, None, None, None))(_step_fun, _obj_fun, _alist, _c, _xk, _pk, _objk, _pk_norm)


@partial(jit, static_argnums=(0,1,))
def armijo(_step_fun, _obj_fun, _alist, _c, _xk, _pk, _objk, _pk_norm):
    vec = armijo_steps(_step_fun, _obj_fun, _alist, _c, _xk, _pk, _objk, _pk_norm)
    idx = np.sum((vec<0.0).astype(int))
    return _alist[idx]


@partial(jit, static_argnums=(0, 1, 2, 4, 5, 6, 7))
def line_search_step(_step_fun, _obj, _dobj, _xk, _a, _b, _c, _d):
    _, _alist = scan(cumprod, _a/_b, np.ones(_d) * _b)
    _objk = _obj(_xk)
    _pk = _dobj(_xk)
    _pk_norm = np.linalg.norm(_pk)
    _a = armijo(_step_fun, _obj, _alist, _c, _xk, _pk, _objk, _pk_norm)
    return _step_fun(_xk, -_a, _pk)

