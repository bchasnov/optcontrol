import cvxpy as cp
import numpy as np
import numpy.linalg as la
from argparse import Namespace
import time

K = 25
parameters = dict(K=K, vi=np.zeros(3), ai=np.zeros(3),
                  ri=np.array([100, 10, 50]), 
                  rf=np.array([-100, 10, 50]), 
                  vf=np.zeros(3), af=np.zeros(3),
                  wp=np.array([0, -10, 50]), wp_idx=int(K/2),
                  gravity=np.array([0.,0.,-9.81]),
                  v_fast=5, v_slow=2.5, a_min=5., a_max=12.0, 
                  theta_max=np.radians(20.0),
                  j_max=40, max_iter=30,
                  weight_ri=1e3, weight_rf=1e4, weight_wp=1e4,
                  weight_obs=1e2, weight_trust=1e-2, n_obs=1)

""" Initialize parameters """
P = Namespace(**parameters)

ri = cp.Parameter(3)
vi = cp.Parameter(3)
ai = cp.Parameter(3)
rf = cp.Parameter(3)
vf = cp.Parameter(3)
af = cp.Parameter(3)
wp = cp.Parameter(3)

w_rf = cp.Parameter()
w_wp = cp.Parameter()
w_trust = cp.Parameter()
# w_obs = cp.Parameter()
# w_ri = cp.Parameter()

ri.value = P.ri
vi.value = P.vi
ai.value = P.ai
rf.value = P.rf
vf.value = P.vf
af.value = P.af
wp.value = P.wp

w_rf.value = P.weight_rf
w_wp.value = P.weight_wp
w_trust.value = P.weight_trust
# w_obs.value = P.weight_obs
# w_ri.value = P.weight_ri

""" Initialize variables """
r = cp.Variable((P.K, 3))
v = cp.Variable((P.K, 3))
a = cp.Variable((P.K, 3))
dtau = cp.Variable()

r0 = cp.Parameter((P.K, 3))
v0 = cp.Parameter((P.K, 3))
a0 = cp.Parameter((P.K, 3))
tau = cp.Parameter()
#stc = cp.Parameter(P.K)

s = cp.Variable(P.K, nonneg=True)
eta = cp.Variable((P.K, P.n_obs), nonneg=True)
delta = cp.Variable((P.K, 2), nonneg=True)
trust = cp.Variable(nonneg=True)

rf_relax = cp.Variable(3)
ri_relax = cp.Variable(3)
wp_relax = cp.Variable(3)

s_ri = cp.Variable()
s_rf = cp.Variable()
s_wp = cp.Variable()

sigma = cp.Variable(nonneg=True)
# helper variables to maintain disciplined parametric programming
raccel = cp.Variable((P.K-1, 3))
Dt_raccel = cp.Variable((P.K-1, 3))

f = 0
g = dict() 

""" Define cost """
f += cp.norm(s)
f += tau + dtau
# f += w_ri*s_ri
f += w_rf*s_rf
f += w_wp*s_wp
f += w_trust*trust

g["relax_rf"] = s_rf >= cp.norm(rf_relax)**2
g["relax_wp"] = s_wp >= cp.norm(wp_relax)**2
# g["relax_ri"] = s_ri >= cp.norm(ri_relax)

""" Trust Region """
g["trust_dtau"] = cp.abs(dtau) <= trust
# g["trust_pos"] = cp.abs(r[:,:2] - r0[:,:2]) <= delta
# g["trust_obs"] = cp.sum(delta) <= trust

""" Boundary conditions """
g["pos_initial"] = r[0] == ri #+ ri_relax
g["pos_final"] = r[-1] == rf + rf_relax
g["vel_initial"] = v[0] == vi
g["vel_final"] = v[-1] == vf
g["waypoint"] = r[P.wp_idx] == wp + wp_relax
g["accel_initial"] = a[0,:2] == ai[:2]
g["accel_final"] = a[-1,:2] == af[:2]

""" Dynamics """
grav = np.tile(P.gravity, (P.K-1, 1)) 

g["helper_raccel"] = raccel == 1/2*(1/3*(2*a[:-1] + a[1:]) + grav)*tau
g["helper_Dt_raccel"] = Dt_raccel == (1/3*(2*a0[:-1] + a0[1:]) + grav)*dtau

g["dynamics_pos"] = \
r[1:] == r[:-1] + (v[:-1] + raccel)*tau + v0[:-1]*dtau + Dt_raccel*tau
g["dynamics_vel"] = \
v[1:] == v[:-1] + 1/2*(a[:-1] + a[1:] + grav)*tau \
       + 1/2*(a0[:-1] + a0[1:] + grav)*dtau# + v_virtual

""" State constraints """
#g["height_max"] = r[:,2] <= 0
#g["height_min"] = -r[:,2] >= 0
#g["vertical_vel"] = cp.abs(v[:,2]) <= 0
#g["vel_max"] = cp.norm(v, axis=1) <= (1-stc)*P.v_fast + (stc)*P.v_slow
g["horizontal_jerk_max"] = cp.norm(a[:-1] - a[1:], 2, axis=1) <= P.j_max*(tau + dtau)
#g += [P.cpos_A@@r <= np.tile(P.cpos_b, (K,1))]

g["lossless"] = cp.norm(a, axis=1) <= s
g["bank_angle"] = np.cos(P.theta_max)*s <= a[:,2]
g["thrust_min"] = P.a_min <= s 
g["thrust_max"] = s <= P.a_max


prob = cp.Problem(cp.Minimize(f), list(g.values()))

def init():
    zeros = np.zeros((P.K,3))  
    state = dict(r=zeros, v=zeros, a=zeros, dtau=0.5)
    return state, dict(ri=P.ri, rf=P.rf, wp=P.wp)
    
def step(state):
    r0.value = state['r']
    v0.value = state['v']
    a0.value = state['a']
    tau.value = tau.value + state['dtau'] \
            if tau.value != None else state['dtau']
    #stc.value = np.logical_and(r0[:,0].value <=.30, r0[:,0].value >=.25)*1.
    out = prob.solve(solver="ECOS", warm_start=True)
    if prob.status in ["infeasible", "unbounded"]:
        return init()[0]
    return dict(r=r.value, v=v.value, a=a.value, dtau=dtau.value)

def update(inp):
    ri.value = np.array(inp['ri'])
    rf.value = np.array(inp['rf'])
    wp.value = np.array(inp['wp'])

if __name__ == "__main__":
    state, params = init()
    for _ in range(P.max_iter):
        state = step(state)
        print(state)
    
