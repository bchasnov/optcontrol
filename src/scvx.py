import cvxpy as cp

from argparse import Namespace

class scvx(*modes, **kwargs):
    """Successive convexification for trajectory generation.

    Supported features:
    * Objectives
      - convex: minimum fuel
      - non-convex: minimum time
    * Dynamics
      - linear: double integrator, zero-order hold, first-order hold
      - non-linear: general $\dot x = f(x,u)$ system.
    * Constraints
      - state: keep-in zones, keep-out zones, partially-restricted zones
      - control: minimum thrust, angle-of-attack, lossless convexification
      - logic: state-triggered constraints, linear temporal logic


    >> init = scvx.Problem(scvx.MINIMUM_TIME, [scvx.DOUBLE_INTEGRATOR, scvx.FIRST_ORDER_HOLD])
    >> config = Namespace(tol=1e-3)
    >> parameters = dict(n_horizon=15, n_waypoints=1)
    >> inputs = dict(max_velocity=1.)
    >> opt, step = init(**parameters)
    >> for i in range(max_iter):
    >>     inputs.max_velocity = 1
    >>     opt = step(opt, **inputs)
    >>     print(status)
    >>     if opt.has_converged <= config.tol: break
    >> print opt.outputs

    """

    def Problem(objectives, constraints):
        
        def init(**kwargs):
            # Setup cvxpy variables
            #r = cp.Variable(...)
            
            # Setup problem
            #prob = cp.Problem(...)
            
            # Store all relevant variables and parameters
            #opt = (r,v,a)
            
            def step(opt, **kwargs):
                # Update inputs and operating point, then solve one convex sub-problem.
                #prob.solve()
                raise NotImplemented
                
            #return opt, step
            raise NotImplemented
        
        return init
        
    
    
