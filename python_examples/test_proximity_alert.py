import openravepy
import trajoptpy
from trajoptpy.check_traj import traj_is_safe
import json
import time
import numpy as np
import copy

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()

env = openravepy.Environment()
env.StopSimulation()
# env.Load("robots/kuka-youbot.zae")  # arm on mobile platform
env.Load("robots/mitsubishi-pa10.zae")  # standard arm
env.Load("../data/test-block.xml")

# pause every iteration, until you press 'p'.
# Press escape to disable further plotting
trajoptpy.SetInteractive(False)
robot = env.GetRobots()[0]

print env.GetRobots()
print robot.GetManipulators()

arm_name = robot.GetManipulators()[0].GetName()
arm_indices = robot.GetManipulator(arm_name).GetArmIndices()
joint_start = [0] * len(arm_indices)
robot.SetDOFValues(joint_start,
                   arm_indices)

joint_target = [0, 0, 0, 0, 0, 0, 0]

trajoptpy.SetInteractive(False)
robot = env.GetRobots()[0]

request = {
    "basic_info": {
        "n_steps": 1,
        "manip": arm_name,
        # DOF values at first timestep are fixed based on current robot state
        "start_fixed": True
    },
    "costs": [
        {
            "type": "joint_vel",  # joint-space velocity cost
            # a list of length one is automatically expanded
            # to a list of length n_dofs
            "params": {"coeffs": [100]}
        },
        {
            "type": "collision",
            # shorten name so printed table will be prettier
            "name": "disc_coll",
            "params": {
                "continuous": False,
                # penalty coefficients. list of length one is automatically
                # expanded to a list of length n_timesteps
                "coeffs": [20],
                # robot-obstacle distance that penalty kicks in.
                # expands to length n_timesteps
                "dist_pen": [0.025]
            }
        }
    ],
    "constraints": [
        # BEGIN joint_constraint
        {
            "type": "joint",
            "params": {"vals": joint_target}

        },
        # END joint_constraint
        # BEGIN chance_constraint
        {
            "type": "uncertain_obstacles",
            "name": "chance_constr",
            "params": {
                # The list of uncertain KinBodies
                "uncertain_body_names": ['obs0'],
                # As a reminder: covariances must be positive semidefinite
                # This implies that it must be symmetric, so we only specify
                # the six unique values (numbered below):
                #
                #       [0 1 2]
                #       [. 3 4]
                #       [. . 5]
                #
                # This list must be the same length as obstacles above
                "location_covariances": [
                    [0.01, 0.0, 0.0, 0.01, 0.0, 0.01]
                ],
                # The acceptable max. risk at each waypoint. Lists of length 1
                # are expanded to n_timesteps
                "overall_risk_tolerance": 0.1,
                # 10^-6 precision
                "precision": 0.000001,
                "coeff": 1,
                "grad_scale_factor": 0.5
            }

        }
        # END chance_constraint
    ],
    # BEGIN init
    "init_info": {
        # start from previous
        "type": "straight_line",
        "endpoint": joint_target
    }
    # END init
}

# Convert dictionary to JSON string for TrajOpt
s = json.dumps(request)
# Create trajopt problem
prob = trajoptpy.ConstructProblem(s, env)

# time the optimization
t_start = time.time()
# optimize
result = trajoptpy.OptimizeProblem(prob)
t_elapsed_second = time.time() - t_start

# Display results
print result

# save chance-constrained trajectory
cc_traj = result.GetTraj()

# Check trajectories, both initial_traj and cc_traj
# prob.SetRobotActiveDOFs()
# assert traj_is_safe(result.GetTraj(), robot)

# we do this by conducting a bunch of trials, and on each trial we
# perturb the position of the table and check if the trajectory is safe
N = 100

sigma = np.diag([0.01, 0.01, 0.01])

obs0_kinbody = env.GetKinBody('obs0')
obs0_xform_nominal = env.GetKinBody('obs0').GetTransform()

env.SetViewer('qtcoin') # attach viewer (optional)

cc_traj_collisions = 0
for i in range(N):
    # get random perturbation
    perturbation0 = np.concatenate([
        np.random.multivariate_normal([0, 0, 0], sigma),
        [0]])

    obs0_xform_perturbed = copy.copy(obs0_xform_nominal)
    obs0_xform_perturbed[:, 3] += perturbation0

    print obs0_xform_perturbed[:, 3]

    # update transforms
    obs0_kinbody.SetTransform(obs0_xform_perturbed)

    # check if each trajectory is still safe
    prob.SetRobotActiveDOFs()
    if not traj_is_safe([joint_target], robot):
        cc_traj_collisions += 1

    raw_input()

# print out results
print "Out of %d trials:" % N
print "\tCollided %d times, rate %f" % (
    cc_traj_collisions, (1.0 * cc_traj_collisions) / (1.0 * N))
