import openravepy
import trajoptpy
from trajoptpy.check_traj import traj_is_safe
import json
import time
import numpy as np
import copy

from quadrature_ira import *

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()

env = openravepy.Environment()
# env.SetViewer('qtcoin')
env.StopSimulation()
# env.Load("robots/kuka-youbot.zae")  # arm on mobile platform
env.Load("robots/mitsubishi-pa10.zae")  # standard arm
env.Load("../data/industrial_obstacle_course.xml")

# pause every iteration, until you press 'p'.
# Press escape to disable further plotting
trajoptpy.SetInteractive(False)
robot = env.GetRobots()[0]

print env.GetRobots()
print robot.GetManipulators()

arm_name = robot.GetManipulators()[0].GetName()
arm_indices = robot.GetManipulator(arm_name).GetArmIndices()
joint_start = [-0.8, 0.9, 0, 1.45, 0, 0.8, 0]
robot.SetDOFValues(joint_start,
                   arm_indices)

joint_target = [0.8, 0.8, 0, 1.8, 0, 0.6, 0]

n_steps = 10

request = {
    "basic_info": {
        "n_steps": n_steps,
        "manip": arm_name,
        # DOF values at first timestep are fixed based on current robot state
        "start_fixed": True
    },
    "costs": [
        {
            "type": "joint_vel",  # joint-space velocity cost
            # a list of length one is automatically expanded
            # to a list of length n_dofs
            "params": {"coeffs": [10]}
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
                "dist_pen": [0.01]
            }
        }
    ],
    "constraints": [
        # BEGIN joint_constraint
        {
            "type": "joint",
            "params": {"vals": joint_target}

        }
    ],
    # BEGIN init
    "init_info": {
        # straight line in joint space.
        "type": "straight_line",
        "endpoint": joint_target
    }
    # END init
}

# Convert dictionary to JSON string for TrajOpt
s = json.dumps(request)
# Create trajopt problem
prob = trajoptpy.ConstructProblem(s, env)

# time the optimization over a large number of runs
N_runs = 1
t_start = time.time()
for i in range(N_runs):
    # optimize
    result = trajoptpy.OptimizeProblem(prob)

t_elapsed_first = (time.time() - t_start) / N_runs

print "First optimization too %.3f seconds" % (t_elapsed_first * N_runs)

# save collision free seed
initial_traj = result.GetTraj()

trajoptpy.SetInteractive(False)
robot = env.GetRobots()[0]

joint_start = [-0.8, 0.9, 0, 1.45, 0, 0.8, 0]
robot.SetDOFValues(joint_start,
                   arm_indices)

request = {
    "basic_info": {
        "n_steps": n_steps,
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
                "dist_pen": [0.01]
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
                "uncertain_body_names": ['obs0', 'operator'],
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
                    [0.0001, 0.0, 0.0, 0.0001, 0.0, 0.01],
                    [0.01, 0.0, 0.0, 0.1, 0.0, 0.0001]
                ],
                # The acceptable max. risk at each waypoint. Lists of length 1
                # are expanded to n_timesteps
                "overall_risk_tolerance": 0.01,
                # 10^-6 precision
                "precision": 0.00001,
                "coeff": 1,
                "grad_scale_factor": 0.1
            }

        }
        # END chance_constraint
    ],
    # BEGIN init
    "init_info": {
        # # straight line in joint space.
        # "type": "straight_line",
        # "endpoint": joint_target
        # start from previous
        "type": "given_traj",
        "data": result.GetTraj().tolist()
    }
    # END init
}

# Convert dictionary to JSON string for TrajOpt
s = json.dumps(request)
# Create trajopt problem
prob = trajoptpy.ConstructProblem(s, env)

# time the optimization
t_start = time.time()
for i in range(N_runs):
    # optimize
    result = trajoptpy.OptimizeProblem(prob)
t_elapsed_second = (time.time() - t_start) / N_runs
# save chance-constrained trajectory
cc_traj = result.GetTraj()

print "Second optimization too %.3f seconds" % (t_elapsed_second * N_runs)

# now compare with IRA
manipulator = robot.GetManipulator(arm_name)
overall_risk_limit = 0.01
obstacle_names = ['obs0', 'operator']
obstacle_sigmas = [
    np.diag([0.0001, 0.0001, 0.01]),
    np.diag([0.01, 0.1, 0.0001]),
]
t_start = time.time()
for i in range(N_runs):
    # optimize
    ira_traj = trajopt_ira(
        joint_start,
        joint_target,
        n_steps,
        robot,
        manipulator,
        env,
        overall_risk_limit,
        obstacle_names,
        obstacle_sigmas
    )
t_elapsed_ira = (time.time() - t_start) / N_runs

print "IRA optimization too %.3f seconds" % (t_elapsed_ira * N_runs)

# Check trajectories, both initial_traj and cc_traj
# prob.SetRobotActiveDOFs()
# assert traj_is_safe(result.GetTraj(), robot)

# we do this by conducting a bunch of trials, and on each trial we
# perturb the position of the table and check if the trajectory is safe
N = 10

sigmas = [np.diag([0.0001, 0.0001, 0.01]),
          np.diag([0.01, 0.1, 0.0001])]

obs0_kinbody = env.GetKinBody('obs0')
obs0_xform_nominal = env.GetKinBody('obs0').GetTransform()
operator_kinbody = env.GetKinBody('operator')
operator_xform_nominal = env.GetKinBody('operator').GetTransform()

# time the collision checking
t_start = time.time()

initial_traj_collisions = 0
cc_traj_collisions = 0
ira_traj_collisions = 0
for i in range(N):
    print "\r" + str(i * 100.0 / N) + "%",
    # get random perturbation
    perturbation0 = np.concatenate([
        np.random.multivariate_normal([0, 0, 0], sigmas[0]),
        [1]])
    perturbation1 = np.concatenate([
        np.random.multivariate_normal([0, 0, 0], sigmas[1]),
        [1]])

    obs0_xform_perturbed = copy.copy(obs0_xform_nominal)
    obs0_xform_perturbed[:, 3] += perturbation0
    operator_xform_perturbed = copy.copy(operator_xform_nominal)
    operator_xform_perturbed[:, 3] += perturbation1

    # update transforms
    obs0_kinbody.SetTransform(obs0_xform_perturbed)
    operator_kinbody.SetTransform(operator_xform_perturbed)

    # check if each trajectory is still safe
    prob.SetRobotActiveDOFs()
    if not traj_is_safe(initial_traj, robot):
        initial_traj_collisions += 1
    if not traj_is_safe(cc_traj, robot):
        cc_traj_collisions += 1
    if not traj_is_safe(ira_traj, robot):
        ira_traj_collisions += 1

t_elapsed_check = time.time() - t_start

# print out results
# print "\n%d collision trials:" % N
# print "\tNominally collision-free trajectory collided %d times, rate %f" % (
#     initial_traj_collisions, (1.0 * initial_traj_collisions) / (1.0 * N))
# print "\tChance-constrained trajectory collided %d times, rate %f" % (
#     cc_traj_collisions, (1.0 * cc_traj_collisions) / (1.0 * N))
# print "(Collision checking took %f s)" % t_elapsed_check
def get_traj_length_rad(traj):
    """Return the length in radians of the given trajectory.
    Formally, Sum_{n=0}^{N-1} Sum_{i in DOFs}|\theta_i[n+1] - \theta_i[n]|
    """
    return np.sum(np.abs(np.diff(traj, axis=0)))


print "Num Waypoints: %d" % n_steps
print "Algorithm | Avg. Runtime (s, %d trials) | Avg. Path Length (%d trials) | True Collision Rate (%d trials, interpolated to 100 steps)" % (
    N_runs, N_runs, N)
print "      SCO | %21.3f | %26.2f | %56.2f" % (
    t_elapsed_first, get_traj_length_rad(initial_traj), (100.0 * initial_traj_collisions) / (1.0 * N))
print "      IRA | %21.3f | %26.2f | %56.2f" % (
    t_elapsed_ira, get_traj_length_rad(ira_traj), (100.0 * ira_traj_collisions) / (1.0 * N))
print "  eps-SCO | %21.3f | %26.2f | %56.2f" % (
    t_elapsed_second, get_traj_length_rad(cc_traj), (100.0 * cc_traj_collisions) / (1.0 * N))
