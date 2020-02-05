import openravepy
import trajoptpy
from trajoptpy.check_traj import traj_is_safe
import json
import time

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()

env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
env.Load("../data/table.xml")

# pause every iteration, until you press 'p'.
# Press escape to disable further plotting
trajoptpy.SetInteractive(False)
robot = env.GetRobots()[0]

joint_start = [-1.832, -0.332, -1.011, -1.437, -1.1, -1.926,  3.074]
robot.SetDOFValues(joint_start,
                   robot.GetManipulator('rightarm').GetArmIndices())

joint_target = [0.062, 1.287, 0.1, -1.43, -3.011, -0.268, 2.988]

request = {
    "basic_info": {
        "n_steps": 10,
        "manip": "rightarm",
        # DOF values at first timestep are fixed based on current robot state
        "start_fixed": True
    },
    "costs": [
        {
            "type": "joint_vel",  # joint-space velocity cost
            # a list of length one is automatically expanded
            # to a list of length n_dofs
            "params": {"coeffs": [1]}
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
                "dist_pen": [0.05]
            }
        }
    ],
    "constraints": [
        # BEGIN joint_constraint
        {
            "type": "joint",
            "params": {"vals": joint_target}

        }
        # END joint_constraint
        # # BEGIN chance_constraint
        # {
        #     "type": "uncertain_obstacles",
        #     "name": "chance_constr",
        #     "params": {
        #         # The list of uncertain KinBodies
        #         "uncertain_body_names": ['table'],
        #         # As a reminder: covariances must be positive semidefinite
        #         # This implies that it must be symmetric, so we only specify
        #         # the six unique values (numbered below):
        #         #
        #         #       [0 1 2]
        #         #       [. 3 4]
        #         #       [. . 5]
        #         #
        #         # This list must be the same length as obstacles above
        #         "location_covariances": [
        #             [0.0001, 0.0, 0.0, 0.1, 0.0, 0.0001]
        #         ],
        #         # The acceptable max. risk at each waypoint. Lists of length 1
        #         # are expanded to n_timesteps
        #         "waypoint_risk_tolerances": [0.1],
        #         # 10^-6 precision
        #         "precision": 0.000001,
        #         "coeff": 100
        #     }

        # }
        # # END chance_constraint
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

# time the optimization
t_start = time.time()
# optimize
result = trajoptpy.OptimizeProblem(prob)
t_elapsed_first = time.time() - t_start


trajoptpy.SetInteractive(True)
robot = env.GetRobots()[0]

def generate_request(waypoint_risk_tolerances):
    request = {
        "basic_info": {
            "n_steps": 10,
            "manip": "rightarm",
            # DOF values at first timestep are fixed based on current robot state
            "start_fixed": True
        },
        "costs": [
            {
                "type": "joint_vel",  # joint-space velocity cost
                # a list of length one is automatically expanded
                # to a list of length n_dofs
                "params": {"coeffs": [1]}
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
                    "uncertain_body_names": ['table'],
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
                        [0.0001, 0.0, 0.0, 0.01, 0.0, 0.0001]
                    ],
                    # The acceptable max. risk at each waypoint. Lists of length 1
                    # are expanded to n_timesteps
                    "waypoint_risk_tolerances": waypoint_risk_tolerances,
                    # 10^-6 precision
                    "precision": 0.000001,
                    "coeff": 1,
                    "grad_scale_factor": 1000
                }

            }
            # END chance_constraint
        ],
        # BEGIN init
        "init_info": {
            # start from previous
            "type": "given_traj",
            "data": result.GetTraj().tolist()
        }
        # END init
    }
    return request

# time the optimization
t_start = time.time()
# optimize, use IRA
OVERALL_RISK_TOLERANCE = 0.1
MAX_ITERS = 10
NUM_STEPS = 10
SMOL_BIT = 0.1
waypoint_risk_tolerances = [OVERALL_RISK_TOLERANCE / NUM_STEPS] * NUM_STEPS
for i in range(MAX_ITERS):
    # Convert dictionary to JSON string for TrajOpt
    s = json.dumps(generate_request(waypoint_risk_tolerances))
    # Create trajopt problem
    prob = trajoptpy.ConstructProblem(s, env)

    # inner loop optimization
    result = trajoptpy.OptimizeProblem(prob)

    # get constraints
    constraints = result.GetConstraints()
    # get number of active constraints
    num_active = 0
    for constr in constraints:
        if constr[1] > 0:
            num_active += 1

    # nothing to be done if num_active = 0: we're all good!
    if num_active == 0:
        print "No active constraints!"
    # nothing we can do if all constraints are active, but this is sad
    # this indicates infeasibility
    elif num_active == NUM_STEPS:
        print "All constraints active; no improvement possible with IRA"


t_elapsed_second = time.time() - t_start

# Display results
print result.GetTraj()
print "======================="
print "First optimization too %.3f seconds" % t_elapsed_first
print "Second optimization too %.3f seconds" % t_elapsed_second

# Check trajectory
prob.SetRobotActiveDOFs()
assert traj_is_safe(result.GetTraj(), robot)
