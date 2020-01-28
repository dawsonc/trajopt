import openravepy
import trajoptpy
import json
import numpy as np
import trajoptpy.kin_utils as ku

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--position_only", action="store_true")
args = parser.parse_args()

env = openravepy.Environment()
env.StopSimulation()
env.Load("robots/pr2-beta-static.zae")
env.Load("../data/table.xml")

# Pause every iteration, until you press 'p'.
# Press escape to disable further plotting
trajoptpy.SetInteractive(args.interactive)

robot = env.GetRobots()[0]
joint_start = [-1.832, -0.332, -1.011, -1.437, -1.1, -2.106, 3.074]
robot.SetDOFValues(joint_start,
                   robot.GetManipulator('rightarm').GetArmIndices())

quat_target = [1, 0, 0, 0]  # wxyz
xyz_target = [6.51073449e-01, -1.87673551e-01, 4.91061915e-01]
hmat_target = openravepy.matrixFromPose(np.r_[quat_target, xyz_target])

# BEGIN ik
manip = robot.GetManipulator("rightarm")
robot.SetActiveManipulator(manip)
ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(
    robot, iktype=openravepy.IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()
init_joint_target = ku.ik_for_link(
    hmat_target, manip, "r_gripper_tool_frame",
    filter_options=openravepy.IkFilterOptions.CheckEnvCollisions)
# END ik

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
            "name": "cont_coll",
            "params": {
                "continuous": True,
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
        # BEGIN pose_constraint
        {
            "type": "pose",
            "params": {"xyz": xyz_target,
                       "wxyz": quat_target,
                       "link": "r_gripper_tool_frame",
                       "timestep": 9
                       }

        },
        # END pose_constraint
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
                    [0.02, 0.0, 0.0, 0.02, 0.0, 0.02]  # 0.02 * 3x3 identity
                ],
                # The acceptable max. risk at each waypoint. Lists of length 1
                # are expanded to n_timesteps
                "waypoint_risk_tolerances": [0.01]
            }

        }
        # END chance_constraint
    ],
    # BEGIN init
    "init_info": {
        # straight line in joint space.
        "type": "straight_line",
        # need to convert numpy array to list
        "endpoint": init_joint_target.tolist()
    }
    # END init
}
