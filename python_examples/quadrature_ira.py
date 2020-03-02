import numpy as np
import itertools
import json
import trajoptpy


def generate_3dof_quadrature_samples(sigma):
    """Takes a a diagonal covariance
    matrix sigma (assumes that all dimensions are independent).

    Returns a list of sampled perturbations and corresponding weights for
    the Gauss-Hermite quadrature integral approximation.

    Based off of Sylvie's code for her MS thesis work
    """
    # We're working with rigid bodies with uncertain location, so we have
    # 3 dimensions to consider
    DIMENSIONS = 3
    assert (DIMENSIONS, DIMENSIONS) == sigma.shape, \
        "sigma must be {}x{}".format(DIMENSIONS, DIMENSIONS)

    weighted_samples = []  # to accumulate weighted samples for each dimension
    for dim in range(DIMENSIONS):
        # We're following Sylvie's approach here by using a 3-point quadrature
        dimension_samples = []
        # The first sample for each variable is just its nominal value
        dimension_samples.append((0, 2.0 / 3))
        # The other two samples are at +/- a multiple of the standard deviation
        std_deviation = sigma[dim][dim] ** 0.5
        GH_offset = std_deviation * (6**0.5) / 2.0
        dimension_samples.append((GH_offset, 1.0 / 6))
        dimension_samples.append((-GH_offset, 1.0 / 6))

        # save the sample points for this dimension, so we can combine later
        weighted_samples.append(dimension_samples)

    # We now want to unroll the weighted_samples array into a list of tuples
    # ([sample_dim0, sample_dim1, ...], weight=\prod_{i} weight_dim_i)
    weighted_samples = itertools.product(*weighted_samples)
    weighted_samples = [(
        [s[0] for s in sample_list],
        (np.pi)**(-float(DIMENSIONS) / 2.0) * np.prod([s[1] for s in sample_list])
    ) for sample_list in weighted_samples]

    return weighted_samples


def estimate_collision_probability(configuration,
                                   robot,
                                   manipulator,
                                   env,
                                   obstacle_names,
                                   obstacle_sigmas):
    """Returns the estimated probability that the robot collides with at least
    one obstacle in the given configuration, computed using a 3-point Gauss-
    Hermite quadrature approximate integral. Takes a list of obstacle names,
    nominal location, and diagonal covariance matrices. Returns a float"""

    # set robot to specified configuration
    robot.SetDOFValues(configuration, manipulator.GetArmIndices())

    # We will assume that all obstacles are independent, and we will upper-
    # bound the probability of collision with any obstacle as the sum of the
    # probability of collision with each obstacle.
    total_collision_probability = 0.0

    # Iterate through obstacles, estimate the probability of collision with
    # each, and accumulate
    for obs_idx, obstacle_name in enumerate(obstacle_names):
        # get quadrature samples
        weighted_samples = generate_3dof_quadrature_samples(
            obstacle_sigmas[obs_idx]
        )

        # For each sample, check collision and accumulate the result
        obstacle_collision_probability = 0.0
        # Save nominal transform from OpenRAVE environment
        obstacle_body = env.GetKinBody(obstacle_name)
        original_transform = obstacle_body.GetTransform()

        for weighted_sample in weighted_samples:
            # extract sampled location and weight
            sample, weight = weighted_sample

            sampled_transform = np.copy(original_transform)
            sampled_transform[0:3, 3] += np.array(sample)
            obstacle_body.SetTransform(sampled_transform)

            # Check for collision with this perturbed obstacle
            if env.CheckCollision(robot, obstacle_body):
                obstacle_collision_probability += weight

        # reset obstacle location
        obstacle_body.SetTransform(original_transform)       
        # accumulate the collision probability
        total_collision_probability += obstacle_collision_probability

    return total_collision_probability


def estimate_collision_probability_traj(trajectory,
                                        robot,
                                        manipulator,
                                        env,
                                        obstacle_names,
                                        obstacle_sigmas):
    n_waypoints = trajectory.shape[0]
    waypoint_risk_estimates = np.zeros(n_waypoints)
    for i in range(n_waypoints):
        waypoint_risk_estimates[i] = estimate_collision_probability(
            trajectory[i],
            robot,
            manipulator,
            env,
            obstacle_names,
            obstacle_sigmas
        )
    return waypoint_risk_estimates


def construct_trajopt_request(d_list, n_steps, arm_name, joint_start, joint_target):
    """Constructs a TrajOpt planning request dictionary using the provided
    distance hit-in penalties"""
    padding = int(n_steps / 2)
    request = {
        "basic_info": {
            "n_steps": n_steps,
            "manip": arm_name,
            # DOF values at first timestep are fixed at current robot state
            "start_fixed": True
        },
        "costs": [
            {
                "type": "joint_vel",  # joint-space velocity cost
                # a list of length one is automatically expanded
                # to a list of length n_dofs
                "params": {"coeffs": [50]}
            },
            {
                "type": "collision",
                # shorten name so printed table will be prettier
                "name": "disc_coll",
                "params": {
                    "continuous": True,
                    # penalty coefficients. list of length one is automatically
                    # expanded to a list of length n_timesteps
                    "coeffs": [50],
                    # robot-obstacle distance that penalty kicks in.
                    # expands to length n_timesteps
                    "dist_pen": list(d_list)
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
            # "type": "straight_line",
            # "endpoint": joint_target
            "type": "given_traj",
            "data": [joint_start] * padding + [joint_target] * (n_steps - padding)
        }
        # END init
    }

    return request


def risk_reallocation(risk_estimates,
                      risk_limits,
                      adjustment_rate,
                      overall_risk_limit,
                      tolerance):
    """Performs the reallocation step of IRA, per Algorithm 1 in Sylvie's paper
    """
    N = len(risk_estimates)
    risk_limits_new = np.zeros(len(risk_limits))
    violated_constraints = []
    for i in range(N):
        if risk_estimates[i] > risk_limits[i]:
            violated_constraints.append(i)

        if risk_limits[i] - risk_estimates[i] > tolerance:
            # inactive constraint
            risk_limits_new[i] = adjustment_rate * risk_limits[i] + \
                (1 - adjustment_rate) * risk_estimates[i]
        else:
            # active constraint
            risk_limits_new[i] = risk_limits[i]

    residual_risk = overall_risk_limit - np.sum(risk_limits_new)
    total_violation = 0
    for i in violated_constraints:
        total_violation += risk_estimates[i] - risk_limits[i]

    for i in violated_constraints:
        risk_limits_new[i] = risk_limits[i] + residual_risk * \
            (risk_estimates[i] - risk_limits[i]) / total_violation

    return risk_limits_new


def violation(risk_estimates, risk_limits):
    """Return false if all estimates are within limits, true otherwise"""
    for i in range(len(risk_estimates)):
        if risk_estimates[i] > risk_limits[i]:
            return True
    return False


def trajopt_ira(joint_start,
                joint_target,
                n_steps,
                robot,
                manipulator,
                environment,
                overall_risk_limit,
                obstacle_names,
                obstacle_sigmas):
    """Run IRA using the TrajOpt algorithm"""
    # constants
    d_step = 0.02
    adjustment_rate = 0.9
    tolerance = 0.000001

    # Start with zero distance hit-ins and uniform risk allocation
    d_list = np.zeros(n_steps)
    risk_limits = np.zeros(n_steps) + overall_risk_limit / n_steps

    # get a seed trajectory from trajopt
    num_trajopt_runs = 0
    request = construct_trajopt_request(d_list,
                                        n_steps,
                                        manipulator.GetName(),
                                        joint_start,
                                        joint_target)
    robot.SetDOFValues(joint_start, manipulator.GetArmIndices())
    prob = trajoptpy.ConstructProblem(json.dumps(request), environment)
    trajoptpy.SetInteractive(False)
    result = trajoptpy.OptimizeProblem(prob)
    num_trajopt_runs += 1
    trajectory = result.GetTraj()

    # get risk estimates for this trajectory
    risk_estimates = estimate_collision_probability_traj(
        trajectory, robot, manipulator, environment,
        obstacle_names, obstacle_sigmas
    )
    # reallocate and reoptimize until we're out of violation
    while violation(risk_estimates, risk_limits):
        for i in range(len(risk_estimates)):
            if risk_estimates[i] > risk_limits[i]:
                # increase hit-in distance for waypionts in violation
                d_list[i] += d_step

        risk_limits = risk_reallocation(
            risk_estimates, risk_limits,
            adjustment_rate, overall_risk_limit, tolerance
        )

        request = construct_trajopt_request(d_list,
                                            n_steps,
                                            manipulator.GetName(),
                                            joint_start,
                                            joint_target)
        robot.SetDOFValues(joint_start, manipulator.GetArmIndices())
        prob = trajoptpy.ConstructProblem(json.dumps(request), environment)
        trajoptpy.SetInteractive(False)
        result = trajoptpy.OptimizeProblem(prob)
        trajectory = result.GetTraj()
        num_trajopt_runs += 1

        # re-estimate collision risk
        risk_estimates = estimate_collision_probability_traj(
            trajectory, robot, manipulator, environment,
            obstacle_names, obstacle_sigmas
        )

    print "#### IRA Finished, required %d calls to TrajOpt" % num_trajopt_runs
    request = construct_trajopt_request(d_list,
                                        n_steps,
                                        manipulator.GetName(),
                                        joint_start,
                                        joint_target)
    robot.SetDOFValues(joint_start, manipulator.GetArmIndices())
    prob = trajoptpy.ConstructProblem(json.dumps(request), environment)
    trajoptpy.SetInteractive(False)
    result = trajoptpy.OptimizeProblem(prob)

    return trajectory
