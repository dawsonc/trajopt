#include "trajopt/collision_terms.hpp"
#include "trajopt/collision_checker.hpp"
#include "trajopt/rave_utils.hpp"
#include "trajopt/utils.hpp"
#include "sco/expr_vec_ops.hpp"
#include "sco/expr_ops.hpp"
#include "sco/sco_common.hpp"
#include <boost/foreach.hpp>
#include "utils/eigen_conversions.hpp"
#include "sco/modeling_utils.hpp"
#include "utils/stl_to_string.hpp"
#include "utils/logging.hpp"
#include <boost/functional/hash.hpp>
using namespace OpenRAVE;
using namespace sco;
using namespace util;
using namespace std;

namespace trajopt {


void CollisionsToDistances(const vector<Collision>& collisions, const Link2Int& m_link2ind,
    DblVec& dists) {
  // Note: this checking (that the links are in the list we care about) is probably unnecessary
  // since we're using LinksVsAll
  dists.clear();
  dists.reserve(collisions.size());
  BOOST_FOREACH(const Collision& col, collisions) {
    Link2Int::const_iterator itA = m_link2ind.find(col.linkA);
    Link2Int::const_iterator itB = m_link2ind.find(col.linkB);
    if (itA != m_link2ind.end() || itB != m_link2ind.end()) {
      dists.push_back(col.distance);
    }
  }
}

void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad,
    const Link2Int& link2ind, const VarVector& vars, const DblVec& dofvals, vector<AffExpr>& exprs, bool isTimestep1) {

  exprs.clear();
  exprs.reserve(collisions.size());
  rad.SetDOFValues(dofvals); // since we'll be calculating jacobians
  BOOST_FOREACH(const Collision& col, collisions) {
    AffExpr dist(col.distance);
    Link2Int::const_iterator itA = link2ind.find(col.linkA);
    if (itA != link2ind.end()) {
      VectorXd dist_grad = toVector3d(col.normalB2A).transpose()*rad.PositionJacobian(itA->second, col.ptA);
      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
    }
    Link2Int::const_iterator itB = link2ind.find(col.linkB);
    if (itB != link2ind.end()) {
      VectorXd dist_grad = -toVector3d(col.normalB2A).transpose()*rad.PositionJacobian(itB->second, (isTimestep1 && (col.cctype == CCType_Between)) ? col.ptB1 : col.ptB);
      exprInc(dist, varDot(dist_grad, vars));
      exprInc(dist, -dist_grad.dot(toVectorXd(dofvals)));
    }
    if (itA != link2ind.end() || itB != link2ind.end()) {
      exprs.push_back(dist);
    }
  }
  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}

void CollisionsToDistanceExpressions(const vector<Collision>& collisions, Configuration& rad, const Link2Int& link2ind,
    const VarVector& vars0, const VarVector& vars1, const DblVec& vals0, const DblVec& vals1,
    vector<AffExpr>& exprs) {
  vector<AffExpr> exprs0, exprs1;
  CollisionsToDistanceExpressions(collisions, rad, link2ind, vars0, vals0, exprs0, false);
  CollisionsToDistanceExpressions(collisions, rad, link2ind, vars1, vals1, exprs1,true);

  exprs.resize(exprs0.size());
  for (int i=0; i < exprs0.size(); ++i) {
    exprScale(exprs0[i], (1-collisions[i].time));
    exprScale(exprs1[i], collisions[i].time);
    exprs[i] = AffExpr(0);
    exprInc(exprs[i], exprs0[i]);
    exprInc(exprs[i], exprs1[i]);
    cleanupAff(exprs[i]);
  }
}

inline size_t hash(const DblVec& x) {
  return boost::hash_range(x.begin(), x.end());
}

void CollisionEvaluator::GetCollisionsCached(const DblVec& x, vector<Collision>& collisions) {
  double key = hash(getDblVec(x, GetVars()));
  vector<Collision>* it = m_cache.get(key);
  if (it != NULL) {
    LOG_DEBUG("using cached collision check\n");
    collisions = *it;
  }
  else {
    LOG_DEBUG("not using cached collision check\n");
    CalcCollisions(x, collisions);
    m_cache.put(key, collisions);
  }
}

SingleTimestepCollisionEvaluator::SingleTimestepCollisionEvaluator(ConfigurationPtr rad, const VarVector& vars) :
  m_env(rad->GetEnv()),
  m_cc(CollisionChecker::GetOrCreate(*m_env)),
  m_rad(rad),
  m_vars(vars),
  m_link2ind(),
  m_links(),
  m_filterMask(-1) {
  vector<KinBody::LinkPtr> links;
  vector<int> inds;
  rad->GetAffectedLinks(m_links, true, inds);
  for (int i=0; i < m_links.size(); ++i) {
    m_link2ind[m_links[i].get()] = inds[i];
  }
  // TODO add argument
}


void SingleTimestepCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Collision>& collisions) {
  DblVec dofvals = getDblVec(x, m_vars);
  m_rad->SetDOFValues(dofvals);
  m_cc->LinksVsAll(m_links, collisions, m_filterMask);
}

void SingleTimestepCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  CollisionsToDistances(collisions, m_link2ind, dists);
}


void SingleTimestepCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  DblVec dofvals = getDblVec(x, m_vars);
  CollisionsToDistanceExpressions(collisions, *m_rad, m_link2ind, m_vars, dofvals, exprs, false);
}

////////////////////////////////////////

CastCollisionEvaluator::CastCollisionEvaluator(ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1) :
  m_env(rad->GetEnv()),
  m_cc(CollisionChecker::GetOrCreate(*m_env)),
  m_rad(rad),
  m_vars0(vars0),
  m_vars1(vars1),
  m_link2ind(),
  m_links() {
  vector<KinBody::LinkPtr> links;
  vector<int> inds;
  rad->GetAffectedLinks(m_links, true, inds);
  for (int i=0; i < m_links.size(); ++i) {
    m_link2ind[m_links[i].get()] = inds[i];
  }
}

void CastCollisionEvaluator::CalcCollisions(const DblVec& x, vector<Collision>& collisions) {
  DblVec dofvals0 = getDblVec(x, m_vars0);
  DblVec dofvals1 = getDblVec(x, m_vars1);
  m_rad->SetDOFValues(dofvals0);
  m_cc->CastVsAll(*m_rad, m_links, dofvals0, dofvals1, collisions);
}
void CastCollisionEvaluator::CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  DblVec dofvals0 = getDblVec(x, m_vars0);
  DblVec dofvals1 = getDblVec(x, m_vars1);
  CollisionsToDistanceExpressions(collisions, *m_rad, m_link2ind, m_vars0, m_vars1, dofvals0, dofvals1, exprs);
}
void CastCollisionEvaluator::CalcDists(const DblVec& x, DblVec& dists) {
  vector<Collision> collisions;
  GetCollisionsCached(x, collisions);
  CollisionsToDistances(collisions, m_link2ind, dists);
}


//////////////////////////////////////////



typedef OpenRAVE::RaveVector<float> RaveVectorf;

void PlotCollisions(const std::vector<Collision>& collisions, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles, double safe_dist) {
  BOOST_FOREACH(const Collision& col, collisions) {
    RaveVectorf color;
    if (col.distance < 0) color = RaveVectorf(1,0,0,1);
    else if (col.distance < safe_dist) color = RaveVectorf(1,1,0,1);
    else color = RaveVectorf(0,1,0,1);
    if (col.cctype == CCType_Between) {
      handles.push_back(env.drawarrow(col.ptB, col.ptB1, .002, RaveVectorf(0,0,0,1)));
    }
    OR::Vector ptB = (col.cctype == CCType_Between)  ? ((1-col.time)* col.ptB +col.time*col.ptB1) : col.ptB;
    handles.push_back(env.drawarrow(col.ptA, ptB, .0025, color));
  }
}

CollisionCost::CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars) :
    Cost("collision"),
    m_calc(new SingleTimestepCollisionEvaluator(rad, vars)), m_dist_pen(dist_pen), m_coeff(coeff)
{}

CollisionCost::CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1) :
    Cost("cast_collision"),
    m_calc(new CastCollisionEvaluator(rad, vars0, vars1)), m_dist_pen(dist_pen), m_coeff(coeff)
{}
ConvexObjectivePtr CollisionCost::convex(const vector<double>& x, Model* model) {
  ConvexObjectivePtr out(new ConvexObjective(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);
  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
    out->addHinge(viol, m_coeff);
  }
  return out;
}
double CollisionCost::value(const vector<double>& x) {
  DblVec dists;
  m_calc->CalcDists(x, dists);
  double out = 0;
  for (int i=0; i < dists.size(); ++i) {
    out += pospart(m_dist_pen - dists[i]) * m_coeff;
  }
  return out;
}

void CollisionCost::Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles) {
  vector<Collision> collisions;
  m_calc->GetCollisionsCached(x, collisions);
  PlotCollisions(collisions, env, handles, m_dist_pen);
}

// ALMOST EXACTLY COPIED FROM CollisionCost

CollisionConstraint::CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars) :
    m_calc(new SingleTimestepCollisionEvaluator(rad, vars)), m_dist_pen(dist_pen), m_coeff(coeff)
{
  name_="collision";
}

CollisionConstraint::CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1) :
    m_calc(new CastCollisionEvaluator(rad, vars0, vars1)), m_dist_pen(dist_pen), m_coeff(coeff)
{
  name_="collision";
}
ConvexConstraintsPtr CollisionConstraint::convex(const vector<double>& x, Model* model) {
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  vector<AffExpr> exprs;
  m_calc->CalcDistExpressions(x, exprs);
  for (int i=0; i < exprs.size(); ++i) {
    AffExpr viol = exprSub(AffExpr(m_dist_pen), exprs[i]);
    out->addIneqCnt(exprMult(viol,m_coeff));
  }
  return out;
}
DblVec CollisionConstraint::value(const vector<double>& x) {
  DblVec dists;
  m_calc->CalcDists(x, dists);
  DblVec out(dists.size());
  for (int i=0; i < dists.size(); ++i) {
    out[i] = pospart(m_dist_pen - dists[i]) * m_coeff;
  }
  return out;
}

/////////////////////////////////////////////////////////
// Risk-aware section starts here.
/////////////////////////////////////////////////////////

/**
 * Constructor for the collision chance constraint
 *
 * @param uncertain_body_names: the names of the bodies representing uncertain obstacles
 * @param location_covariances: the covariance matrices of the Gaussian uncertainty in the location of the
 *                              corresponding uncertain bodies. Must have the same length as uncertain_body_names
 * @param risk_tolerance: a double representing the maximum allowable risk at this time step.
 * @param precision: a double representing the desired precision tolerance in our risk estimates. True risk is guaranteed
 *                   not to exceed the estimate by more than precision/2.
 * @param rad: a pointer to the configuration of the robot and evironment.
 * @param vars: a vector of variables to be used in constructing symbolic expressions for linearized risk.
 */
CollisionChanceConstraint::CollisionChanceConstraint(std::vector<std::string> uncertain_body_names,
                                                     std::vector<Matrix3d> location_covariances,
                                                     double risk_tolerance,
                                                     double required_precision,
                                                     ConfigurationPtr rad,
                                                     const VarVector& vars) :
    m_calc(new SingleTimestepCollisionRiskEvaluator(uncertain_body_names, location_covariances, required_precision, rad, vars)),
    m_uncertain_body_names(uncertain_body_names),
    m_location_covariances(location_covariances),
    m_risk_tolerance(risk_tolerance)
{
  name_="collision";
}


/**
 * Convexify the constraint with the given values for the degrees of freedom, yielding symbolic expressions in terms
 * of the given model
 *
 * @param x: a vector of doubles containing the current values of the degrees of freedom (i.e. \theta_0)
 * @param model: the model with which to construct the convexified expressions.
 *
 * @returns a pointer to the convexified constraint.
 */
ConvexConstraintsPtr CollisionChanceConstraint::convex(const vector<double>& x, Model* model) {
  // Set up the convexified constraint object
  ConvexConstraintsPtr out(new ConvexConstraints(model));

  // Get the linearized risk expressions
  vector<AffExpr> exprs;
  m_calc->CalcRiskExpressions(x, exprs);

  // Accumulate the risks into a single constraint
  AffExpr risk_violation(-1.0*m_risk_tolerance);
  for (int i=0; i < exprs.size(); ++i) {
    // Add this risk expression to the risk violation expression
    exprInc(risk_violation, exprs[i]);
  }
  // Add the accumulated constraint to the model
  out->addIneqCnt(risk_violation);
  return out;
}

/**
 * Evaluate the constraint with the given values for the degrees of freedom.
 *
 * @param x: a vector of doubles containing the current values of the degrees of freedom (i.e. \theta_0)
 *
 * @returns a vector of doubles with one element representing the total risk at this timestep:
 *
 * The constraint is (summed over all obstacles)
 *          risk_tolerance - risk >= 0
 * but we relax it using the hingle loss to be
 *          |risk - risk_tolerance|^+
 */
DblVec CollisionChanceConstraint::value(const vector<double>& x) {
  // Start by getting the risk estimates
  DblVec risks;
  m_calc->CalcRisks(x, risks);

  // Now compute the relaxed hinge-loss penalties
  DblVec out(1);
  out[0] = 0;
  for (int i=0; i < risks.size(); ++i) {
    out[0] += pospart(risks[i] - m_risk_tolerance);
  }
  LOG_DEBUG("+++++ RISK VIOLATION %.8f", out[0]);
  return out;
}


/**
 * Constructor for the SingleTimestepCollisionRiskEvaluator class, which is an object used to evaluate
 * the risk of collision between the robot and all uncertain obstacles at a single timestep.
 *
 * @param uncertain_body_names: the names of the bodies representing uncertain obstacles
 * @param location_covariances: the covariance matrices of the Gaussian uncertainty in the location of the
 *                              corresponding uncertain bodies. Must have the same length as uncertain_body_names
 * @param precision: a double representing the desired precision tolerance in our risk estimates. True risk is guaranteed
 *                   not to exceed the estimate by more than precision/2.
 * @param rad: a pointer to the configuration of the robot and evironment.
 * @param vars: a vector of variables to be used in constructing symbolic expressions for linearized risk.
 */
SingleTimestepCollisionRiskEvaluator::SingleTimestepCollisionRiskEvaluator(
    std::vector<std::string> uncertain_body_names,
    std::vector<Matrix3d> location_covariances,
    double precision,
    ConfigurationPtr rad,
    const VarVector& vars) :
  m_env(rad->GetEnv()),
  m_cc(CollisionChecker::GetOrCreate(*m_env)),
  m_rad(rad),
  m_vars(vars),
  m_link2ind(),
  m_link2covariance(),
  m_links(),
  m_filterMask(RobotFilter),
  m_precision(precision)
{
  // Find uncertain links by body name
  vector<int> inds;
  vector<Matrix3d> covariances;
  int body_index = 0;

  // Match links to covariances
  BOOST_FOREACH(std::string body_name, uncertain_body_names) {
    BOOST_FOREACH(const KinBody::LinkPtr& link, m_env->GetKinBody(body_name)->GetLinks()) {
      if (!link->GetGeometries().empty()) {
        m_links.push_back(link);
        inds.push_back(link->GetIndex());
        covariances.push_back(location_covariances[body_index]);
      }
    }

    ++body_index;
  }

  // For easy access, save the OpenRAVE index and covariance matrix of each link in a map
  for (int i=0; i < m_links.size(); ++i) {
    m_link2ind[m_links[i].get()] = inds[i];
    m_link2covariance[m_links[i].get()] = covariances[i];
  }
}


/**
 * Serves as a wrapper for the PerformRiskCheck function, caching results of previous queries to give
 * performance benefits.
 *
 * @param x: a vector of doubles containing the current values of the degrees of freedom (i.e. \theta_0)
 * @param risks: a vector of RiskQueryResult structs into which we will put the results of the risk query.
 *               These structs contain both the risk estimate itself and information for computing the gradient.
 *
 * @returns void, but risks will be cleared and then filled with the risk query results.
 */
void CollisionRiskEvaluator::GetRisksCached(const DblVec& x, vector<RiskQueryResult>& risks) {
  double key = hash(getDblVec(x, GetVars()));
  vector<RiskQueryResult>* it = m_cache.get(key);
  if (it != NULL) {
    LOG_DEBUG("using cached risk query result\n");
    risks = *it;
  }
  else {
    LOG_DEBUG("not using cached risk query result\n");
    PerformRiskCheck(x, risks);
    m_cache.put(key, risks);
  }
}

/**
 * Calls the ProximityAlert risk estimation via the collision checker to estimate the risks of collision
 * between the robot and uncertain obstacles.
 *
 * @param x: a vector of doubles containing the current values of the degrees of freedom (i.e. \theta_0)
 * @param risks: a vector of RiskQueryResult structs into which we will put the results of the risk query.
 *               These structs contain both the risk estimate itself and information for computing the gradient.
 *
 * @returns void, but risks will be cleared and then filled with the risk query results.
 */
void SingleTimestepCollisionRiskEvaluator::PerformRiskCheck(const DblVec& x, vector<RiskQueryResult>& risks) {
  DblVec dofvals = getDblVec(x, m_vars);
  m_rad->SetDOFValues(dofvals);
  m_cc->LinksVsAllRiskEstimate(m_links, risks, m_filterMask, m_link2covariance, m_precision);
}

/**
 * Compute the risks of collision between the robot and each uncertain obstacle. Will use cached values if
 * possible; otherwise, will use the ProximityAlert epsilon-shadow algorithm to estimate the risk.
 *
 * @param x: a vector of doubles containing the current values of the degrees of freedom (i.e. \theta_0)
 * @param risks: a vector of doubles into which we will put the risk estimates
 *
 * @returns void, but risks will be cleared and then filled with the risk estimates.
 */
void SingleTimestepCollisionRiskEvaluator::CalcRisks(const DblVec& x, DblVec& risks) {
  vector<RiskQueryResult> risk_query_results;
  GetRisksCached(x, risk_query_results);

  // Extract risk measures from risk_query_results
  risks.clear();
  BOOST_FOREACH(const RiskQueryResult& risk_result, risk_query_results) {
    risks.push_back(risk_result.epsilon);
  }
}

/**
 * Construct symbolic expressions for the linearized collision risks at this single timestep.
 *
 * Begins by computing the risk of collision between each uncertain obstacle and the robot at this specific
 * timestep, then for each obstacle, computes the gradient of the risk with respect to joint values and
 * adds a symbolic affine expression to exprs of the form:
 *
 *    \epsilon_0 + \nabla_\theta \epsilon (\theta - \theta_0)
 *
 * where \epsilon_0 is the risk of collision with the obstacle, and \theta_0 is the vector of joint values at
 * this timestep, \nabla_\theta \epsilon is the gradient of \epsilon w.r.t. the joint values, and \theta is a
 * symbolic vector of joint values.
 *
 * @param x: a vector of doubles containing the current values of the degrees of freedom (i.e. \theta_0)
 * @param exprs: a vector of symbolic affine expressions into which we will put the linearized risk expressions
 *
 * @returns void, but exprs will be cleared and then filled with the linearized risk expressions.
 */
void SingleTimestepCollisionRiskEvaluator::CalcRiskExpressions(const DblVec& x, vector<AffExpr>& exprs) {
  vector<RiskQueryResult> risk_query_results;
  GetRisksCached(x, risk_query_results);
  DblVec dofvals = getDblVec(x, m_vars);

  exprs.clear();
  exprs.reserve(risk_query_results.size());
  m_rad->SetDOFValues(dofvals); // since we'll be calculating jacobians

  // For each risk result, we need to compute the linearized expression for the collision risk
  // risk(theta) \approx risk_0 + gradient * (theta - theta_0)
  BOOST_FOREACH(const RiskQueryResult& risk_result, risk_query_results) {
    AffExpr risk(risk_result.epsilon);

    // Get the indices of the two robot links that the risk estimate collides with
    Link2Int::const_iterator itRobotOneShot = m_link2ind.find(risk_result.linkRobotOneShot);
    Link2Int::const_iterator itRobotTwoShot = m_link2ind.find(risk_result.linkRobotTwoShot);

    // If the risk is extremely small, that means that the largest epsilon shadow used to estimate
    // the risk did not touch the robot. In that case, the risk is constant at zero, since small motions
    // are assumed not to bring the robot "into risk" from zero-risk.
    // Because of this, we only add the linear terms if the risk is non-zero (within tolerance).
    // We can also skip this if both contact links are not valid
    if (risk_result.epsilon > m_precision && (itRobotOneShot != m_link2ind.end() || itRobotTwoShot != m_link2ind.end())) {
      
      // There are three cases:
      //  1.) neither the one shot nor the two shot search ended in collision with the controlled links
      //  2.) only the one shot search ended in collision
      //  3.) both ended in collision
      // (collision may have occured with an uncontrolled link, like the robot base).
      //
      // If (1) occured, then neither risk_result.linkRobotOneShot nor risk_result.linkRobotTwo
      // are valid.
      //
      // If (2) occured, then risk_result.linkRobotOneShot is a valid link but
      // risk_result.linkRobotTwoShot is not, so we only add the gradient terms from the first shot
      //
      // If (3) occured, then both risk_result.linkRobotOneShot and risk_result.linkRobotTwoShot
      // will be valid links, so we add the gradient terms from both.

      // In either case 2 or case 3, we include the terms from the first shot, so we can calculate
      // the gradient: d_epsilon_d_theta (epsilon is the risk estimate and theta the joint angles).
      //
      // The explanation of this calculation relies on some knowledge of the proximity alert algorithms.
      // The ProximityAlert risk estimator gives us d_epsilon_d_x for both the one and two shot estimates.
      // In this context, x is the vector from the center of the epsilon-shadow ellipsoid to the contact point
      // with the robot, n is a vector into the robot normal to the contact with the shadow, and J is the Jacobian
      // of the contact point w.r.t. the joint angles. Then, we have
      //
      // d_epsilon_d_theta = d_epsilon_d_x * n * n^T * J
      //
      // First we need to make sure the contact unit vector is normalized
      Vector3d n_hat_one_shot = risk_result.contact_normal_into_robot_one_shot.normalized();

      VectorXd one_shot_jacobian = m_rad->PositionJacobian(itRobotOneShot->second, risk_result.ptRobotOneShot);
      VectorXd d_epsilon_d_theta_one_shot = risk_result.d_epsilon_dx_one_shot * n_hat_one_shot * n_hat_one_shot.transpose() * one_shot_jacobian;

      // We need to check if we're in case 3 before calculating the gradient for the second shot
      VectorXd risk_grad;
      if (itRobotTwoShot != m_link2ind.end()) {
        // make sure the contact unit vector is normalized
        Vector3d n_hat_two_shot = risk_result.contact_normal_into_robot_two_shot.normalized();

        VectorXd two_shot_jacobian = m_rad->PositionJacobian(itRobotTwoShot->second, risk_result.ptRobotTwoShot);
        VectorXd d_epsilon_d_theta_two_shot = risk_result.d_epsilon_dx_two_shot * n_hat_two_shot * n_hat_two_shot.transpose() * two_shot_jacobian;

        // Because the combined one- and two-shot risk estimate is epsilon = 0.5*(epsilon_1 + epsilon_2),
        // The combined gradient is the sum of half of each separate gradient
        risk_grad = 0.5 * d_epsilon_d_theta_one_shot + 0.5 * d_epsilon_d_theta_two_shot;
      } else {
        // Otherwise, we're in case 2 and the gradient is just the gradient from the first shot
        risk_grad = d_epsilon_d_theta_one_shot;
      }

      // Now we add risk_grad * (theta - theta_0) to the affine expression
      exprInc(risk, varDot(risk_grad, m_vars));
      exprInc(risk, -risk_grad.dot(toVectorXd(dofvals)));
    }
    
    // And return the risk expression by pushing the affine expression into the supplied vector.
    exprs.push_back(risk);
  }
  LOG_DEBUG("%ld distance expressions\n", exprs.size());
}


}
