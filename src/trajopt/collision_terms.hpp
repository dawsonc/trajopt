#pragma once
#include "trajopt/common.hpp"
#include "trajopt/collision_checker.hpp"
#include "sco/modeling.hpp"
#include "sco/sco_fwd.hpp"
#include "cache.hxx"


namespace trajopt {

typedef std::map<const OR::KinBody::Link*, int> Link2Int;
typedef std::map<const OR::KinBody::Link*, Matrix3d> Link2Matrix3d;


struct CollisionEvaluator {
  virtual void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs) = 0;
  virtual void CalcDists(const DblVec& x, DblVec& exprs) = 0;
  virtual void CalcCollisions(const DblVec& x, vector<Collision>& collisions) = 0;
  void GetCollisionsCached(const DblVec& x, vector<Collision>&);
  virtual ~CollisionEvaluator() {}
  virtual VarVector GetVars()=0;

  Cache<size_t, vector<Collision>, 3> m_cache;
};
typedef boost::shared_ptr<CollisionEvaluator> CollisionEvaluatorPtr;

struct SingleTimestepCollisionEvaluator : public CollisionEvaluator {
public:
  SingleTimestepCollisionEvaluator(ConfigurationPtr rad, const VarVector& vars);
  /**
  @brief linearize all contact distances in terms of robot dofs
  
  Do a collision check between robot and environment.
  For each contact generated, return a linearization of the signed distance function
  */
  void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs);
  /**
   * Same as CalcDistExpressions, but just the distances--not the expressions
   */
  void CalcDists(const DblVec& x, DblVec& exprs);
  void CalcCollisions(const DblVec& x, vector<Collision>& collisions);
  VarVector GetVars() {return m_vars;}

  OR::EnvironmentBasePtr m_env;
  CollisionCheckerPtr m_cc;
  ConfigurationPtr m_rad;
  VarVector m_vars;
  Link2Int m_link2ind;
  vector<OR::KinBody::LinkPtr> m_links;
  short m_filterMask;
};

struct CastCollisionEvaluator : public CollisionEvaluator {
public:
  CastCollisionEvaluator(ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1);
  void CalcDistExpressions(const DblVec& x, vector<AffExpr>& exprs);
  void CalcDists(const DblVec& x, DblVec& exprs);
  void CalcCollisions(const DblVec& x, vector<Collision>& collisions);
  VarVector GetVars() {return concat(m_vars0, m_vars1);}
  

  // parameters:
  OR::EnvironmentBasePtr m_env;
  CollisionCheckerPtr m_cc;
  ConfigurationPtr m_rad;
  VarVector m_vars0;
  VarVector m_vars1;
  typedef std::map<const OR::KinBody::Link*, int> Link2Int;
  Link2Int m_link2ind;
  vector<OR::KinBody::LinkPtr> m_links;
  short m_filterMask;

};


class TRAJOPT_API CollisionCost : public Cost, public Plotter {
public:
  /* constructor for single timestep */
  CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars);
  /* constructor for cast cost */
  CollisionCost(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1);
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  virtual double value(const vector<double>&);
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
  VarVector getVars() {return m_calc->GetVars();}
private:
  CollisionEvaluatorPtr m_calc;
  double m_dist_pen;
  double m_coeff;
};
class TRAJOPT_API CollisionConstraint : public IneqConstraint {
public:
  /* constructor for single timestep */
  CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars);
  /* constructor for cast cost */
  CollisionConstraint(double dist_pen, double coeff, ConfigurationPtr rad, const VarVector& vars0, const VarVector& vars1);
  virtual ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
  virtual DblVec value(const vector<double>&);
  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);
  VarVector getVars() {return m_calc->GetVars();}
private:
  CollisionEvaluatorPtr m_calc;
  double m_dist_pen;
  double m_coeff;
};


/////////////////////////////////////////////////////////
// Risk-aware section starts here.
/////////////////////////////////////////////////////////

struct CollisionRiskEvaluator {
  virtual void CalcRiskExpressions(const DblVec& x, vector<AffExpr>& exprs) = 0;
  virtual void CalcRisks(const DblVec& x, DblVec& exprs) = 0;
  virtual void PerformRiskCheck(const DblVec& x, vector<RiskQueryResult>& risks) = 0;
  void GetRisksCached(const DblVec& x, vector<RiskQueryResult>&);
  virtual ~CollisionRiskEvaluator() {}
  virtual VarVector GetVars()=0;

  virtual void PlotRisks(const std::vector<RiskQueryResult>& risk_query_results, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles) = 0;
  virtual void PlotRisks(const DblVec& x, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles) = 0;

  Cache<size_t, vector<RiskQueryResult>, 3> m_cache;
};
typedef boost::shared_ptr<CollisionRiskEvaluator> CollisionRiskEvaluatorPtr;


struct SingleTimestepCollisionRiskEvaluator : public CollisionRiskEvaluator {
public:
  SingleTimestepCollisionRiskEvaluator(std::vector<std::string> uncertain_body_names,
                                       std::vector<Matrix3d> location_covariances,
                                       double precision, double grad_scale_factor,
                                       ConfigurationPtr rad,
                                       const VarVector& vars);
  /**
  @brief linearize all risk estimates in terms of robot dofs
  
  Do a risk query check between robot and uncertain robots in the environment.
  For each risk estimate generated, return a linearization of the risk estimate
  */
  void CalcRiskExpressions(const DblVec& x, vector<AffExpr>& exprs);
  /**
   * Same as CalcRiskExpressions, but just the risks --not the expressions
   */
  void CalcRisks(const DblVec& x, DblVec& risks);
  void PerformRiskCheck(const DblVec& x, vector<RiskQueryResult>& risks);
  VarVector GetVars() {return m_vars;}

  void PlotRisks(const std::vector<RiskQueryResult>& risk_query_results, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles);
  void PlotRisks(const DblVec& x, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles) {}

  OR::EnvironmentBasePtr m_env;
  CollisionCheckerPtr m_cc;
  ConfigurationPtr m_rad;
  VarVector m_vars;
  Link2Int m_link2ind;
  Link2Matrix3d m_link2covariance;
  vector<OR::KinBody::LinkPtr> m_links;
  short m_filterMask;
  double m_precision;
  double m_grad_scale_factor;
};

struct OverallCollisionRiskEvaluator : public CollisionRiskEvaluator {
public:
  OverallCollisionRiskEvaluator(std::vector<std::string> uncertain_body_names,
                                std::vector<Matrix3d> location_covariances,
                                double precision, double grad_scale_factor, int numsteps,
                                ConfigurationPtr rad,
                                const VarArray& vars);
  OverallCollisionRiskEvaluator(std::vector<std::string> uncertain_body_names,
                                std::vector<Matrix3d> location_covariances,
                                double precision, double grad_scale_factor, int numsteps,
                                ConfigurationPtr rad,
                                const VarArray& vars,
                                bool use_max_risk);
  /**
  @brief linearize all risk estimates in terms of robot dofs
  
  Do a risk query check between robot and uncertain robots in the environment.
  For each risk estimate generated, return a linearization of the risk estimate
  */
  void CalcRiskExpressions(const DblVec& x, vector<AffExpr>& exprs);
  /**
   * Same as CalcRiskExpressions, but just the risks--not the expressions
   */
  void CalcRisks(const DblVec& x, DblVec& risks);
  VarVector GetVars() {return m_vars.flatten();}

  // These aren't used but must be declared
  void PerformRiskCheck(const DblVec& x, vector<RiskQueryResult>& risks) {}
  void PlotRisks(const std::vector<RiskQueryResult>& risk_query_results, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles) {}
  void PlotRisks(const DblVec& x, OR::EnvironmentBase& env, vector<OR::GraphHandlePtr>& handles);

  vector<SingleTimestepCollisionRiskEvaluator> m_timestep_risk_evals;
  int m_numsteps;

  OR::EnvironmentBasePtr m_env;
  CollisionCheckerPtr m_cc;
  ConfigurationPtr m_rad;
  VarArray m_vars;
  Link2Int m_link2ind;
  Link2Matrix3d m_link2covariance;
  vector<OR::KinBody::LinkPtr> m_links;
  short m_filterMask;
  double m_precision;
  double m_grad_scale_factor;
  bool m_max_risk;
};


class TRAJOPT_API CollisionChanceConstraint : public IneqConstraint, public Plotter {
public:
  /* constructor for single timestep */
  CollisionChanceConstraint(std::vector<std::string> uncertain_body_names,
                            std::vector<Matrix3d> location_covariances,
                            double risk_tolerance,
                            double required_precision,
                            double coeff,
                            double grad_scale_factor,
                            int num_timesteps,
                            ConfigurationPtr rad,
                            const VarArray& vars);

  virtual ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
  virtual DblVec value(const vector<double>&);

  void Plot(const DblVec& x, OR::EnvironmentBase& env, std::vector<OR::GraphHandlePtr>& handles);

  VarVector getVars() {return m_vars.flatten();}
private:
  CollisionRiskEvaluatorPtr m_calc;
  VarArray m_vars;

  double m_coeff;
  int m_numsteps;

  std::vector<std::string> m_uncertain_body_names;
  std::vector<Matrix3d> m_location_covariances;
  double m_risk_tolerance;
};

}
