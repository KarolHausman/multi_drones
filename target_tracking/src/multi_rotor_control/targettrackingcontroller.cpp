#include "targettrackingcontroller.h"

#include "motionmodel.h"
#include "sensormodel.h"
#include "ekf.h"
#include <nlopt.h>
#include <iostream>
#include <cfloat>

namespace ranav {

TargetTrackingController::TargetTrackingController() {
}

TargetTrackingController::~TargetTrackingController() {
}

void TargetTrackingController::init(const TParam &p) {
  params = p;
  topo.init(p);
}

double f_evaluate(unsigned n, const double *x, double *grad, void *data) {
  TargetTrackingController::Evaluator *o = (TargetTrackingController::Evaluator*)data;
  Eigen::VectorXd p = Eigen::Map<const Eigen::VectorXd>(x, n);
  if (grad) {
    Eigen::VectorXd g = o->gradient(p);
    assert(g.size() == n);
    memcpy(grad, g.data(), n*sizeof(double));
  }
  return o->evaluate(p);
}

double TargetTrackingController::Evaluator::evaluate(const Eigen::VectorXd &control) const {
  std::vector<Eigen::MatrixXd> traj;
  EKF e(*ekf);
  for (int i=0; i<horizon; ++i) {
    e.predict(control);
    for (std::vector<const SensorModel*>::const_iterator it = sensorModel.begin();
        it != sensorModel.end(); ++it) {
      if ((*it)->measurementAvailable(e.getMean())) {
        e.correct((*it)->sense(e.getMean()), *(*it));
      }
    }
    traj.push_back(e.getCovariance());
  }
  double traceCost = 0;
  double discount = 1;
  assert(nT == 1);
  for (std::vector<Eigen::MatrixXd>::const_iterator it = traj.begin();
      it != traj.end(); ++it) {
    traceCost += discount * it->block(motionModel->getAgentStateDim()*nA, motionModel->getAgentStateDim()*nA, 2, 2).trace();
    discount *= discountFactor;
  }
  return traceCost;
}

Eigen::VectorXd TargetTrackingController::Evaluator::gradient(const Eigen::VectorXd &p) const {
  Eigen::VectorXd eps = Eigen::VectorXd::Constant(p.size(), 1E-6);
  Eigen::VectorXd gradient(p.size());
  Eigen::VectorXd pp = p;
  for (unsigned int i=0; i<p.size(); ++i) {
    pp(i) += eps(i);
    double plus = evaluate(pp);
    pp(i) = p(i) - eps(i);
    double minus = evaluate(pp);
    pp(i) = p(i);
    gradient(i) = (plus-minus)/(2*eps(i));
  }
  return gradient;
}

Eigen::VectorXd TargetTrackingController::getControl(const EKF *ekf, const MultiAgentMotionModel *motionModel, const std::vector<const SensorModel*> &sensorModel, double *f) const {
  Evaluator evaluator(ekf, motionModel, sensorModel, params);

  Eigen::VectorXd p = Eigen::VectorXd::Zero(motionModel->getControlDim());
  Eigen::VectorXd lowerBound = Eigen::VectorXd::Constant(motionModel->getControlDim(), params("multi_rotor_control/controlMin").toDouble());
  Eigen::VectorXd upperBound = Eigen::VectorXd::Constant(motionModel->getControlDim(), params("multi_rotor_control/controlMax").toDouble());

  nlopt_opt opt = nlopt_create(NLOPT_LN_COBYLA, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_LN_BOBYQA, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_LN_NEWUOA_BOUND, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_LN_PRAXIS, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_LN_NELDERMEAD, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_LN_SBPLX, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_GN_ORIG_DIRECT, p.size()); // failed
//  nlopt_opt opt = nlopt_create(NLOPT_GN_ORIG_DIRECT_L, p.size()); // very good: p    0.0118546 -6.27225e-05  6.27225e-05 -2.09075e-05  2.09075e-05 -8.51788e-06 -2.09075e-05           10
//  nlopt_opt opt = nlopt_create(NLOPT_GN_ISRES, p.size()); // rather bad
//  nlopt_opt opt = nlopt_create(NLOPT_GN_CRS2_LM, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_LD_MMA, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_LD_CCSAQ, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_LD_SLSQP, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_LD_LBFGS, p.size());
//  nlopt_opt opt = nlopt_create(NLOPT_LD_TNEWTON_PRECOND, p.size()); // bad
//  nlopt_opt opt = nlopt_create(NLOPT_LD_TNEWTON_PRECOND_RESTART, p.size()); // bad
//  nlopt_opt opt = nlopt_create(NLOPT_LD_VAR2, p.size());

  nlopt_set_min_objective(opt, f_evaluate, &evaluator);
  nlopt_set_lower_bounds(opt, lowerBound.data());
  nlopt_set_upper_bounds(opt, upperBound.data());
  nlopt_set_ftol_abs(opt, 1E-6);
  nlopt_set_xtol_rel(opt, 1E-3);
  nlopt_set_maxeval(opt, 1E8);
  nlopt_set_maxtime(opt, 7200);
  double pa[p.size()];
  memcpy(pa, p.data(), p.size()*sizeof(double));
  double cost = 0;
//  std::string tmp; std::cerr << "Press enter to start optimization\n"; std::getline(std::cin, tmp);
  nlopt_result ret = nlopt_optimize(opt, pa, &cost);
  Eigen::VectorXd p_res = Eigen::Map<Eigen::VectorXd>(pa, p.size());
  if (f)
    *f = cost;

  std::cerr << "\nInitial guess:\n";
  std::cerr << "  p " << p.transpose() << "\n";
  std::cerr << "  value " << evaluator.evaluate(p) << "\n";

  std::cerr << "Optimization result (return code " << ret << "):\n";
  std::cerr << "  p " << p_res.transpose() << "\n";
  std::cerr << "  value " << evaluator.evaluate(p_res) << "\n";
  nlopt_destroy(opt);
  return p_res;
}

Eigen::VectorXd TargetTrackingController::getControlTopo(const EKF *ekf, const MultiAgentMotionModel *motionModel, std::vector<const SensorModel*> &sensorModel) {
  std::vector<Topology> neighbors = topo.getNeighbors();
  neighbors.push_back(topo);
  std::vector<Eigen::VectorXd> controls(neighbors.size());
  std::vector<double> cost(neighbors.size());
  int min_idx = 0;
  double min_cost = DBL_MAX;
#ifndef DEBUG
#pragma omp parallel for
#endif
  for (unsigned int i=0; i<neighbors.size(); ++i) {
    controls[i] = getControl(ekf, motionModel, neighbors[i].getSensorModels(), &cost[i]);
  }
  for (unsigned int i=0; i<neighbors.size(); ++i) {
    if (cost[i] < min_cost) {
      min_idx = i;
      min_cost = cost[i];
    }
  }
  topo = neighbors[min_idx];
  sensorModel = topo.getSensorModels();
  return controls[min_idx];
}

} /* namespace ranav */
