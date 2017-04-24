#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include "StateSpace.h"

class KalmanFilter
{
public:
  KalmanFilter(StateSpace *e, MatrixXf P_, MatrixXf Q_, double R_);
  double next(double output,double input);
  void reset()
  {
  }
  StateSpace *eq;

  MatrixXf Xnew;
  MatrixXf G;
private:
  MatrixXf P;
  MatrixXf Pm;
  MatrixXf Pb;
  MatrixXf Q;
  MatrixXf Buff;
  double R;
};

#endif
