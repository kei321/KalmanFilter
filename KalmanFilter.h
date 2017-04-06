#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include "StateEq.h"

class KalmanFilter
{
public:
  KalmanFilter(StateEq *e,double o, double p);
  double next(double output,double input);
  void reset()
  {
    G=0;omega=0;
  }
private:
  std::vector<double> P;
  std::vector<double> Pd;
  std::vector<double> buff;
  double G;
  double omega;
  StateEq *eq;
};

#endif
